// ----------------------------------------------------------------------------
// TMC429.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC429.h"


void TMC429::setup(size_t chip_select_pin,
  uint8_t clock_frequency_mhz)
{
  chip_select_pin_ = chip_select_pin;

  pinMode(chip_select_pin_,OUTPUT);
  digitalWrite(chip_select_pin_,HIGH);

  specifyClockFrequencyInMHz(clock_frequency_mhz);

  for (uint8_t motor=0; motor<MOTOR_COUNT; ++motor)
  {
    pulse_div_[motor] = 0;
    ramp_div_[motor] = 0;
  }

  spiBegin();

  setStepDiv(STEP_DIV_MAX);

  stopAll();

  initialize();
}

bool TMC429::communicating()
{
  return (getVersion() == VERSION);
}

uint32_t TMC429::getVersion()
{
  return readRegister(SMDA_COMMON, ADDRESS_TYPE_VERSION_429);
}

void TMC429::setRampMode(size_t motor)
{
  setMode(motor, RAMP_MODE);
}

void TMC429::setSoftMode(size_t motor)
{
  setMode(motor, SOFT_MODE);
}

void TMC429::setHoldMode(size_t motor)
{
  setMode(motor, HOLD_MODE);
}

void TMC429::setVelocityMode(size_t motor)
{
  setMode(motor, VELOCITY_MODE);
}

void TMC429::setLimitsInHz(size_t motor,
  uint32_t velocity_min_hz,
  uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }

  setOptimalStepDivHz(velocity_max_hz);

  setOptimalPulseDivHz(motor, velocity_max_hz);

  setVelocityMinInHz(motor, velocity_min_hz);

  setVelocityMaxInHz(motor, velocity_max_hz);

  setOptimalRampDivHz(motor, velocity_max_hz, acceleration_max_hz_per_s);

  uint32_t a_max = setAccelerationMaxInHzPerS(motor, velocity_max_hz, acceleration_max_hz_per_s);

  setOptimalPropFactor(motor, a_max);
}

uint32_t TMC429::getVelocityMaxUpperLimitInHz()
{
  return convertVelocityToHz(0, VELOCITY_REGISTER_MAX);
}

uint32_t TMC429::getVelocityMinInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(pulse_div_[motor], getVelocityMin(motor));
}

uint32_t TMC429::getVelocityMaxInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(pulse_div_[motor], getVelocityMax(motor));
}

int32_t TMC429::getTargetVelocityInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(pulse_div_[motor], getTargetVelocity(motor));
}

void TMC429::setTargetVelocityInHz(size_t motor,
  int32_t velocity_hz)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setTargetVelocity(motor, convertVelocityFromHz(pulse_div_[motor], velocity_hz));
}

int16_t TMC429::getTargetVelocity(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t velocity_unsigned = readRegister(motor, ADDRESS_V_TARGET);
  return unsignedToSigned(velocity_unsigned, V_BIT_COUNT);
}

void TMC429::setTargetVelocity(size_t motor,
  int16_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_V_TARGET, velocity);
}

bool TMC429::atTargetVelocity(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return true;
  }
  int16_t actual_velocity = getActualVelocity(motor);
  int16_t target_velocity = getTargetVelocity(motor);
  return (actual_velocity == target_velocity);
}

int32_t TMC429::getActualVelocityInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(pulse_div_[motor], getActualVelocity(motor));
}

int16_t TMC429::getActualVelocity(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t velocity_unsigned = readRegister(motor, ADDRESS_V_ACTUAL);
  return unsignedToSigned(velocity_unsigned, V_BIT_COUNT);
}

void TMC429::setHoldVelocityMaxInHz(size_t motor,
  uint32_t velocity_max_hz)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }

  setOptimalStepDivHz(velocity_max_hz);

  setOptimalPulseDivHz(motor, velocity_max_hz);

  setVelocityMaxInHz(motor, velocity_max_hz);
}

void TMC429::setHoldVelocityInHz(size_t motor,
  int32_t velocity_hz)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setHoldVelocity(motor, convertVelocityFromHz(pulse_div_[motor], velocity_hz));
}

void TMC429::setHoldVelocity(size_t motor,
  int16_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_V_ACTUAL, velocity);
}

uint32_t TMC429::getAccelerationMaxUpperLimitInHzPerS(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div;
  if (pulse_div > 0)
  {
    ramp_div = pulse_div - 1;
  }
  else
  {
    ramp_div = RAMP_DIV_MIN;
  }
  return getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);
}

uint32_t TMC429::getAccelerationMaxLowerLimitInHzPerS(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div = RAMP_DIV_MAX;
  return getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);
}

uint32_t TMC429::getAccelerationMaxInHzPerS(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertAccelerationToHzPerS(pulse_div_[motor], ramp_div_[motor], getAccelerationMax(motor));
}

uint32_t TMC429::getActualAccelerationInHzPerS(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertAccelerationToHzPerS(pulse_div_[motor], ramp_div_[motor], getActualAcceleration(motor));
}

int32_t TMC429::getTargetPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = readRegister(motor, ADDRESS_X_TARGET);
  return unsignedToSigned(position_unsigned, X_BIT_COUNT);
}

void TMC429::setTargetPosition(size_t motor,
  int32_t position)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_X_TARGET, position);
}

bool TMC429::atTargetPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return true;
  }
  int32_t actual_position = getActualPosition(motor);
  int32_t target_position = getTargetPosition(motor);
  return (actual_position == target_position);
}

int32_t TMC429::getActualPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = readRegister(motor, ADDRESS_X_ACTUAL);
  return unsignedToSigned(position_unsigned, X_BIT_COUNT);
}

void TMC429::setActualPosition(size_t motor,
  int32_t position)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_X_ACTUAL, position);
}

void TMC429::stop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setMode(motor, VELOCITY_MODE);
  setTargetVelocity(motor, 0);
}

void TMC429::stopAll()
{
  for (uint8_t motor=0; motor<MOTOR_COUNT; ++motor)
  {
    stop(motor);
  }
}

void TMC429::enableInverseStepPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_stp = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::disableInverseStepPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_stp = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::enableInverseDirPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_dir = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::disableInverseDirPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_dir = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::setSwitchesActiveLow()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_ref = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::setSwitchesActiveHigh()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_ref = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::enableLeftSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_l = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::disableLeftSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_l = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429::leftSwitchStopEnabled(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return !ref_conf_mode.ref_conf.disable_stop_l;
}

bool TMC429::leftSwitchActive(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  SwitchState switch_state = getSwitchState();
  switch (motor)
  {
    case 0:
    {
      return switch_state.l0;
      break;
    }
    case 1:
    {
      return switch_state.l1;
      break;
    }
    case 2:
    {
      return switch_state.l2;
      break;
    }
  }
  return false;
}

void TMC429::enableRightSwitches()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_refr = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::disableRightSwitches()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_refr = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

bool TMC429::rightSwitchesEnabled()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  return if_conf.if_conf.en_refr;
}

void TMC429::enableRightSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_r = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::disableRightSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_r = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429::rightSwitchStopEnabled(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return !ref_conf_mode.ref_conf.disable_stop_r;
}

bool TMC429::rightSwitchActive(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  SwitchState switch_state = getSwitchState();
  switch (motor)
  {
    case 0:
    {
      return switch_state.r0;
      break;
    }
    case 1:
    {
      return switch_state.r1;
      break;
    }
    case 2:
    {
      return switch_state.r2;
      break;
    }
  }
  return false;
}

void TMC429::enableSwitchSoftStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.soft_stop = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::disableSwitchSoftStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.soft_stop = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429::switchSoftStopEnabled(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return ref_conf_mode.ref_conf.soft_stop;
}

void TMC429::setReferenceSwitchToLeft(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.ref_rnl = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::setReferenceSwitchToRight(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.ref_rnl = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::startLatchPositionWaiting(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_X_LATCHED, 0);
}

bool TMC429::latchPositionWaiting(size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.lp = false;
  if (motor < MOTOR_COUNT)
  {
    ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.lp;
}

int32_t TMC429::getLatchPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = readRegister(motor, ADDRESS_X_LATCHED);
  return unsignedToSigned(position_unsigned, X_BIT_COUNT);
}

void TMC429::setPositionCompareMotor(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.pos_comp_sel = motor;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

TMC429::Status TMC429::getStatus()
{
  getVersion();
  return status_;
}

// private
void TMC429::initialize()
{
  setStepDirOutput();
}

void TMC429::setStepDirOutput()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_sd = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

// void TMC429::setSpiOutput()
// {
//   IfConf if_conf;
//   if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
//   if_conf.if_conf.en_sd = 0;
//   writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
// }

uint32_t TMC429::readRegister(uint8_t smda,
  uint8_t address)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.rrs = RRS_REGISTER;
  mosi_datagram.address = address;
  mosi_datagram.smda = smda;
  mosi_datagram.rw = RW_READ;
  mosi_datagram.data = 0;
  MisoDatagram miso_datagram = writeRead(mosi_datagram);
  return miso_datagram.data;
}

void TMC429::writeRegister(uint8_t smda,
  uint8_t address,
  uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.rrs = RRS_REGISTER;
  mosi_datagram.address = address;
  mosi_datagram.smda = smda;
  mosi_datagram.rw = RW_WRITE;
  mosi_datagram.data = data;
  writeRead(mosi_datagram);
}

TMC429::MisoDatagram TMC429::writeRead(MosiDatagram mosi_datagram)
{
  MisoDatagram miso_datagram;
  miso_datagram.bytes = 0x0;
  beginTransaction();
  for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
  {
    uint8_t byte_write = (mosi_datagram.bytes >> (8*i)) & 0xff;
    uint8_t byte_read = spiTransfer(byte_write);
    miso_datagram.bytes |= ((uint32_t)byte_read) << (8*i);
  }
  endTransaction();
  noInterrupts();
  status_ = miso_datagram.status;
  interrupts();
  return miso_datagram;
}

int32_t TMC429::unsignedToSigned(uint32_t input_value,
  uint8_t num_bits)
{
  uint32_t mask = 1 << (num_bits - 1);
  return -(input_value & mask) + (input_value & ~mask);
}

void TMC429::specifyClockFrequencyInMHz(uint8_t clock_frequency)
{
  if (clock_frequency <= CLOCK_FREQUENCY_MAX)
  {
    clock_frequency_ = clock_frequency;
  }
  else
  {
    clock_frequency_ = CLOCK_FREQUENCY_MAX;
  }
}

void TMC429::setOptimalStepDivHz(uint32_t velocity_max_hz)
{
  int step_div = getStepDiv();

  double step_time = stepDivToStepTime(step_div);

  uint32_t velocity_max_upper_limit = (double)MHZ_PER_HZ/(step_time*2);

  while ((velocity_max_upper_limit < velocity_max_hz) && (step_div >= 1))
  {
    --step_div;
    step_time = stepDivToStepTime(step_div);
    velocity_max_upper_limit = (double)MHZ_PER_HZ/(step_time*2);
  }

  setStepDiv(step_div);
}

uint8_t TMC429::getStepDiv()
{
  GlobalParameters global_parameters;
  global_parameters.bytes = readRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS);
  return global_parameters.clk2_div & STEP_DIV_MASK;
}

void TMC429::setStepDiv(uint8_t step_div)
{
  GlobalParameters global_parameters;
  global_parameters.bytes = readRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS);
  global_parameters.clk2_div = step_div & STEP_DIV_MASK;
  writeRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS, global_parameters.bytes);
}

double TMC429::stepDivToStepTime(uint8_t step_div)
{
  double step_time = (double)(16*(1 + step_div))/(double)clock_frequency_;
  return step_time;
}

int32_t TMC429::convertVelocityToHz(uint8_t pulse_div,
  int16_t velocity)
{
  // (clock_frequency_*MHZ_PER_HZ*velocity)/((1 << pulse_div)*VELOCITY_CONSTANT);
  double x = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)VELOCITY_CONSTANT;
  double y = (x*(double)velocity)/((double)(1 << pulse_div));
  return y;
}

int16_t TMC429::convertVelocityFromHz(uint8_t pulse_div,
  int32_t velocity)
{
  // (velocity*(1 << pulse_div)*VELOCITY_CONSTANT)/(clock_frequency_*MHZ_PER_HZ);
  double x = ((double)velocity*(double)(1 << pulse_div))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double y = x*(double)VELOCITY_CONSTANT;
  return y;
}

uint8_t TMC429::findOptimalPulseDivHz(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = PULSE_DIV_MAX + 1;
  uint32_t velocity_max_upper_limit = 0;
  while ((velocity_max_upper_limit < velocity_max_hz) && (pulse_div >= 1))
  {
    --pulse_div;
    velocity_max_upper_limit = getVelocityMaxUpperLimitInHz(pulse_div);
  }
  return pulse_div;
}

void TMC429::setOptimalPulseDivHz(size_t motor,
  uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  ClkConfig clk_config;
  clk_config.bytes = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  clk_config.clk_config.pulse_div = pulse_div;
  writeRegister(motor, ADDRESS_CLOCK_CONFIGURATION, clk_config.bytes);
  pulse_div_[motor] = pulse_div;
}

TMC429::Mode TMC429::getMode(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return RAMP_MODE;
  }

  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  switch (ref_conf_mode.mode)
  {
    case RAMP_MODE:
      return RAMP_MODE;
      break;
    case SOFT_MODE:
      return SOFT_MODE;
      break;
    case VELOCITY_MODE:
      return VELOCITY_MODE;
      break;
    case HOLD_MODE:
      return HOLD_MODE;
      break;
  }
  return RAMP_MODE;
}

void TMC429::setMode(size_t motor,
  Mode mode)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }

  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.mode = (uint8_t)mode;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

TMC429::ReferenceConfiguration TMC429::getReferenceConfiguration(size_t motor)
{
  RefConfMode ref_conf_mode;
  if (motor < MOTOR_COUNT)
  {
    ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.ref_conf;
}

TMC429::InterfaceConfiguration TMC429::getInterfaceConfiguration()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  return if_conf.if_conf;
}

TMC429::SwitchState TMC429::getSwitchState()
{
  SwState switch_state;
  switch_state.bytes = readRegister(SMDA_COMMON, ADDRESS_SWITCHES);
  return switch_state.switch_state;
}

TMC429::ClockConfiguration TMC429::getClockConfiguration(size_t motor)
{
  ClkConfig clk_config;
  if (motor < MOTOR_COUNT)
  {
    clk_config.bytes = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  }
  return clk_config.clk_config;
}

double TMC429::getProportionalityFactor(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0.0;
  }
  PropFactor prop_factor;
  prop_factor.bytes = readRegister(motor, ADDRESS_PROP_FACTOR);
  int pm = prop_factor.pmul;
  int pd = prop_factor.pdiv;
  return ((double)(pm)) / ((double)(1 << (pd + 3)));
}

double TMC429::getStepTimeInMicroS()
{
  uint8_t step_div = getStepDiv();
  return stepDivToStepTime(step_div);
}

uint16_t TMC429::getVelocityMin(size_t motor)
{
  return readRegister(motor, ADDRESS_V_MIN);
}

void TMC429::setVelocityMinInHz(size_t motor,
  uint32_t velocity_min_hz)
{
  uint32_t velocity_min = convertVelocityFromHz(pulse_div_[motor], velocity_min_hz);
  if (velocity_min < VELOCITY_MIN_MIN)
  {
    velocity_min = VELOCITY_MIN_MIN;
  }
  writeRegister(motor, ADDRESS_V_MIN, velocity_min);
}

uint16_t TMC429::getVelocityMax(size_t motor)
{
  return readRegister(motor, ADDRESS_V_MAX);
}

void TMC429::setVelocityMaxInHz(size_t motor,
  uint32_t velocity_max_hz)
{
  uint32_t velocity_max = convertVelocityFromHz(pulse_div_[motor], velocity_max_hz);
  uint32_t velocity_max_upper_limit = getVelocityMaxUpperLimitInHz();
  if (velocity_max > velocity_max_upper_limit)
  {
    velocity_max = velocity_max_upper_limit;
  }
  writeRegister(motor, ADDRESS_V_MAX, velocity_max);
}

uint32_t TMC429::convertAccelerationToHzPerS(uint8_t pulse_div,
  uint8_t ramp_div,
  uint32_t acceleration)
{
  // (clock_frequency_*MHZ_PER_HZ*clock_frequency_*MHZ_PER_HZ*acceleration)/((1 << pulse_div)*(1 << ramp_div)*ACCELERATION_CONSTANT);
  double a = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)ACCELERATION_CONSTANT;
  double b = a*(double)clock_frequency_*(double)MHZ_PER_HZ;
  double c = b/((double)(1 << pulse_div));
  double d = c/((double)(1 << ramp_div));
  uint32_t e = round(d*(double)acceleration);
  return e;
}

uint32_t TMC429::convertAccelerationFromHzPerS(uint8_t pulse_div,
  uint8_t ramp_div,
  uint32_t acceleration)
{
  // (acceleration*(1 << pulse_div)*(1 << ramp_div)*ACCELERATION_CONSTANT)/(clock_frequency_*MHZ_PER_HZ*clock_frequency_*MHZ_PER_HZ);
  double a = ((double)acceleration*(double)(1 << pulse_div))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double b = a*(double)ACCELERATION_CONSTANT;
  double c = b/((double)clock_frequency_*(double)MHZ_PER_HZ);
  uint32_t d = round(c*(1 << ramp_div));
  return d;
}

uint8_t TMC429::findOptimalRampDivHz(uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div = RAMP_DIV_MAX;
  uint32_t acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);;
  uint32_t acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);

  while ((acceleration_max_upper_limit < acceleration_max_hz_per_s) &&
    (acceleration_max_lower_limit < acceleration_max_hz_per_s) &&
    (ramp_div >= 1) &&
    (ramp_div >= pulse_div))
  {
    --ramp_div;
    acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);
    acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);
  }
  return ramp_div;
}

void TMC429::setOptimalRampDivHz(size_t motor,
  uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  uint8_t ramp_div = findOptimalRampDivHz(velocity_max_hz, acceleration_max_hz_per_s);
  ClkConfig clk_config;
  clk_config.bytes = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  clk_config.clk_config.ramp_div = ramp_div;
  writeRegister(motor, ADDRESS_CLOCK_CONFIGURATION, clk_config.bytes);
  ramp_div_[motor] = ramp_div;
}

uint32_t TMC429::getVelocityMaxUpperLimitInHz(uint8_t pulse_div)
{
  return convertVelocityToHz(pulse_div, VELOCITY_REGISTER_MAX);
}

uint32_t TMC429::getAccelerationMaxUpperLimitInHzPerS(uint8_t pulse_div,
  uint8_t ramp_div)
{
  uint32_t a_max_upper_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div + 1) >= 0)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  else if (((int8_t)ramp_div - (int8_t)pulse_div + 12) < 1)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_upper_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div + 12)) - 1;
  }
  if (a_max_upper_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  if (a_max_upper_limit < ACCELERATION_REGISTER_MIN)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MIN;
  }
  return convertAccelerationToHzPerS(pulse_div, ramp_div, a_max_upper_limit);
}

uint32_t TMC429::getAccelerationMaxLowerLimitInHzPerS(uint8_t pulse_div,
  uint8_t ramp_div,
  uint32_t velocity_max)
{
  uint32_t a_max_lower_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div - 1) <= 0)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_lower_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div - 1));
    if (convertVelocityFromHz(pulse_div, velocity_max) <= (int16_t)VELOCITY_REGISTER_THRESHOLD)
    {
      a_max_lower_limit /= 2;
    }
  }
  if (a_max_lower_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MAX;
  }
  if (a_max_lower_limit < ACCELERATION_REGISTER_MIN)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MIN;
  }
  return convertAccelerationToHzPerS(pulse_div, ramp_div, a_max_lower_limit);
}

uint32_t TMC429::getAccelerationMax(size_t motor)
{
  return readRegister(motor, ADDRESS_A_MAX);
}

uint32_t TMC429::setAccelerationMaxInHzPerS(size_t motor,
  uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  uint32_t acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div_[motor], ramp_div_[motor]);
  uint32_t acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div_[motor], ramp_div_[motor], velocity_max_hz);
  if (acceleration_max_hz_per_s > acceleration_max_upper_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_upper_limit;
  }
  if (acceleration_max_hz_per_s < acceleration_max_lower_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_lower_limit;
  }
  uint32_t acceleration_max = convertAccelerationFromHzPerS(pulse_div_[motor], ramp_div_[motor], acceleration_max_hz_per_s);
  if (acceleration_max > ACCELERATION_REGISTER_MAX)
  {
    acceleration_max = ACCELERATION_REGISTER_MAX;
  }
  if (acceleration_max < ACCELERATION_REGISTER_MIN)
  {
    acceleration_max = ACCELERATION_REGISTER_MIN;
  }
  writeRegister(motor, ADDRESS_A_MAX, acceleration_max);
  return acceleration_max;
}

int16_t TMC429::getActualAcceleration(size_t motor)
{
  uint32_t acceleration_unsigned = readRegister(motor, ADDRESS_A_ACTUAL);
  return unsignedToSigned(acceleration_unsigned, A_BIT_COUNT);
}

void TMC429::setOptimalPropFactor(size_t motor,
  uint32_t acceleration_max)
{
  // int pdiv, pmul, pm, pd ;
  // double p_ideal, p_best, p, p_reduced;

  // pm=-1; pd=-1; // -1 indicates : no valid pair found

  // p_ideal = a_max / (pow(2, ramp_div-pulse_div)*128.0);
  // p = a_max / ( 128.0 * pow(2, ramp_div-pulse_div) );
  // p_reduced = p * ( 1.0 – p_reduction );
  // for (pdiv=0; pdiv<=13; pdiv++)
  // {
  //   pmul = (int)(p_reduced * 8.0 * pow(2, pdiv)) – 128;
  //   if ( (0 <= pmul) && (pmul <= 127) )
  //   {
  //     pm = pmul + 128;
  //     pd = pdiv;
  //   }
  //   *p_mul = pm;
  //   *p_div = pd;
  //   p_best = ((double)(pm)) / ((double)pow(2, pd+3));
  // }
  // *PIdeal = p_ideal;
  // *PBest = p_best;
  // *PRedu = p_reduced;

  int pdiv, pmul, pm, pd ;
  double p_ideal, p_reduced;

  pm=-1; pd=-1; // -1 indicates : no valid pair found
  p_ideal = acceleration_max/(128.0*(1 << (ramp_div_[motor] - pulse_div_[motor])));
  p_reduced = p_ideal*0.99;
  for (pdiv=0; pdiv<=13; ++pdiv)
  {
    pmul = (int)(p_reduced*8.0*(1 << pdiv)) - 128;
    if ((0 <= pmul) && (pmul <= 127))
    {
      pm = pmul + 128;
      pd = pdiv;
    }
  }
  if ((pm == -1) || (pd == -1))
  {
    return;
  }
  PropFactor prop_factor;
  prop_factor.bytes = readRegister(motor, ADDRESS_PROP_FACTOR);
  prop_factor.pmul = pm;
  prop_factor.pdiv = pd;
  writeRegister(motor, ADDRESS_PROP_FACTOR, prop_factor.bytes);
}

void TMC429::enableChipSelect()
{
  digitalWrite(chip_select_pin_, LOW);
}

void TMC429::disableChipSelect()
{
  digitalWrite(chip_select_pin_, HIGH);
}

void TMC429::beginTransaction()
{
  enableChipSelect();
  delayMicroseconds(1);
  spiBeginTransaction(SPISettings(SPI_CLOCK, SPI_BIT_ORDER, SPI_MODE));
}

void TMC429::endTransaction()
{
  spiEndTransaction();
  delayMicroseconds(1);
  disableChipSelect();
}

void TMC429::spiBegin()
{
  SPI.begin();
}

uint8_t TMC429::spiTransfer(uint8_t byte)
{
  return SPI.transfer(byte);
}

void TMC429::spiBeginTransaction(SPISettings settings)
{
  SPI.beginTransaction(settings);
}

void TMC429::spiEndTransaction()
{
  SPI.endTransaction();
}
