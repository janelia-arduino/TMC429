// ----------------------------------------------------------------------------
// TMC429.cpp
//
// Authors:
// Peter Polidoro polidorop@janelia.hhmi.org
// ----------------------------------------------------------------------------

#include "TMC429.h"


void TMC429::setup(const size_t cs_pin,
                   const uint8_t clock_frequency_mhz)
{
  cs_pin_ = cs_pin;

  pinMode(cs_pin_,OUTPUT);
  digitalWrite(cs_pin_,HIGH);

  specifyClockFrequencyInMHz(clock_frequency_mhz);

  for (uint8_t motor=0; motor<MOTOR_COUNT; ++motor)
  {
    pulse_div_[motor] = 0;
    ramp_div_[motor] = 0;
  }

  SPI.begin();

  setStepDiv(STEP_DIV_MAX);
}

uint32_t TMC429::getVersion()
{
  return readRegister(SMDA_COMMON,ADDRESS_TYPE_VERSION_429);
}

bool TMC429::checkVersion()
{
  return (getVersion() == VERSION);
}

void TMC429::setStepDirOutput()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.en_sd = 1;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

// void TMC429::setSpiOutput()
// {
//   IfConf if_conf;
//   if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
//   if_conf.fields.if_conf.en_sd = 0;
//   writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
// }

TMC429::Mode TMC429::getMode(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return RAMP_MODE;
  }

  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  switch (ref_conf_mode.fields.mode)
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

void TMC429::setMode(const size_t motor,
                     const Mode mode)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }

  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.mode = (uint8_t)mode;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

uint32_t TMC429::getVelocityMaxUpperLimitInHz()
{
  // (clock_frequency_*MHZ_PER_HZ*VELOCITY_REGISTER_MAX)/(VELOCITY_CONSTANT);
  double x = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)VELOCITY_CONSTANT;
  double y = x*(double)VELOCITY_REGISTER_MAX;
  return y;
}

void TMC429::setLimitsInHz(const size_t motor,
                           const uint32_t velocity_min,
                           const uint32_t velocity_max,
                           const uint32_t acceleration_max)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }

  setOptimalStepDiv(velocity_max);

  setOptimalPulseDiv(motor,velocity_max);

  setVelocityMin(motor,convertVelocityFromHz(motor,velocity_min));

  setVelocityMax(motor,convertVelocityFromHz(motor,velocity_max));

  setOptimalRampDiv(motor,acceleration_max);

  uint16_t a_max = setAccelerationMax(motor,convertAccelerationFromHzPerS(motor,acceleration_max));

  setOptimalPropFactor(motor,a_max);
}

uint32_t TMC429::getAccelerationMaxInHzPerS(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertAccelerationToHzPerS(motor,getAccelerationMax(motor));
}

uint32_t TMC429::getAccelerationMaxUpperLimitInHzPerS(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint8_t ramp_div_min = pulse_div_[motor] - 1;
  uint32_t a_max_upper_limit = (1 << (ramp_div_min - pulse_div_[motor] + 12));
  if (a_max_upper_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  double a = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)ACCELERATION_CONSTANT;
  double b = a*(double)clock_frequency_*(double)MHZ_PER_HZ;
  double c = b/((double)(1 << pulse_div_[motor]));
  double d = c/((double)(1 << ramp_div_min));
  double e = d*(double)a_max_upper_limit;
  return e;
}

uint32_t TMC429::getAccelerationActualInHzPerS(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertAccelerationToHzPerS(motor,getAccelerationActual(motor));
}

uint32_t TMC429::getVelocityMinInHz(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(motor,getVelocityMin(motor));
}

uint32_t TMC429::getVelocityMaxInHz(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(motor,getVelocityMax(motor));
}

int32_t TMC429::getVelocityTargetInHz(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(motor,getVelocityTarget(motor));
}

void TMC429::setVelocityTargetInHz(const size_t motor,
                                   const int32_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setVelocityTarget(motor,convertVelocityFromHz(motor,velocity));
}

int32_t TMC429::getVelocityActualInHz(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(motor,getVelocityActual(motor));
}

int32_t TMC429::getPositionTarget(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = readRegister(motor,ADDRESS_X_TARGET);
  return unsignedToSigned(position_unsigned,X_BIT_COUNT);
}

void TMC429::setPositionTarget(const size_t motor,
                               int32_t position)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor,ADDRESS_X_TARGET,position);
}

int32_t TMC429::getPositionActual(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = readRegister(motor,ADDRESS_X_ACTUAL);
  return unsignedToSigned(position_unsigned,X_BIT_COUNT);
}

void TMC429::setPositionActual(const size_t motor,
                               int32_t position)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor,ADDRESS_X_ACTUAL,position);
}

TMC429::Status TMC429::getStatus()
{
  getVersion();
  return status_;
}

TMC429::ReferenceConfiguration TMC429::getReferenceConfiguration(const size_t motor)
{
  RefConfMode ref_conf_mode;
  if (motor < MOTOR_COUNT)
  {
    ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.fields.ref_conf;
}

void TMC429::enableLeftSwitchStop(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.disable_stop_l = 0;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::disableLeftSwitchStop(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.disable_stop_l = 1;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::enableRightSwitchStop(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.disable_stop_r = 0;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::disableRightSwitchStop(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.disable_stop_r = 1;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::enableSoftStop(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.soft_stop = 1;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::disableSoftStop(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.soft_stop = 0;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::setReferenceSwitchToLeft(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.ref_rnl = 0;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::setReferenceSwitchToRight(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.ref_rnl = 1;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

bool TMC429::positionLatched(const size_t motor)
{
  RefConfMode ref_conf_mode;
  if (motor < MOTOR_COUNT)
  {
    ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.fields.lp;
}

TMC429::InterfaceConfiguration TMC429::getInterfaceConfiguration()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  return if_conf.fields.if_conf;
}

void TMC429::setReferenceActiveLow()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.inv_ref = 1;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

void TMC429::setReferenceActiveHigh()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.inv_ref = 0;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

void TMC429::enableInverseStepPolarity()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.inv_stp = 1;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

void TMC429::disableInverseStepPolarity()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.inv_stp = 0;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

void TMC429::enableInverseDirPolarity()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.inv_dir = 1;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

void TMC429::disableInverseDirPolarity()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.inv_dir = 0;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

void TMC429::setPositionCompareMotor(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.pos_comp_sel = motor;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

void TMC429::enableRightReferences()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.en_refr = 1;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

void TMC429::disableRightReferences()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.en_refr = 0;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

TMC429::SwitchState TMC429::getSwitchState()
{
  SwState switch_state;
  switch_state.uint32 = readRegister(SMDA_COMMON,ADDRESS_SWITCHES);
  return switch_state.fields.switch_state;
}

TMC429::ClockConfiguration TMC429::getClockConfiguration(const size_t motor)
{
  ClkConfig clk_config;
  if (motor < MOTOR_COUNT)
  {
    clk_config.uint32 = readRegister(motor,ADDRESS_CLOCK_CONFIGURATION);
  }
  return clk_config.fields.clk_config;
}

double TMC429::getProportionalityFactor(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0.0;
  }
  PropFactor prop_factor;
  prop_factor.uint32 = readRegister(motor,ADDRESS_PROP_FACTOR);
  int pm = prop_factor.fields.pmul;
  int pd = prop_factor.fields.pdiv;
  return ((double)(pm)) / ((double)(1 << (pd + 3)));
}

double TMC429::getStepTimeInMicroS()
{
  uint8_t step_div = getStepDiv();
  return stepDivToStepTime(step_div);
}

// private
uint32_t TMC429::readRegister(const uint8_t smda,
                              const uint8_t address)
{
  MosiDatagram datagram_write;
  datagram_write.fields.rrs = RRS_REGISTER;
  datagram_write.fields.address = address;
  datagram_write.fields.smda = smda;
  datagram_write.fields.rw = RW_READ;
  datagram_write.fields.data = 0;
  MisoDatagram datagram_read = writeRead(datagram_write);
  return datagram_read.fields.data;
}

void TMC429::writeRegister(const uint8_t smda,
                           const uint8_t address,
                           const uint32_t data)
{
  MosiDatagram datagram_write;
  datagram_write.fields.rrs = RRS_REGISTER;
  datagram_write.fields.address = address;
  datagram_write.fields.smda = smda;
  datagram_write.fields.rw = RW_WRITE;
  datagram_write.fields.data = data;
  writeRead(datagram_write);
}

TMC429::MisoDatagram TMC429::writeRead(const MosiDatagram datagram_write)
{
  MisoDatagram datagram_read;
  datagram_read.uint32 = 0x0;
  SPI.beginTransaction(SPISettings(SPI_CLOCK,SPI_BIT_ORDER,SPI_MODE));
  digitalWrite(cs_pin_,LOW);
  for (int i=(sizeof(datagram_write) - 1); i>=0; --i)
  {
    uint8_t byte_write = (datagram_write.uint32 >> (8*i)) & 0xff;
    uint8_t byte_read = SPI.transfer(byte_write);
    datagram_read.uint32 |= byte_read << (8*i);
  }
  digitalWrite(cs_pin_,HIGH);
  SPI.endTransaction();
  noInterrupts();
  status_ = datagram_read.fields.status;
  interrupts();
  return datagram_read;
}

int32_t TMC429::unsignedToSigned(uint32_t input_value, uint8_t num_bits)
{
  uint32_t mask = 1 << (num_bits - 1);
  return -(input_value & mask) + (input_value & ~mask);
}

void TMC429::specifyClockFrequencyInMHz(const uint8_t clock_frequency)
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

void TMC429::setOptimalStepDiv(const uint32_t velocity_max_hz)
{
  int step_div = getStepDiv();

  double step_time = stepDivToStepTime(step_div);

  uint32_t v_max = (double)MHZ_PER_HZ/(step_time*2);

  while ((v_max < velocity_max_hz) && (step_div >= 1))
  {
    --step_div;
    step_time = stepDivToStepTime(step_div);
    v_max = (double)MHZ_PER_HZ/(step_time*2);
  }

  setStepDiv(step_div);
}

uint8_t TMC429::getStepDiv()
{
  GlobalParameters global_parameters;
  global_parameters.uint32 = readRegister(SMDA_COMMON,ADDRESS_GLOBAL_PARAMETERS);
  return global_parameters.fields.clk2_div & STEP_DIV_MASK;
}

void TMC429::setStepDiv(const uint8_t step_div)
{
  GlobalParameters global_parameters;
  global_parameters.uint32 = readRegister(SMDA_COMMON,ADDRESS_GLOBAL_PARAMETERS);
  global_parameters.fields.clk2_div = step_div & STEP_DIV_MASK;
  writeRegister(SMDA_COMMON,ADDRESS_GLOBAL_PARAMETERS,global_parameters.uint32);
}

double TMC429::stepDivToStepTime(const uint8_t step_div)
{
  double step_time = (double)(16*(1 + step_div))/(double)clock_frequency_;
  return step_time;
}

int32_t TMC429::convertVelocityToHz(const size_t motor,
                                    const int16_t velocity)
{
  // (clock_frequency_*MHZ_PER_HZ*velocity)/((1 << pulse_div_[motor])*VELOCITY_CONSTANT);
  double x = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)VELOCITY_CONSTANT;
  double y = (x*(double)velocity)/((double)(1 << pulse_div_[motor]));
  return y;
}

int16_t TMC429::convertVelocityFromHz(const size_t motor,
                                      const int32_t velocity)
{
  // (velocity*(1 << pulse_div_[motor])*VELOCITY_CONSTANT)/(clock_frequency_*MHZ_PER_HZ);
  double x = ((double)velocity*(double)(1 << pulse_div_[motor]))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double y = x*(double)VELOCITY_CONSTANT;
  return y;
}

void TMC429::setOptimalPulseDiv(const size_t motor,
                                const uint32_t velocity_max_hz)
{
  int8_t pulse_div = PULSE_DIV_MAX + 1;
  uint32_t v_max = 0;

  while ((v_max < velocity_max_hz) && (pulse_div >= 1))
  {
    --pulse_div;
    pulse_div_[motor] = pulse_div;
    v_max = convertVelocityToHz(motor,VELOCITY_REGISTER_MAX);
  }
  ClkConfig clk_config;
  clk_config.uint32 = readRegister(motor,ADDRESS_CLOCK_CONFIGURATION);
  clk_config.fields.clk_config.pulse_div = pulse_div;
  writeRegister(motor,ADDRESS_CLOCK_CONFIGURATION,clk_config.uint32);
}

uint16_t TMC429::getVelocityMin(const size_t motor)
{
  return readRegister(motor,ADDRESS_V_MIN);
}

void TMC429::setVelocityMin(const size_t motor,
                            const uint16_t velocity)
{
  writeRegister(motor,ADDRESS_V_MIN,velocity);
}

uint16_t TMC429::getVelocityMax(const size_t motor)
{
  return readRegister(motor,ADDRESS_V_MAX);
}

void TMC429::setVelocityMax(const size_t motor,
                            const uint16_t velocity)
{
  writeRegister(motor,ADDRESS_V_MAX,velocity);
}

int16_t TMC429::getVelocityTarget(const size_t motor)
{
  uint32_t velocity_unsigned = readRegister(motor,ADDRESS_V_TARGET);
  return unsignedToSigned(velocity_unsigned,V_BIT_COUNT);
}

void TMC429::setVelocityTarget(const size_t motor,
                               const int16_t velocity)
{
  writeRegister(motor,ADDRESS_V_TARGET,velocity);
}

int16_t TMC429::getVelocityActual(const size_t motor)
{
  uint32_t velocity_unsigned = readRegister(motor,ADDRESS_V_ACTUAL);
  return unsignedToSigned(velocity_unsigned,V_BIT_COUNT);
}

int32_t TMC429::convertAccelerationToHzPerS(const size_t motor,
                                            const int16_t acceleration)
{
  // (clock_frequency_*MHZ_PER_HZ*clock_frequency_*MHZ_PER_HZ*acceleration)/((1 << pulse_div_[motor])*(1 << ramp_div_[motor])*ACCELERATION_CONSTANT);
  double a = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)ACCELERATION_CONSTANT;
  double b = a*(double)clock_frequency_*(double)MHZ_PER_HZ;
  double c = b/((double)(1 << pulse_div_[motor]));
  double d = c/((double)(1 << ramp_div_[motor]));
  double e = d*(double)acceleration;
  return e;
}

int16_t TMC429::convertAccelerationFromHzPerS(const size_t motor,
                                              const int32_t acceleration)
{
  // (acceleration*(1 << pulse_div_[motor])*(1 << ramp_div_[motor])*ACCELERATION_CONSTANT)/(clock_frequency_*MHZ_PER_HZ*clock_frequency_*MHZ_PER_HZ);
  double a = ((double)acceleration*(double)(1 << pulse_div_[motor]))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double b = a*(double)ACCELERATION_CONSTANT;
  double c = b/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double d = c*(1 << ramp_div_[motor]);
  return d;
}

void TMC429::setOptimalRampDiv(const size_t motor,
                               const uint32_t acceleration_max_hz_per_s)
{
  int8_t ramp_div = RAMP_DIV_MAX + 1;
  uint32_t a_max = 0;

  while ((a_max < acceleration_max_hz_per_s) &&
         (ramp_div >= 1) &&
         (ramp_div >= pulse_div_[motor]))
  {
    --ramp_div;
    ramp_div_[motor] = ramp_div;
    a_max = convertAccelerationToHzPerS(motor,ACCELERATION_REGISTER_MAX);
  }
  ClkConfig clk_config;
  clk_config.uint32 = readRegister(motor,ADDRESS_CLOCK_CONFIGURATION);
  clk_config.fields.clk_config.ramp_div = ramp_div;
  writeRegister(motor,ADDRESS_CLOCK_CONFIGURATION,clk_config.uint32);
}

uint16_t TMC429::getAccelerationMaxUpperLimit(const size_t motor)
{
  uint32_t a_max_upper_limit = (1 << (ramp_div_[motor] - pulse_div_[motor] + 12));
  if (a_max_upper_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  return a_max_upper_limit;
}

uint16_t TMC429::getAccelerationMaxLowerLimit(const size_t motor)
{
  uint32_t a_max_lower_limit = (1 << (ramp_div_[motor] - pulse_div_[motor] - 1));
  return a_max_lower_limit;
}

uint16_t TMC429::getAccelerationMax(const size_t motor)
{
  return readRegister(motor,ADDRESS_A_MAX);
}

uint16_t TMC429::setAccelerationMax(const size_t motor,
                                    const uint16_t acceleration)
{
  uint32_t a_max = acceleration;
  uint32_t a_max_upper_limit = getAccelerationMaxUpperLimit(motor);
  uint32_t a_max_lower_limit = getAccelerationMaxLowerLimit(motor);
  if (a_max > a_max_upper_limit)
  {
    a_max = a_max_upper_limit;
  }
  else if (a_max < a_max_lower_limit)
  {
    a_max = a_max_lower_limit;
  }
  writeRegister(motor,ADDRESS_A_MAX,a_max);
  return a_max;
}

int16_t TMC429::getAccelerationActual(const size_t motor)
{
  uint32_t acceleration_unsigned = readRegister(motor,ADDRESS_A_ACTUAL);
  return unsignedToSigned(acceleration_unsigned,A_BIT_COUNT);
}

void TMC429::setOptimalPropFactor(const size_t motor,
                                  const uint16_t acceleration_max)
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
  //   p_best = ((double)(pm)) / ((double)pow(2,pd+3));
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
  prop_factor.uint32 = readRegister(motor,ADDRESS_PROP_FACTOR);
  prop_factor.fields.pmul = pm;
  prop_factor.fields.pdiv = pd;
  writeRegister(motor,ADDRESS_PROP_FACTOR,prop_factor.uint32);
}

