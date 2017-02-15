// ----------------------------------------------------------------------------
// TMC429.cpp
//
// Authors:
// Peter Polidoro polidorop@janelia.hhmi.org
// ----------------------------------------------------------------------------

#include "TMC429.h"


// TMC429::TMC429()
// {
//   // initialized_ = false;
// }

TMC429::TMC429(const size_t cs_pin) :
  cs_pin_(cs_pin)
{
  pinMode(cs_pin_,OUTPUT);
  digitalWrite(cs_pin_,HIGH);

  specifyClockFrequencyInMHz(CLOCK_FREQUENCY_MAX);
  pulse_div_ = 0;
}

void TMC429::setup()
{
  SPI.begin();
}

uint32_t TMC429::getVersion()
{
  return readRegister(SMDA_COMMON,ADDRESS_TYPE_VERSION_429);
}

bool TMC429::checkVersion()
{
  return (getVersion() == VERSION);
}

void TMC429::specifyClockFrequencyInMHz(uint8_t clock_frequency)
{
  if (clock_frequency <= CLOCK_FREQUENCY_MAX)
  {
    clock_frequency_ = clock_frequency;
  }
}

double TMC429::getStepTimeInMicroSeconds()
{
  GlobalParameters global_parameters;
  global_parameters.uint32 = readRegister(SMDA_COMMON,ADDRESS_GLOBAL_PARAMETERS);
  uint8_t stpdiv_429 = global_parameters.fields.clk2_div;
  Serial << "stpdiv_429: " << stpdiv_429 << "\n";
  double step_time = 16*(1 + stpdiv_429)/clock_frequency_;
  return step_time;
}

uint32_t TMC429::getVelocityMaxMaxInHz()
{
  // (clock_frequency_*MHZ_PER_HZ*VELOCITY_REGISTER_MAX)/(VELOCITY_CONSTANT);
  double x = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)VELOCITY_CONSTANT;
  double y = x*(double)VELOCITY_REGISTER_MAX;
  return y;
}

void TMC429::setVelocityScaleUsingTotalMaxInHz(const uint32_t velocity)
{
  setOptimalPulseDiv(velocity);
}

uint32_t TMC429::getPositionTarget(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_X_TARGET);
}

void TMC429::setPositionTarget(const size_t motor, uint32_t position)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor,ADDRESS_X_TARGET,position);
}

uint32_t TMC429::getPositionActual(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_X_ACTUAL);
}

void TMC429::setPositionActual(const size_t motor, uint32_t position)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor,ADDRESS_X_ACTUAL,position);
}

uint32_t TMC429::getVelocityMinInHz(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(getVelocityMin(motor));
}

void TMC429::setVelocityMinInHz(const size_t motor, const uint32_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setVelocityMin(motor,convertVelocityFromHz(velocity));
}

uint32_t TMC429::getVelocityMaxInHz(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(getVelocityMax(motor));
}

void TMC429::setVelocityMaxInHz(const size_t motor, const uint32_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setVelocityMax(motor,convertVelocityFromHz(velocity));
}

int32_t TMC429::getVelocityTargetInHz(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(getVelocityTarget(motor));
}

void TMC429::setVelocityTargetInHz(const size_t motor, const int32_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setVelocityTarget(motor,convertVelocityFromHz(velocity));
}

int32_t TMC429::getVelocityActualInHz(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(getVelocityActual(motor));
}

TMC429::Status TMC429::getStatus()
{
  return status_;
}

TMC429::Mode TMC429::getMode(const size_t motor)
{
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

void TMC429::setMode(const size_t motor, const Mode mode)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.mode = (uint8_t)mode;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

TMC429::ReferenceConfiguration TMC429::getReferenceConfiguration(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  return ref_conf_mode.fields.ref_conf;
}

void TMC429::enableLeftSwitchStop(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.disable_stop_l = 0;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::disableLeftSwitchStop(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.disable_stop_l = 1;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::enableRightSwitchStop(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.disable_stop_r = 0;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::disableRightSwitchStop(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.disable_stop_r = 1;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::enableSoftStop(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.soft_stop = 1;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::disableSoftStop(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.soft_stop = 0;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::setReferenceSwitchToLeft(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.ref_rnl = 0;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

void TMC429::setReferenceSwitchToRight(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.ref_conf.ref_rnl = 1;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

bool TMC429::positionLatched(const size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
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

void TMC429::setStepDirOutput()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.en_sd = 1;
  writeRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429,if_conf.uint32);
}

void TMC429::setSpiOutput()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  if_conf.fields.if_conf.en_sd = 0;
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

TMC429::ClockConfiguration TMC429::getClockConfiguration()
{
  ClkConfig clk_config;
  clk_config.uint32 = readRegister(SMDA_COMMON,ADDRESS_CLOCK_CONFIGURATION);
  return clk_config.fields.clk_config;
}

// private
uint32_t TMC429::readRegister(const uint8_t smda, const uint8_t address)
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

void TMC429::writeRegister(const uint8_t smda, const uint8_t address, const uint32_t data)
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

int32_t TMC429::convertVelocityToHz(const int16_t velocity)
{
  // (clock_frequency_*MHZ_PER_HZ*velocity)/((1 << pulse_div_)*VELOCITY_CONSTANT);
  double x = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)VELOCITY_CONSTANT;
  double y = (x*(double)velocity)/((double)(1 << pulse_div_));
  return y;
}

int16_t TMC429::convertVelocityFromHz(const int32_t velocity)
{
  // (velocity*(1 << pulse_div_)*VELOCITY_CONSTANT)/(clock_frequency_*MHZ_PER_HZ);
  double x = ((double)velocity*(double)(1 << pulse_div_))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double y = x*(double)VELOCITY_CONSTANT;
  return y;
}

void TMC429::setOptimalPulseDiv(const uint32_t velocity_max)
{
  int8_t pulse_div = PULSE_DIV_MAX + 1;
  uint32_t v_max = 0;
  while ((v_max < velocity_max) && (pulse_div >= 1))
  {
    --pulse_div;
    v_max = (VELOCITY_REGISTER_MAX*MHZ_PER_HZ)/((1 << pulse_div)*(VELOCITY_REGISTER_MAX + 1));
  }
  pulse_div_ = pulse_div;
  // writeRegister();
}

uint16_t TMC429::getVelocityMin(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_V_MIN);
}

void TMC429::setVelocityMin(const size_t motor, const uint16_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor,ADDRESS_V_MIN,velocity);
}

uint16_t TMC429::getVelocityMax(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_V_MAX);
}

void TMC429::setVelocityMax(const size_t motor, const uint16_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor,ADDRESS_V_MAX,velocity);
}

int16_t TMC429::getVelocityTarget(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  int16_t velocity = readRegister(motor,ADDRESS_V_TARGET);
  if (velocity > (int16_t)(V_MASK >> 1))
  {
    velocity -= V_MASK;
  }
  return velocity;
}

void TMC429::setVelocityTarget(const size_t motor, const int16_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  int16_t _velocity = velocity;
  if (_velocity < 0)
  {
    _velocity += V_MASK;
  }
  writeRegister(motor,ADDRESS_V_TARGET,_velocity);
}

int16_t TMC429::getVelocityActual(const size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  int16_t velocity = readRegister(motor,ADDRESS_V_ACTUAL);
  if (velocity > (int16_t)(V_MASK >> 1))
  {
    velocity -= V_MASK;
  }
  return velocity;
}

