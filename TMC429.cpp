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
}

void TMC429::setup()
{
  SPI.begin();
}

uint32_t TMC429::getPositionTarget(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_X_TARGET);
}

void TMC429::setPositionTarget(size_t motor, uint32_t position)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor,ADDRESS_X_TARGET,position);
}

uint32_t TMC429::getPositionActual(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_X_ACTUAL);
}

uint16_t TMC429::getVelocityMin(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_V_MIN);
}

void TMC429::setVelocityMin(size_t motor, uint16_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor,ADDRESS_V_MIN,velocity);
}

uint16_t TMC429::getVelocityMax(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_V_MAX);
}

void TMC429::setVelocityMax(size_t motor, uint16_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor,ADDRESS_V_MAX,velocity);
}

TMC429::Status TMC429::getStatus()
{
  return status_;
}

uint32_t TMC429::getTypeVersion()
{
  return readRegister(SMDA_COMMON,ADDRESS_TYPE_VERSION_429);
}

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

