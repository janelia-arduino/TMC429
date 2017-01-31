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

void TMC429::test()
{
  uint32_t type_version = getTypeVersion();
  Serial << "type_version: " << _HEX(type_version) << "\n";

  uint32_t position_target = getPositionTarget(0);
  Serial << "position_target: " << position_target << "\n";

  uint32_t position_actual = getPositionActual(0);
  Serial << "position_actual: " << position_actual << "\n";

  Status status = getStatus();
  Serial << "status.at_target_position_0 = " << status.at_target_position_0 << "\n";
  Serial << "status.switch_left_0 = " << status.switch_left_0 << "\n";
  Serial << "status.at_target_position_1 = " << status.at_target_position_1 << "\n";
  Serial << "status.switch_left_1 = " << status.switch_left_1 << "\n";
  Serial << "status.at_target_position_2 = " << status.at_target_position_2 << "\n";
  Serial << "status.switch_left_2 = " << status.switch_left_2 << "\n\n";

  // MosiDatagram datagram_write;
  // datagram_write.fields.rrs = RRS_REGISTER;
  // datagram_write.fields.address = ADDRESS_TYPE_VERSION_429;
  // datagram_write.fields.smda = SMDA_COMMON;
  // datagram_write.fields.rw = RW_READ;
  // datagram_write.fields.data = 0;
  // Serial << "sizeof(datagram_write): " << sizeof(datagram_write) << "\n";
  // Serial << "datagram_write: " << _HEX(datagram_write.uint32) << "\n";

  // // uint32_t datagram_write = 0x7f000000;
  // // uint32_t datagram_write = 0x73000000;
  // // uint32_t datagram_write = 0x1a000123;
  // // uint32_t datagram_write = 0x1b000000;
  // // Serial << "datagram_write = " << _HEX(datagram_write) << "\n";
  // // uint32_t datagram_read = writeRead(datagram_write);
  // MisoDatagram datagram_read = writeRead(datagram_write);
  // Serial << "datagram_read = " << _HEX(datagram_read.uint32) << "\n";
  // Serial << "datagram_read.fields.data = " << _HEX(datagram_read.fields.data) << "\n";
  // Serial << "datagram_read.fields.status.at_target_position_0 = " << !bool(datagram_read.fields.status.at_target_position_0) << "\n";
  // Serial << "datagram_read.fields.status.switch_left_0 = " << !bool(datagram_read.fields.status.switch_left_0) << "\n";
  // Serial << "datagram_read.fields.status.at_target_position_1 = " << !bool(datagram_read.fields.status.at_target_position_1) << "\n";
  // Serial << "datagram_read.fields.status.switch_left_1 = " << !bool(datagram_read.fields.status.switch_left_1) << "\n";
  // Serial << "datagram_read.fields.status.at_target_position_2 = " << !bool(datagram_read.fields.status.at_target_position_2) << "\n";
  // Serial << "datagram_read.fields.status.switch_left_2 = " << !bool(datagram_read.fields.status.switch_left_2) << "\n\n";

  // datagram_write.uint32 = 0x1b000000;
  // Serial << "datagram_write = " << _HEX(datagram_write.uint32) << "\n";
  // datagram_read = writeRead(datagram_write);
  // Serial << "datagram_read = " << _HEX(datagram_read.uint32) << "\n\n";
  // Serial << "datagram_read.fields.data = " << _HEX(datagram_read.fields.data) << "\n\n";

  // datagram_write.uint32 = 0x0;
  // Serial << "datagram_write = " << _HEX(datagram_write.uint32) << "\n";
  // datagram_read = writeRead(datagram_write);
  // Serial << "datagram_read = " << _HEX(datagram_read.uint32) << "\n\n\n";
  // Serial << "datagram_read.fields.data = " << _HEX(datagram_read.fields.data) << "\n\n";
}

uint32_t TMC429::getPositionTarget(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_X_TARGET);
}

uint32_t TMC429::getPositionActual(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return readRegister(motor,ADDRESS_X_ACTUAL);
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

