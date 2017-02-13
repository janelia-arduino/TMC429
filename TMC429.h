// ----------------------------------------------------------------------------
// TMC429.h
//
// Authors:
// Peter Polidoro polidorop@janelia.hhmi.org
// ----------------------------------------------------------------------------

#ifndef TMC429_H
#define TMC429_H
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "SPI.h"

#include "Streaming.h"


class TMC429
{
public:
  TMC429(const size_t cs_pin);

  void setup();

  uint32_t getPositionTarget(size_t motor);
  void setPositionTarget(size_t motor, uint32_t position);
  uint32_t getPositionActual(size_t motor);

  uint16_t getVelocityMin(size_t motor);
  void setVelocityMin(size_t motor, uint16_t velocity);

  uint16_t getVelocityMax(size_t motor);
  void setVelocityMax(size_t motor, uint16_t velocity);

  uint32_t getTypeVersion();

  struct Status
  {
    uint8_t at_target_position_0 : 1;
    uint8_t switch_left_0 : 1;
    uint8_t at_target_position_1 : 1;
    uint8_t switch_left_1 : 1;
    uint8_t at_target_position_2 : 1;
    uint8_t switch_left_2 : 1;
    uint8_t cdgw : 1;
    uint8_t interrupt : 1;
  };

  Status getStatus();

private:
  // SPISettings
  const static uint32_t SPI_CLOCK = 1000000;
  const static uint8_t SPI_BIT_ORDER = MSBFIRST;
  const static uint8_t SPI_MODE = SPI_MODE3;

  const static uint8_t MOTOR_COUNT = 3;

  Status status_;

  // MOSI Datagrams
  union MosiDatagram
  {
    struct Fields
    {
      uint32_t data : 24;
      uint8_t rw : 1;
      uint8_t address : 4;
      uint8_t smda : 2;
      uint8_t rrs : 1;
    } fields;
    uint32_t uint32;
  };
  const static uint8_t RW_READ = 1;
  const static uint8_t RW_WRITE = 0;

  // IDX
  const static uint8_t ADDRESS_X_TARGET = 0b0000;
  const static uint8_t ADDRESS_X_ACTUAL = 0b0001;
  const static uint8_t ADDRESS_V_MIN = 0b0010;
  const static uint8_t ADDRESS_V_MAX = 0b0011;
  const static uint8_t ADDRESS_V_TARGET = 0b0100;
  const static uint8_t ADDRESS_V_ACTUAL = 0b0101;
  const static uint8_t ADDRESS_A_MAX = 0b0110;
  const static uint8_t ADDRESS_A_ACTUAL = 0b0111;
  const static uint8_t ADDRESS_A_THRESHOLD = 0b1000;

  // JDX
  const static uint8_t ADDRESS_POWER_DOWN = 0b1000;
  const static uint8_t ADDRESS_TYPE_VERSION_429 = 0b1001;

  const static uint8_t SMDA_COMMON = 0b11;

  const static uint8_t RRS_REGISTER = 0;
  const static uint8_t RRS_RAM = 1;

  // MISO Datagrams
  union MisoDatagram
  {
    struct Fields
    {
      uint32_t data : 24;
      Status status;
      // uint32_t at_target_position_0 : 1;
      // uint32_t switch_left_0 : 1;
      // uint32_t at_target_position_1 : 1;
      // uint32_t switch_left_1 : 1;
      // uint32_t at_target_position_2 : 1;
      // uint32_t switch_left_2 : 1;
      // uint32_t cdgw : 6;
      // uint32_t interrupt : 1;
    } fields;
    uint32_t uint32;
  };

  size_t cs_pin_;

  uint32_t readRegister(const uint8_t smda, const uint8_t address);
  void writeRegister(const uint8_t smda, const uint8_t address, const uint32_t data);
  MisoDatagram writeRead(const MosiDatagram datagram_write);
};

#endif
