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
  void setup(const size_t cs_pin,
             const uint8_t clock_frequency_mhz);

  uint32_t getVersion();
  bool checkVersion();

  void setStepDirOutput();
  // void setSpiOutput();

  enum Mode
    {
      RAMP_MODE=0b00,
      SOFT_MODE=0b01,
      VELOCITY_MODE=0b10,
      HOLD_MODE=0b11,
    };

  Mode getMode(const size_t motor);
  void setMode(const size_t motor,
               const Mode mode);

  uint32_t getVelocityMaxUpperLimitInHz();

  void setLimitsInHz(const size_t motor,
                     const uint32_t velocity_min,
                     const uint32_t velocity_max,
                     const uint32_t acceleration_max);

  uint32_t getAccelerationMaxInHzPerS(const size_t motor);
  uint32_t getAccelerationMaxUpperLimitInHzPerS(const size_t motor);

  uint32_t getActualAccelerationInHzPerS(const size_t motor);

  uint32_t getVelocityMinInHz(const size_t motor);
  uint32_t getVelocityMaxInHz(const size_t motor);

  int32_t getTargetVelocityInHz(const size_t motor);
  void setTargetVelocityInHz(const size_t motor,
                             const int32_t velocity);

  int32_t getActualVelocityInHz(const size_t motor);

  int32_t getTargetPosition(const size_t motor);
  void setTargetPosition(const size_t motor,
                         const int32_t position);

  int32_t getActualPosition(const size_t motor);
  void setActualPosition(const size_t motor,
                         const int32_t position);

  void stop(const size_t motor);
  void stopAll();

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

  struct ReferenceConfiguration
  {
    uint8_t disable_stop_l : 1;
    uint8_t disable_stop_r : 1;
    uint8_t soft_stop : 1;
    uint8_t ref_rnl : 1;
    uint8_t space : 4;
  };

  ReferenceConfiguration getReferenceConfiguration(const size_t motor);

  void enableLeftSwitchStop(const size_t motor);
  void disableLeftSwitchStop(const size_t motor);

  void enableRightSwitchStop(const size_t motor);
  void disableRightSwitchStop(const size_t motor);

  void enableSoftStop(const size_t motor);
  void disableSoftStop(const size_t motor);

  void setReferenceSwitchToLeft(const size_t motor);
  void setReferenceSwitchToRight(const size_t motor);

  bool positionLatched(const size_t motor);

  struct InterfaceConfiguration
  {
    uint16_t inv_ref : 1;
    uint16_t sdo_int : 1;
    uint16_t step_half : 1;
    uint16_t inv_stp : 1;
    uint16_t inv_dir : 1;
    uint16_t en_sd : 1;
    uint16_t pos_comp_sel : 2;
    uint16_t en_refr : 1;
    uint16_t space : 7;
  };

  InterfaceConfiguration getInterfaceConfiguration();

  void setReferenceActiveLow();
  void setReferenceActiveHigh();

  void enableInverseStepPolarity();
  void disableInverseStepPolarity();

  void enableInverseDirPolarity();
  void disableInverseDirPolarity();

  void setPositionCompareMotor(const size_t motor);

  void enableRightReferences();
  void disableRightReferences();

  struct SwitchState
  {
    uint8_t r0 : 1;
    uint8_t l0 : 1;
    uint8_t r1 : 1;
    uint8_t l1 : 1;
    uint8_t r2 : 1;
    uint8_t l2 : 1;
    uint8_t space : 2;
  };

  SwitchState getSwitchState();

  struct ClockConfiguration
  {
    uint16_t usrs : 3;
    uint16_t space0 : 5;
    uint16_t ramp_div : 4;
    uint16_t pulse_div : 4;
  };

  ClockConfiguration getClockConfiguration(const size_t motor);
  double getProportionalityFactor(const size_t motor);

  double getStepTimeInMicroS();

private:
  // SPISettings
  const static uint32_t SPI_CLOCK = 1000000;
  const static uint8_t SPI_BIT_ORDER = MSBFIRST;
  const static uint8_t SPI_MODE = SPI_MODE3;

  const static uint8_t MOTOR_COUNT = 3;

  const static uint32_t VERSION = 0x429101;

  const static uint8_t CLOCK_FREQUENCY_MAX = 32;
  const static uint8_t PULSE_DIV_MAX = 13;
  const static uint8_t STEP_DIV_MAX = 15;
  const static uint32_t MHZ_PER_HZ = 1000000;
  const static uint32_t VELOCITY_CONSTANT = 65536;
  const static uint32_t VELOCITY_REGISTER_MAX = 2047;
  const static uint8_t RAMP_DIV_MAX = 13;
  const static uint32_t ACCELERATION_REGISTER_MAX = 2047;
  const static uint32_t ACCELERATION_CONSTANT = 536870912; // (1 << 29)

  Status status_;
  uint8_t clock_frequency_;
  uint8_t pulse_div_[MOTOR_COUNT];
  uint8_t ramp_div_[MOTOR_COUNT];

  // MOSI Datagrams
  union MosiDatagram
  {
    struct Fields
    {
      uint32_t data : 24;
      uint32_t rw : 1;
      uint32_t address : 4;
      uint32_t smda : 2;
      uint32_t rrs : 1;
    } fields;
    uint32_t uint32;
  };
  const static uint8_t RW_READ = 1;
  const static uint8_t RW_WRITE = 0;

  // IDX Addresses
  const static uint8_t ADDRESS_X_TARGET = 0b0000;
  const static uint8_t ADDRESS_X_ACTUAL = 0b0001;
  const static uint8_t ADDRESS_V_MIN = 0b0010;
  const static uint8_t ADDRESS_V_MAX = 0b0011;
  const static uint8_t ADDRESS_V_TARGET = 0b0100;
  const static uint8_t ADDRESS_V_ACTUAL = 0b0101;
  const static uint8_t ADDRESS_A_MAX = 0b0110;
  const static uint8_t ADDRESS_A_ACTUAL = 0b0111;
  const static uint8_t ADDRESS_A_THRESHOLD = 0b1000;
  const static uint8_t ADDRESS_PROP_FACTOR = 0b1001;
  const static uint8_t ADDRESS_REF_CONF_MODE = 0b1010;
  const static uint8_t ADDRESS_INTERRUPT = 0b1011;
  const static uint8_t ADDRESS_CLOCK_CONFIGURATION = 0b1100;
  const static uint8_t ADDRESS_DX_REF_TOLERANCE = 0b1101;
  const static uint8_t ADDRESS_X_LATCHED = 0b1110;
  const static uint8_t ADDRESS_USTEP_COUNT_429 = 0b1111;

  // JDX Addresses
  const static uint8_t ADDRESS_DATAGRAM_LOW_WORD = 0b0000;
  const static uint8_t ADDRESS_DATAGRAM_HIGH_WORD = 0b0001;
  const static uint8_t ADDRESS_COVER_POS_LEN = 0b0010;
  const static uint8_t ADDRESS_COVER_DATAGRAM = 0b0011;
  const static uint8_t ADDRESS_IF_CONFIGURATION_429 = 0b0100;
  const static uint8_t ADDRESS_POS_COMP_429 = 0b0101;
  const static uint8_t ADDRESS_POS_COMP_INT = 0b0110;
  const static uint8_t ADDRESS_POWER_DOWN = 0b1000;
  const static uint8_t ADDRESS_TYPE_VERSION_429 = 0b1001;
  const static uint8_t ADDRESS_SWITCHES = 0b1110;
  const static uint8_t ADDRESS_GLOBAL_PARAMETERS = 0b1111;

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
    } fields;
    uint32_t uint32;
  };

  // Masks
  const static uint8_t STEP_DIV_MASK = 0xf;

  // Bit Count
  const static uint8_t X_BIT_COUNT = 24;
  const static uint8_t V_BIT_COUNT = 12;
  const static uint8_t A_BIT_COUNT = 12;

  // Union Structs
  union PropFactor
  {
    struct Fields
    {
      uint32_t pdiv : 4;
      uint32_t space0 : 4;
      uint32_t pmul : 8;
      uint32_t space1 : 8;
      uint32_t space2 : 8;
    } fields;
    uint32_t uint32;
  };
  union RefConfMode
  {
    struct Fields
    {
      uint32_t mode : 2;
      uint32_t space0 : 6;
      ReferenceConfiguration ref_conf;
      uint32_t lp : 1;
      uint32_t space1 : 7;
      uint32_t space2 : 8;
    } fields;
    uint32_t uint32;
  };
  union IfConf
  {
    struct Fields
    {
      InterfaceConfiguration if_conf;
      uint32_t space0 : 16;
    } fields;
    uint32_t uint32;
  };
  union SwState
  {
    struct Fields
    {
      SwitchState switch_state;
      uint32_t space0 : 16;
    } fields;
    uint32_t uint32;
  };
  union GlobalParameters
  {
    struct Fields
    {
      uint32_t lsmd : 2;
      uint32_t nscs_s : 1;
      uint32_t sck_s : 1;
      uint32_t ph_ab : 1;
      uint32_t fd_ab : 1;
      uint32_t dac_ab : 1;
      uint32_t cs_com_ind : 1;
      uint32_t clk2_div : 8;
      uint32_t cont_update : 1;
      uint32_t space0 : 3;
      uint32_t ref_mux : 1;
      uint32_t mot1r : 1;
      uint32_t space1 : 2;
    } fields;
    uint32_t uint32;
  };
  union ClkConfig
  {
    struct Fields
    {
      ClockConfiguration clk_config;
      uint32_t space0 : 16;
    } fields;
    uint32_t uint32;
  };

  size_t cs_pin_;

  uint32_t readRegister(const uint8_t smda,
                        const uint8_t address);
  void writeRegister(const uint8_t smda,
                     const uint8_t address,
                     const uint32_t data);
  MisoDatagram writeRead(const MosiDatagram datagram_write);

  int32_t unsignedToSigned(uint32_t input_value, uint8_t num_bits);

  void specifyClockFrequencyInMHz(const uint8_t clock_frequency);

  void setOptimalStepDiv(const uint32_t velocity_max_hz);
  uint8_t getStepDiv();
  void setStepDiv(const uint8_t step_div);
  double stepDivToStepTime(const uint8_t step_div);

  int32_t convertVelocityToHz(const size_t motor,
                              const int16_t velocity);
  int16_t convertVelocityFromHz(const size_t motor,
                                const int32_t velocity);

  void setOptimalPulseDiv(const size_t motor,
                          const uint32_t velocity_max_hz);

  uint16_t getVelocityMin(const size_t motor);
  void setVelocityMin(const size_t motor,
                      const uint16_t velocity);

  uint16_t getVelocityMax(const size_t motor);
  void setVelocityMax(const size_t motor,
                      const uint16_t velocity);

  int16_t getTargetVelocity(const size_t motor);
  void setTargetVelocity(const size_t motor,
                         const int16_t velocity);

  int16_t getActualVelocity(const size_t motor);

  int32_t convertAccelerationToHzPerS(const size_t motor,
                                      const int16_t acceleration);
  int16_t convertAccelerationFromHzPerS(const size_t motor,
                                        const int32_t acceleration);

  void setOptimalRampDiv(const size_t motor,
                         const uint32_t acceleration_max_hz_per_s);

  uint16_t getAccelerationMaxUpperLimit(const size_t motor);
  uint16_t getAccelerationMaxLowerLimit(const size_t motor);

  uint16_t getAccelerationMax(const size_t motor);
  uint16_t setAccelerationMax(const size_t motor,
                              const uint16_t acceleration);

  int16_t getActualAcceleration(const size_t motor);

  void setOptimalPropFactor(const size_t motor,
                            const uint16_t acceleration_max);

};

#endif
