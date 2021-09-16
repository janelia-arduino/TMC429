// ----------------------------------------------------------------------------
// TMC429.h
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#ifndef TMC429_H
#define TMC429_H
#include <Arduino.h>
#include <SPI.h>

class TMC429
{
public:
  void setup(size_t chip_select_pin,
    uint8_t clock_frequency_mhz);

  bool communicating();
  uint32_t getVersion();

  void initialize();

  void setRampMode(size_t motor);
  void setSoftMode(size_t motor);
  void setVelocityMode(size_t motor);

  void setLimitsInHz(size_t motor,
    uint32_t velocity_min_hz,
    uint32_t velocity_max_hz,
    uint32_t acceleration_max_hz_per_s);

  uint32_t getVelocityMaxUpperLimitInHz();
  uint32_t getVelocityMinInHz(size_t motor);
  uint32_t getVelocityMaxInHz(size_t motor);

  int32_t getTargetVelocityInHz(size_t motor);
  void setTargetVelocityInHz(size_t motor,
    int32_t velocity_hz);
  bool atTargetVelocity(size_t motor);

  int32_t getActualVelocityInHz(size_t motor);

  uint32_t getAccelerationMaxUpperLimitInHzPerS(uint32_t velocity_max_hz);
  uint32_t getAccelerationMaxLowerLimitInHzPerS(uint32_t velocity_max_hz);
  uint32_t getAccelerationMaxInHzPerS(size_t motor);
  uint32_t getActualAccelerationInHzPerS(size_t motor);

  int32_t getTargetPosition(size_t motor);
  void setTargetPosition(size_t motor,
    int32_t position);
  bool atTargetPosition(size_t motor);

  int32_t getActualPosition(size_t motor);
  void setActualPosition(size_t motor,
    int32_t position);

  void stop(size_t motor);
  void stopAll();

  void enableInverseStepPolarity();
  void disableInverseStepPolarity();

  void enableInverseDirPolarity();
  void disableInverseDirPolarity();

  void setSwitchesActiveLow();
  void setSwitchesActiveHigh();

  void enableLeftSwitchStop(size_t motor);
  void disableLeftSwitchStop(size_t motor);
  bool leftSwitchStopEnabled(size_t motor);
  bool leftSwitchActive(size_t motor);

  void enableRightSwitches();
  void disableRightSwitches();
  bool rightSwitchesEnabled();

  void enableRightSwitchStop(size_t motor);
  void disableRightSwitchStop(size_t motor);
  bool rightSwitchStopEnabled(size_t motor);
  bool rightSwitchActive(size_t motor);

  void enableSwitchSoftStop(size_t motor);
  void disableSwitchSoftStop(size_t motor);
  bool switchSoftStopEnabled(size_t motor);

  void setReferenceSwitchToLeft(size_t motor);
  void setReferenceSwitchToRight(size_t motor);

  void startLatchPositionWaiting(size_t motor);
  bool latchPositionWaiting(size_t motor);
  int32_t getLatchPosition(size_t motor);

  void setPositionCompareMotor(size_t motor);

  struct Status
  {
    uint8_t at_target_position_0 : 1;
    uint8_t switch_left_0 : 1;
    uint8_t at_target_position_1 : 1;
    uint8_t switch_left_1 : 1;
    uint8_t at_target_position_2 : 1;
    uint8_t switch_left_2 : 1;
    uint8_t cover_datagram_waiting : 1;
    uint8_t interrupt : 1;
  };
  Status getStatus();

  uint32_t readRegister(uint8_t smda,
    uint8_t address);
  void writeRegister(uint8_t smda,
    uint8_t address,
    uint32_t data);
private:
  enum Mode
  {
    RAMP_MODE=0b00,
    SOFT_MODE=0b01,
    VELOCITY_MODE=0b10,
    HOLD_MODE=0b11,
  };

  struct ReferenceConfiguration
  {
    uint8_t disable_stop_l : 1;
    uint8_t disable_stop_r : 1;
    uint8_t soft_stop : 1;
    uint8_t ref_rnl : 1;
    uint8_t space : 4;
  };

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

  struct ClockConfiguration
  {
    uint16_t usrs : 3;
    uint16_t space0 : 5;
    uint16_t ramp_div : 4;
    uint16_t pulse_div : 4;
  };

  // SPISettings
  const static uint32_t SPI_CLOCK = 1000000;
#if defined(ARDUINO_ARCH_SAMD)
  const static BitOrder SPI_BIT_ORDER = MSBFIRST;
#else
  const static uint8_t SPI_BIT_ORDER = MSBFIRST;
#endif
  const static uint8_t SPI_MODE = SPI_MODE3;

  const static uint8_t MOTOR_COUNT = 3;

  const static uint32_t VERSION = 0x429101;

  const static uint8_t CLOCK_FREQUENCY_MAX = 32;
  const static uint8_t STEP_DIV_MAX = 15;
  const static uint32_t MHZ_PER_HZ = 1000000;
  const static uint32_t VELOCITY_CONSTANT = 65536;
  const static uint32_t VELOCITY_REGISTER_MIN = 0;
  const static uint32_t VELOCITY_REGISTER_MAX = 2047;
  const static uint32_t VELOCITY_REGISTER_THRESHOLD = 1448;
  const static uint32_t ACCELERATION_REGISTER_MIN = 1;
  const static uint32_t ACCELERATION_REGISTER_MAX = 2047;
  const static uint32_t ACCELERATION_CONSTANT = 536870912; // (1 << 29)
  const static uint16_t VELOCITY_MIN_MIN = 1;

  Status status_;
  uint8_t clock_frequency_;
  uint8_t pulse_div_[MOTOR_COUNT];
  uint8_t ramp_div_[MOTOR_COUNT];

  // Datagrams
  const static uint8_t DATAGRAM_SIZE = 4;

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

  size_t chip_select_pin_;

  void setStepDirOutput();
  // void setSpiOutput();

  MisoDatagram writeRead(MosiDatagram mosi_datagram);

  int32_t unsignedToSigned(uint32_t input_value,
    uint8_t num_bits);

  void specifyClockFrequencyInMHz(uint8_t clock_frequency);

  void setOptimalStepDivHz(uint32_t velocity_max_hz);
  uint8_t getStepDiv();
  void setStepDiv(uint8_t step_div);
  double stepDivToStepTime(uint8_t step_div);

  int32_t convertVelocityToHz(uint8_t pulse_div,
    int16_t velocity);
  int16_t convertVelocityFromHz(uint8_t pulse_div,
    int32_t velocity);

  uint8_t findOptimalPulseDivHz(uint32_t velocity_max_hz);
  void setOptimalPulseDivHz(size_t motor,
    uint32_t velocity_max_hz);

  Mode getMode(size_t motor);
  void setMode(size_t motor,
    Mode mode);

  ReferenceConfiguration getReferenceConfiguration(size_t motor);

  InterfaceConfiguration getInterfaceConfiguration();

  SwitchState getSwitchState();

  ClockConfiguration getClockConfiguration(size_t motor);
  double getProportionalityFactor(size_t motor);

  double getStepTimeInMicroS();

  uint16_t getVelocityMin(size_t motor);
  void setVelocityMinInHz(size_t motor,
    uint32_t velocity_min_hz);

  uint16_t getVelocityMax(size_t motor);
  void setVelocityMaxInHz(size_t motor,
    uint32_t velocity_max_hz);

  int16_t getTargetVelocity(size_t motor);
  void setTargetVelocity(size_t motor,
    int16_t velocity);

  int16_t getActualVelocity(size_t motor);

  uint32_t convertAccelerationToHzPerS(uint8_t pulse_div,
    uint8_t ramp_div,
    uint32_t acceleration);
  uint32_t convertAccelerationFromHzPerS(uint8_t pulse_div,
    uint8_t ramp_div,
    uint32_t acceleration);

  uint8_t findOptimalRampDivHz(uint32_t velocity_max_hz,
    uint32_t acceleration_max_hz_per_s);
  void setOptimalRampDivHz(size_t motor,
    uint32_t velocity_max_hz,
    uint32_t acceleration_max_hz_per_s);

  const static uint8_t PULSE_DIV_MIN = 0;
  const static uint8_t PULSE_DIV_MAX = 13;
  uint32_t getVelocityMaxUpperLimitInHz(uint8_t pulse_div);

  const static uint8_t RAMP_DIV_MIN = 0;
  const static uint8_t RAMP_DIV_MAX = 13;
  uint32_t getAccelerationMaxUpperLimitInHzPerS(uint8_t pulse_div,
    uint8_t ramp_div);
  uint32_t getAccelerationMaxLowerLimitInHzPerS(uint8_t pulse_div,
    uint8_t ramp_div,
    uint32_t velocity_max);

  uint32_t getAccelerationMax(size_t motor);
  uint32_t setAccelerationMaxInHzPerS(size_t motor,
    uint32_t velocity_max_hz,
    uint32_t acceleration_max_hz_per_s);

  int16_t getActualAcceleration(size_t motor);

  void setOptimalPropFactor(size_t motor,
    uint32_t acceleration_max);

  void enableChipSelect();
  void disableChipSelect();
  void beginTransaction();
  void endTransaction();

protected:
  virtual void spiBegin();
  virtual void spiBeginTransaction(SPISettings);
  virtual void spiEndTransaction();
  virtual uint8_t spiTransfer(uint8_t);
};

#endif
