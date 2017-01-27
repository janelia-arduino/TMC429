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


class TMC429
{
public:
  TMC429();
  // TMC429(int cs_pin);
  // TMC429(int cs_pin, int reset_pin);

  // void setup(int ic_count=1, boolean spi_reset=false);
  // void setChannels(uint32_t channels);
  // void setChannelOn(int channel);
  // void setChannelOff(int channel);
  // void setChannelsOn(uint32_t channels);
  // void setChannelsOff(uint32_t channels);
  // void toggleChannel(int channel);
  // void toggleChannels(uint32_t channels);
  // void toggleAllChannels();
  // void setAllChannelsOn();
  // void setAllChannelsOff();
  // void setChannelOnAllOthersOff(int channel);
  // void setChannelOffAllOthersOn(int channel);
  // void setChannelsOnAllOthersOff(uint32_t channels);
  // void setChannelsOffAllOthersOn(uint32_t channels);
  // uint32_t getChannelsOn();
  // int getChannelCount();
  // void reset();
  // void setChannelsMap(uint32_t channels);
  // void setChannelMapTrue(int channel);
  // void setChannelMapFalse(int channel);
  // void setAllChannelsMapTrue();
  // void setAllChannelsMapFalse();
  // void setChannelsBoolean(uint32_t bool_state);
  // void setChannelBooleanAnd(int channel);
  // void setChannelBooleanOr(int channel);
  // void setAllChannelsBooleanAnd();
  // void setAllChannelsBooleanOr();

private:
  // const static int IC_COUNT_MIN = 1;
  // const static int IC_COUNT_MAX = 4;

  // const static int CHANNEL_COUNT_PER_IC = 8;
  // const static int CHANNEL_COUNT_MAX = 32;

  // const static int RESET_DELAY = 200;

  // const static byte CMD_DIAGNOSIS = 0b11<<6;
  // const static byte CMD_READ = 0b01<<6;
  // const static byte CMD_RESET = 0b10<<6;
  // const static byte CMD_WRITE = 0b11<<6;

  // const static byte ADDR_MAP = 0b001; // Input Mapping Register
  // const static byte ADDR_BOL = 0b010; // Boolean Operation Register
  // const static byte ADDR_OVL = 0b011; // Overload Behavior Register
  // const static byte ADDR_OVT = 0b100; // Overtemperature Behavior Register
  // const static byte ADDR_SLE = 0b101; // Switching Speed / Slew Rate Register
  // const static byte ADDR_STA = 0b110; // Output State Register
  // const static byte ADDR_CTL = 0b111; // Serial Output Control Register

  // int cs_pin_;
  // int reset_pin_;
  // boolean initialized_;
  // uint32_t channels_;
  // uint32_t mapped_;
  // uint32_t bool_state_;
  // int ic_count_;
  // boolean spi_reset_;

  void spiBegin();
};

#endif
