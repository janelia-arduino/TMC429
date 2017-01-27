// ----------------------------------------------------------------------------
// TMC429.cpp
//
// Authors:
// Peter Polidoro polidorop@janelia.hhmi.org
// ----------------------------------------------------------------------------

#include "TMC429.h"


TMC429::TMC429()
{
  // initialized_ = false;
}

// TMC429::TMC429(const int cs_pin) :
//   cs_pin_(cs_pin)
// {
//   initialized_ = false;

//   pinMode(cs_pin_,OUTPUT);
//   digitalWrite(cs_pin_,HIGH);
// }

// TMC429::TMC429(const int cs_pin, const int reset_pin) :
//   cs_pin_(cs_pin),
//   reset_pin_(reset_pin)
// {
//   initialized_ = false;

//   pinMode(cs_pin,OUTPUT);
//   digitalWrite(cs_pin,HIGH);

//   pinMode(reset_pin,OUTPUT);
//   digitalWrite(reset_pin,HIGH);
// }

// void TMC429::setup(const int ic_count, const boolean spi_reset)
// {
//   spi_reset_ = spi_reset;
//   if ((0 < ic_count) && (ic_count <= IC_COUNT_MAX))
//   {
//     ic_count_ = ic_count;
//   }
//   else
//   {
//     ic_count_ = IC_COUNT_MIN;
//   }
//   spiBegin();
//   setAllChannelsMapTrue();
//   setAllChannelsBooleanAnd();
//   setAllChannelsOff();
//   initialized_ = true;
// }

// void TMC429::setChannels(uint32_t channels)
// {
//   if (spi_reset_)
//   {
//     spiBegin();
//   }
//   digitalWrite(cs_pin_,LOW);
//   noInterrupts();
//   channels_ = channels;
//   for (int ic = (ic_count_ - 1); ic >= 0; --ic)
//   {
//     SPI.transfer(CMD_WRITE + ADDR_CTL);
//     SPI.transfer(channels_>>(ic*8));
//   }
//   interrupts();
//   digitalWrite(cs_pin_,HIGH);
//   digitalRead(cs_pin_);
// }

// void TMC429::setChannelOn(int channel)
// {
//   if ((0 <= channel) && (channel < CHANNEL_COUNT_MAX))
//   {
//     uint32_t bit = 1;
//     bit = bit << channel;
//     noInterrupts();
//     channels_ |= bit;
//     interrupts();
//     setChannels(channels_);
//   }
// }

// void TMC429::setChannelOff(int channel)
// {
//   if ((0 <= channel) && (channel < CHANNEL_COUNT_MAX))
//   {
//     uint32_t bit = 1;
//     bit = bit << channel;
//     noInterrupts();
//     channels_ &= ~bit;
//     interrupts();
//     setChannels(channels_);
//   }
// }

// void TMC429::setChannelsOn(uint32_t channels)
// {
//   noInterrupts();
//   channels_ |= channels;
//   interrupts();
//   setChannels(channels_);
// }

// void TMC429::setChannelsOff(uint32_t channels)
// {
//   noInterrupts();
//   channels_ &= ~channels;
//   interrupts();
//   setChannels(channels_);
// }

// void TMC429::toggleChannel(int channel)
// {
//   if ((0 <= channel) && (channel < CHANNEL_COUNT_MAX))
//   {
//     uint32_t bit = 1;
//     bit = bit << channel;
//     noInterrupts();
//     channels_ ^= bit;
//     interrupts();
//     setChannels(channels_);
//   }
// }

// void TMC429::toggleChannels(uint32_t channels)
// {
//   noInterrupts();
//   channels_ ^= channels;
//   interrupts();
//   setChannels(channels_);
// }

// void TMC429::toggleAllChannels()
// {
//   noInterrupts();
//   channels_ = ~channels_;
//   interrupts();
//   setChannels(channels_);
// }

// void TMC429::setAllChannelsOn()
// {
//   uint32_t bit = 1;
//   noInterrupts();
//   channels_ = (bit << getChannelCount()) - 1;
//   interrupts();
//   setChannels(channels_);
// }

// void TMC429::setAllChannelsOff()
// {
//   noInterrupts();
//   channels_ = 0;
//   interrupts();
//   setChannels(channels_);
// }

// void TMC429::setChannelOnAllOthersOff(int channel)
// {
//   if ((0 <= channel) && (channel < CHANNEL_COUNT_MAX))
//   {
//     uint32_t bit = 1;
//     bit = bit << channel;
//     noInterrupts();
//     channels_ = bit;
//     interrupts();
//     setChannels(channels_);
//   }
// }

// void TMC429::setChannelOffAllOthersOn(int channel)
// {
//   if ((0 <= channel) && (channel < CHANNEL_COUNT_MAX))
//   {
//     uint32_t bit = 1;
//     bit = bit << channel;
//     noInterrupts();
//     channels_ = ~bit;
//     interrupts();
//     setChannels(channels_);
//   }
// }

// void TMC429::setChannelsOnAllOthersOff(uint32_t channels)
// {
//   noInterrupts();
//   channels_ = channels;
//   interrupts();
//   setChannels(channels_);
// }

// void TMC429::setChannelsOffAllOthersOn(uint32_t channels)
// {
//   noInterrupts();
//   channels_ = ~channels;
//   interrupts();
//   setChannels(channels_);
// }

// uint32_t TMC429::getChannelsOn()
// {
//   return channels_;
// }

// int TMC429::getChannelCount()
// {
//   return ic_count_*CHANNEL_COUNT_PER_IC;
// }

// void TMC429::reset()
// {
//   digitalWrite(reset_pin_,LOW);
//   delay(RESET_DELAY);
//   digitalWrite(reset_pin_,HIGH);
//   noInterrupts();
//   channels_ = 0;
//   interrupts();
// }

// void TMC429::setChannelsMap(uint32_t channels)
// {
//   if (spi_reset_)
//   {
//     spiBegin();
//   }
//   digitalWrite(cs_pin_,LOW);
//   noInterrupts();
//   mapped_ = channels;
//   for (int ic = (ic_count_ - 1); ic >= 0; --ic)
//   {
//     SPI.transfer(CMD_WRITE + ADDR_MAP);
//     SPI.transfer(mapped_>>(ic*8));
//   }
//   interrupts();
//   digitalWrite(cs_pin_,HIGH);
//   digitalRead(cs_pin_);
// }

// void TMC429::setChannelMapTrue(int channel)
// {
//   if ((0 <= channel) && (channel < CHANNEL_COUNT_MAX))
//   {
//     uint32_t bit = 1;
//     bit = bit << channel;
//     noInterrupts();
//     mapped_ |= bit;
//     interrupts();
//     setChannelsMap(mapped_);
//   }
// }

// void TMC429::setChannelMapFalse(int channel)
// {
//   if ((0 <= channel) && (channel < CHANNEL_COUNT_MAX))
//   {
//     uint32_t bit = 1;
//     bit = bit << channel;
//     noInterrupts();
//     mapped_ &= ~bit;
//     interrupts();
//     setChannelsMap(mapped_);
//   }
// }

// void TMC429::setAllChannelsMapTrue()
// {
//   uint32_t bit = 1;
//   noInterrupts();
//   mapped_ = (bit << getChannelCount()) - 1;
//   interrupts();
//   setChannelsMap(mapped_);
// }

// void TMC429::setAllChannelsMapFalse()
// {
//   noInterrupts();
//   mapped_ = 0;
//   interrupts();
//   setChannelsMap(mapped_);
// }

// void TMC429::setChannelsBoolean(uint32_t bool_state)
// {
//   if (spi_reset_)
//   {
//     spiBegin();
//   }
//   digitalWrite(cs_pin_,LOW);
//   noInterrupts();
//   bool_state_ = bool_state;
//   for (int ic = (ic_count_ - 1); ic >= 0; --ic)
//   {
//     SPI.transfer(CMD_WRITE + ADDR_BOL);
//     SPI.transfer(bool_state_>>(ic*8));
//   }
//   interrupts();
//   digitalWrite(cs_pin_,HIGH);
//   digitalRead(cs_pin_);
// }

// void TMC429::setChannelBooleanAnd(int channel)
// {
//   if ((0 <= channel) && (channel < CHANNEL_COUNT_MAX))
//   {
//     uint32_t bit = 1;
//     bit = bit << channel;
//     noInterrupts();
//     bool_state_ |= bit;
//     interrupts();
//     setChannelsBoolean(bool_state_);
//   }
// }

// void TMC429::setChannelBooleanOr(int channel)
// {
//   if ((0 <= channel) && (channel < CHANNEL_COUNT_MAX))
//   {
//     uint32_t bit = 1;
//     bit = bit << channel;
//     noInterrupts();
//     bool_state_ &= ~bit;
//     interrupts();
//     setChannelsBoolean(bool_state_);
//   }
// }

// void TMC429::setAllChannelsBooleanAnd()
// {
//   uint32_t bit = 1;
//   noInterrupts();
//   bool_state_ = (bit << getChannelCount()) - 1;
//   interrupts();
//   setChannelsBoolean(bool_state_);
// }

// void TMC429::setAllChannelsBooleanOr()
// {
//   noInterrupts();
//   bool_state_ = 0;
//   interrupts();
//   setChannelsBoolean(bool_state_);
// }

void TMC429::spiBegin()
{
  // SPI.setDataMode(SPI_MODE1);
  // SPI.setClockDivider(SPI_CLOCK_DIV4);
  // SPI.setBitOrder(MSBFIRST);
  // SPI.begin();
}

