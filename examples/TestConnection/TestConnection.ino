#include "Arduino.h"
#include "SPI.h"
#include "Streaming.h"
#include "TMC429.h"

const int BAUDRATE = 115200;
const int LOOP_DELAY = 1000;
const int CS_PIN = 10;

// Instantiate TMC429
TMC429 stepper_controller = TMC429(CS_PIN);

void setup()
{
  // Setup serial communications
  Serial.begin(BAUDRATE);

  stepper_controller.setup();
  // stepper_controller.setup(IC_COUNT,SPI_RESET);
  // channel_count = stepper_controller.getChannelCount();
}

void loop()
{
  stepper_controller.test();
  delay(LOOP_DELAY);
  // for (int channel = 0; channel < channel_count; channel++)
  // {
  //   if (channel%2 == 0)
  //   {
  //     stepper_controller.setChannelOn(channel);
  //     Serial << "set channel " << channel << " on" << endl;
  //   }
  //   else
  //   {
  //     if (channel > 0)
  //     {
  //       stepper_controller.setChannelOff(channel-1);
  //       Serial << "set channel " << (channel-1) << " off" << endl;
  //     }
  //     stepper_controller.setChannelOn(channel);
  //     Serial << "set channel " << (channel) << " on" << endl;
  //   }
  //   delay(LOOP_DELAY);
  // }
  // // Set all channels to off
  // stepper_controller.setChannels(0);
}
