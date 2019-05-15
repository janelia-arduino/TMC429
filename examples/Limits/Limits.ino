#include <Arduino.h>
#include <SPI.h>
#include <Streaming.h>
#include <TMC429.h>


const long BAUD = 115200;
const int LOOP_DELAY = 2000;
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const long STEPS_PER_POSITION_UNITS = 91;
// const uint16_t VELOCITY_COUNT = 100;

// Instantiate TMC429
TMC429 tmc429;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  tmc429.setup(CHIP_SELECT_PIN,CLOCK_FREQUENCY_MHZ);

  tmc429.initialize();

}

void loop()
{
  bool communicating = tmc429.communicating();
  Serial << "communicating: " << communicating << "\n";

  uint32_t velocity_max_upper_limit = tmc429.getVelocityMaxUpperLimitInHz() / STEPS_PER_POSITION_UNITS;
  Serial << "velocity_max_upper_limit: " << velocity_max_upper_limit << "\n";

  for (uint8_t pulse_div=tmc429.PULSE_DIV_MIN; pulse_div<=tmc429.PULSE_DIV_MAX; ++pulse_div)
  {
    uint32_t velocity_max_upper_limit = tmc429.getVelocityMaxUpperLimitInHz(pulse_div) / STEPS_PER_POSITION_UNITS;
    Serial << "pulse_div = " << pulse_div << ", velocity_max_upper_limit: " << velocity_max_upper_limit << "\n";
    for (uint8_t ramp_div=tmc429.RAMP_DIV_MIN; ramp_div<=tmc429.RAMP_DIV_MAX; ++ramp_div)
    {
      if ((ramp_div - pulse_div + 12) >= 1)
      {
        uint32_t acceleration_max_upper_limit = tmc429.getAccelerationMaxUpperLimitInHzPerS(pulse_div,ramp_div) / STEPS_PER_POSITION_UNITS;
        Serial << "pulse_div = " << pulse_div << ", ramp_div = " << ramp_div << ", acceleration_max_upper_limit: " << acceleration_max_upper_limit << "\n";
        uint32_t acceleration_max_lower_limit = tmc429.getAccelerationMaxLowerLimitInHzPerS(pulse_div,ramp_div) / STEPS_PER_POSITION_UNITS;
        Serial << "pulse_div = " << pulse_div << ", ramp_div = " << ramp_div << ", acceleration_max_lower_limit: " << acceleration_max_lower_limit << "\n";
      }
    }
  }

  // uint32_t velocity_max;
  // uint32_t velocity_inc = velocity_max_upper_limit / VELOCITY_COUNT;
  // uint32_t acceleration_max_upper_limit;
  // for (uint8_t velocity_i=1; velocity_i<=VELOCITY_COUNT; ++velocity_i)
  // {
  //   velocity_max = velocity_i * velocity_inc;
  //   acceleration_max_upper_limit = tmc429.getAccelerationMaxUpperLimitInHzPerS(velocity_max);
  //   Serial << "velocity_max = " << velocity_max << ", acceleration_max_upper_limit = " << acceleration_max_upper_limit << "\n";
  // }

  Serial << "\n";
  delay(LOOP_DELAY);
}
