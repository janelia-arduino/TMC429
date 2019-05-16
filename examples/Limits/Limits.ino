#include <Arduino.h>
#include <SPI.h>
#include <Streaming.h>
#include <TMC429.h>


const long BAUD = 115200;
const int LOOP_DELAY = 2000;
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const long STEPS_PER_POSITION_UNITS = 91;
const int MOTOR = 0;
const long VELOCITY_MIN = 0;
const long VELOCITY_MAX = 720;
const long VELOCITY_INC = 60;

// Instantiate TMC429
TMC429 tmc429;

bool communicating;
uint32_t velocity;
uint32_t acceleration_upper_limit;
uint32_t acceleration_lower_limit;
uint32_t acceleration;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  tmc429.setup(CHIP_SELECT_PIN,CLOCK_FREQUENCY_MHZ);

  tmc429.initialize();
}

void loop()
{
  velocity = VELOCITY_INC;
  while (velocity <= VELOCITY_MAX)
  {
    acceleration_upper_limit = tmc429.getAccelerationMaxUpperLimitInHzPerS(velocity * STEPS_PER_POSITION_UNITS) / STEPS_PER_POSITION_UNITS;
    acceleration_lower_limit = tmc429.getAccelerationMaxLowerLimitInHzPerS(velocity * STEPS_PER_POSITION_UNITS) / STEPS_PER_POSITION_UNITS;
    Serial << "velocity = " << velocity << ", acceleration_upper_limit = " << acceleration_upper_limit<< ", acceleration_lower_limit = " << acceleration_lower_limit << "\n";
    velocity += VELOCITY_INC;
  }

  Serial << "\n";
  delay(LOOP_DELAY);
}
