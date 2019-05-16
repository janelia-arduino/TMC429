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

  // uint32_t velocity_max_upper_limit = tmc429.getVelocityMaxUpperLimitInHz() / STEPS_PER_POSITION_UNITS;
  // Serial << "velocity_max_upper_limit = " << velocity_max_upper_limit << "\n";

  uint32_t velocity_max;
  uint32_t acceleration_max_upper_limit;
  uint32_t acceleration_max_lower_limit;
  uint32_t acceleration_max;

  velocity_max = 360;
  Serial << "velocity_max = " << velocity_max << "\n";
  acceleration_max_upper_limit = tmc429.getAccelerationMaxUpperLimitInHzPerS(velocity_max * STEPS_PER_POSITION_UNITS) / STEPS_PER_POSITION_UNITS;
  Serial << "acceleration_max_upper_limit = " << acceleration_max_upper_limit << "\n";
  tmc429.setLimitsInHz(MOTOR,VELOCITY_MIN,velocity_max * STEPS_PER_POSITION_UNITS,acceleration_max_upper_limit * STEPS_PER_POSITION_UNITS);
  acceleration_max = tmc429.getAccelerationMaxInHzPerS(MOTOR) / STEPS_PER_POSITION_UNITS;
  Serial << "acceleration_max = " << acceleration_max << "\n";
  acceleration_max_lower_limit = tmc429.getAccelerationMaxLowerLimitInHzPerS(velocity_max * STEPS_PER_POSITION_UNITS) / STEPS_PER_POSITION_UNITS;
  Serial << "acceleration_max_lower_limit = " << acceleration_max_lower_limit << "\n";
  tmc429.setLimitsInHz(0,0,velocity_max * STEPS_PER_POSITION_UNITS,acceleration_max_lower_limit * STEPS_PER_POSITION_UNITS);
  acceleration_max = tmc429.getAccelerationMaxInHzPerS(MOTOR) / STEPS_PER_POSITION_UNITS;
  Serial << "acceleration_max = " << acceleration_max << "\n";

  velocity_max = 720;
  Serial << "velocity_max = " << velocity_max << "\n";
  acceleration_max_upper_limit = tmc429.getAccelerationMaxUpperLimitInHzPerS(velocity_max * STEPS_PER_POSITION_UNITS) / STEPS_PER_POSITION_UNITS;
  Serial << "acceleration_max_upper_limit = " << acceleration_max_upper_limit << "\n";
  tmc429.setLimitsInHz(MOTOR,VELOCITY_MIN,velocity_max * STEPS_PER_POSITION_UNITS,acceleration_max_upper_limit * STEPS_PER_POSITION_UNITS);
  acceleration_max = tmc429.getAccelerationMaxInHzPerS(MOTOR) / STEPS_PER_POSITION_UNITS;
  Serial << "acceleration_max = " << acceleration_max << "\n";
  acceleration_max_lower_limit = tmc429.getAccelerationMaxLowerLimitInHzPerS(velocity_max * STEPS_PER_POSITION_UNITS) / STEPS_PER_POSITION_UNITS;
  Serial << "acceleration_max_lower_limit = " << acceleration_max_lower_limit << "\n";
  tmc429.setLimitsInHz(0,0,velocity_max * STEPS_PER_POSITION_UNITS,acceleration_max_lower_limit * STEPS_PER_POSITION_UNITS);
  acceleration_max = tmc429.getAccelerationMaxInHzPerS(MOTOR) / STEPS_PER_POSITION_UNITS;
  Serial << "acceleration_max = " << acceleration_max << "\n";

  Serial << "\n";
  delay(LOOP_DELAY);
}
