#include "Arduino.h"
#include "SPI.h"
#include "Streaming.h"
#include "TMC429.h"
#include "TMC26X.h"


const int BAUDRATE = 115200;
const int LOOP_DELAY = 1000;
const int CS_PIN_429 = 10;
const int CLOCK_FREQUENCY_MHZ = 16;
const int MOTOR_COUNT = 3;
const int CS_PIN_26X[MOTOR_COUNT] = {9,8,7};
const int CURRENT_SCALE_PERCENT = 18;
const int STEPS_PER_REV = 200;
const int MICROSTEPS_PER_STEP = 256;
const int REVS_PER_SEC_MAX = 2;
const int INC_PER_REV = 5;
const int VELOCITY_MIN = 100;
const int ACCELERATION_MAX = 50000;
const int MOTOR = 0;

TMC429 step_dir_controller;
TMC26X stepper_drivers[MOTOR_COUNT];
long velocity_inc, velocity_max;
long velocity_target, velocity_actual;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUDRATE);

  step_dir_controller.setup(CS_PIN_429,CLOCK_FREQUENCY_MHZ);

  bool check_version = step_dir_controller.checkVersion();
  Serial << "check_version: " << check_version << "\n";

  step_dir_controller.setStepDirOutput();

  const int microsteps_per_rev = STEPS_PER_REV*MICROSTEPS_PER_STEP;
  velocity_max = microsteps_per_rev*REVS_PER_SEC_MAX;
  velocity_inc = microsteps_per_rev/INC_PER_REV;
  for (int motor=0; motor<MOTOR_COUNT; ++motor)
  {
    step_dir_controller.setMode(MOTOR,TMC429::VELOCITY_MODE);
    step_dir_controller.setLimitsInHz(motor,VELOCITY_MIN,velocity_max,ACCELERATION_MAX);

    stepper_drivers[motor].setup(CS_PIN_26X[motor]);
    stepper_drivers[motor].setStepDirInput();
    stepper_drivers[motor].setMicrostepsPerStepTo256();
    stepper_drivers[motor].setDefaultChopperConfig();
    stepper_drivers[motor].disableCoolStep();
    stepper_drivers[motor].setCurrentScalePercent(CURRENT_SCALE_PERCENT);
  }

  velocity_target = -velocity_max;

}

void loop()
{
  velocity_target += velocity_inc;
  if (velocity_target > velocity_max)
  {
    step_dir_controller.stop(MOTOR);
    Serial << "stopping motor!\n";
    delay(LOOP_DELAY*5);

    velocity_target = -velocity_max;
  }
  step_dir_controller.setVelocityTargetInHz(MOTOR,velocity_target);
  Serial << "set velocity_target: " << velocity_target << "\n";

  velocity_target = step_dir_controller.getVelocityTargetInHz(MOTOR);

  do
  {
    velocity_actual = step_dir_controller.getVelocityActualInHz(MOTOR);
    Serial << "velocity_actual: " << velocity_actual << " velocity_target: " << velocity_target << "\n";
    delay(LOOP_DELAY);
  }
  while (velocity_actual != velocity_target);

  Serial << "velocity_actual == velocity_target == : " << velocity_actual << "!\n";
  Serial << "\n";
}
