#include <Arduino.h>
#include <SPI.h>
#include <Streaming.h>
#include <TMC429.h>
#include <TMC26X.h>


const long BAUD = 115200;
const int LOOP_DELAY = 1000;
const int CHIP_SELECT_PIN_429 = 10;
const int CLOCK_FREQUENCY_MHZ = 16;
const int MOTOR_COUNT = 3;
const int CHIP_SELECT_PIN_26X[MOTOR_COUNT] = {9,8,7};
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
long target_velocity, actual_velocity;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  step_dir_controller.setup(CHIP_SELECT_PIN_429,CLOCK_FREQUENCY_MHZ);

  bool communicating = step_dir_controller.communicating();
  Serial << "communicating: " << communicating << "\n";

  step_dir_controller.initialize();

  const int microsteps_per_rev = STEPS_PER_REV*MICROSTEPS_PER_STEP;
  velocity_max = microsteps_per_rev*REVS_PER_SEC_MAX;
  velocity_inc = microsteps_per_rev/INC_PER_REV;
  for (int motor=0; motor<MOTOR_COUNT; ++motor)
  {
    step_dir_controller.setVelocityMode(motor);
    step_dir_controller.setLimitsInHz(motor,VELOCITY_MIN,velocity_max,ACCELERATION_MAX);
    step_dir_controller.disableLeftSwitchStop(motor);
    step_dir_controller.disableRightSwitchStop(motor);

    stepper_drivers[motor].setup(CHIP_SELECT_PIN_26X[motor]);
    stepper_drivers[motor].setStepDirInput();
    stepper_drivers[motor].setDefaultChopperConfig();
    stepper_drivers[motor].disableCoolStep();
    stepper_drivers[motor].setCurrentScalePercent(CURRENT_SCALE_PERCENT);
    stepper_drivers[motor].setMicrostepsPerStepTo256();
  }

  target_velocity = -velocity_max;

}

void loop()
{
  target_velocity += velocity_inc;
  if (target_velocity > velocity_max)
  {
    step_dir_controller.stop(MOTOR);
    Serial << "stopping motor!\n";
    delay(LOOP_DELAY*5);

    target_velocity = -velocity_max;
  }
  step_dir_controller.setTargetVelocityInHz(MOTOR,target_velocity);
  Serial << "set target_velocity: " << target_velocity << "\n";
  Serial << "at target_velocity: " << step_dir_controller.atTargetVelocity(MOTOR) << "\n";

  target_velocity = step_dir_controller.getTargetVelocityInHz(MOTOR);

  do
  {
    actual_velocity = step_dir_controller.getActualVelocityInHz(MOTOR);
    Serial << "actual_velocity: " << actual_velocity << "\n";
    delay(LOOP_DELAY);
  }
  while (actual_velocity != target_velocity);

  Serial << "at target_velocity: " << step_dir_controller.atTargetVelocity(MOTOR) << "\n";
  Serial << "\n";
}
