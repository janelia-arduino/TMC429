#include <Arduino.h>
#include <TMC2209.h>
#include <TMC429.h>

const long SERIAL_BAUD_RATE = 115200;
const int LOOP_DELAY = 1000;

// Stepper driver settings
HardwareSerial & serial_stream = Serial1;
const int RUN_CURRENT = 40;
const int HOLD_CURRENT = 0;
const int HOLD_DELAY = 0;
const int MICROSTEPS_PER_STEP = 256;
const TMC2209::StandstillMode STANDSTILL_MODE = TMC2209::STRONG_BRAKING;
const int PWM_OFFSET = 36;
const int PWM_GRADIENT = 0;
const int COOL_STEP_DURATION_THRESHOLD = 2000;
const TMC2209::CurrentIncrement COOL_STEP_CURRENT_INCREMENT = TMC2209::CURRENT_INCREMENT_4;
const int COOL_STEP_LOWER_THRESHOLD = 1;
const int COOL_STEP_UPPER_THRESHOLD = 0;

// Instantiate stepper driver
TMC2209 stepper_driver;

// Stepper controller settings
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const int MOTOR_INDEX = 0;
const int STEPS_PER_REV = 200;
const int REVS_PER_SEC_MAX = 2;
const int INC_PER_REV = 5;
const int ACCELERATION_MAX = 50000;
const int MICROSTEPS_PER_REV = STEPS_PER_REV*MICROSTEPS_PER_STEP;
const long VELOCITY_MAX = MICROSTEPS_PER_REV*REVS_PER_SEC_MAX;
const long VELOCITY_MIN = 100;
const long VELOCITY_INC = MICROSTEPS_PER_REV/INC_PER_REV;

// Instantiate stepper controller
TMC429 stepper_controller;

long target_velocity, actual_velocity;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);
  stepper_driver.disableInverseMotorDirection();
  stepper_driver.setRunCurrent(RUN_CURRENT);
  stepper_driver.setHoldCurrent(HOLD_CURRENT);
  stepper_driver.setHoldDelay(HOLD_DELAY);
  stepper_driver.setMicrostepsPerStep(MICROSTEPS_PER_STEP);
  stepper_driver.setStandstillMode(STANDSTILL_MODE);
  stepper_driver.setPwmOffset(PWM_OFFSET);
  stepper_driver.setPwmGradient(PWM_GRADIENT);
  stepper_driver.disableAutomaticCurrentScaling();
  stepper_driver.disableAutomaticGradientAdaptation();
  stepper_driver.setCoolStepDurationThreshold(COOL_STEP_DURATION_THRESHOLD);
  stepper_driver.setCoolStepCurrentIncrement(COOL_STEP_CURRENT_INCREMENT);
  stepper_driver.enableCoolStep(COOL_STEP_LOWER_THRESHOLD,COOL_STEP_UPPER_THRESHOLD);

  stepper_controller.setup(CHIP_SELECT_PIN,CLOCK_FREQUENCY_MHZ);
  stepper_controller.initialize();
  stepper_controller.disableInverseStepPolarity();
  stepper_controller.disableInverseDirPolarity();
  stepper_controller.setSwitchesActiveLow();
  stepper_controller.disableLeftSwitchStop(MOTOR_INDEX);
  stepper_controller.disableRightSwitches();
  stepper_controller.disableRightSwitchStop(MOTOR_INDEX);
  stepper_controller.disableSwitchSoftStop(MOTOR_INDEX);
  stepper_controller.setVelocityMode(MOTOR_INDEX);
  stepper_controller.setLimitsInHz(MOTOR_INDEX,VELOCITY_MIN,VELOCITY_MAX,ACCELERATION_MAX);

  stepper_driver.enable();

  target_velocity = -VELOCITY_MAX;
}

void loop()
{
  target_velocity += VELOCITY_INC;
  if (target_velocity > VELOCITY_MAX)
  {
    stepper_controller.stop(MOTOR_INDEX);
    Serial.println("stopping motor!");
    delay(LOOP_DELAY*5);

    target_velocity = -VELOCITY_MAX;
  }
  stepper_controller.setTargetVelocityInHz(MOTOR_INDEX,target_velocity);
  Serial.print("set target_velocity: ");
  Serial.println(target_velocity);
  Serial.print("at target_velocity: ");
  Serial.println(stepper_controller.atTargetVelocity(MOTOR_INDEX));

  target_velocity = stepper_controller.getTargetVelocityInHz(MOTOR_INDEX);

  do
  {
    actual_velocity = stepper_controller.getActualVelocityInHz(MOTOR_INDEX);
    Serial.print("actual_velocity: ");
    Serial.println(actual_velocity);
    delay(LOOP_DELAY);
  }
  while (actual_velocity != target_velocity);

  Serial.print("at target_velocity: ");
  Serial.println(stepper_controller.atTargetVelocity(MOTOR_INDEX));
  Serial.println();
}
