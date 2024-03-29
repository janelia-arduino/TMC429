#include <TMC2209.h>
#include <TMC429.h>

const long SERIAL_BAUD_RATE = 115200;
const int SETUP_DELAY = 4000;
const int LOOP_DELAY = 1000;

// Stepper driver settings
HardwareSerial & serial_stream = Serial1;
const int MICROSTEPS_PER_STEP = 256;

// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const int RUN_CURRENT_PERCENT = 60;
const int STEPS_PER_REV = 200;
const int REVS_PER_SEC_MAX = 2;
const int MICROSTEPS_PER_REV = STEPS_PER_REV*MICROSTEPS_PER_STEP;
const long VELOCITY_MAX = REVS_PER_SEC_MAX * MICROSTEPS_PER_REV;

// Instantiate stepper driver
TMC2209 stepper_driver;

// Stepper controller settings
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const int MOTOR_INDEX = 0;
const long VELOCITY_INC = 5000;

// Instantiate stepper controller
TMC429 stepper_controller;

long hold_velocity, actual_velocity, delta_velocity;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_driver.setup(serial_stream);
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableAutomaticCurrentScaling();
  stepper_driver.enableAutomaticGradientAdaptation();
  stepper_driver.enableCoolStep();

  stepper_controller.setup(CHIP_SELECT_PIN, CLOCK_FREQUENCY_MHZ);
  stepper_controller.disableLeftSwitchStop(MOTOR_INDEX);
  stepper_controller.disableRightSwitches();
  stepper_controller.setHoldMode(MOTOR_INDEX);
  stepper_controller.setHoldVelocityMaxInHz(MOTOR_INDEX, VELOCITY_MAX);

  stepper_driver.enable();

  hold_velocity = 0;
  delta_velocity = VELOCITY_INC;
  stepper_controller.setHoldVelocityInHz(MOTOR_INDEX, hold_velocity);

  delay(SETUP_DELAY);
}

void loop()
{
  Serial.println("********************");
  Serial.println("Hold Mode");

  Serial.print("hold velocity (Hz): ");
  Serial.println(hold_velocity);

  actual_velocity = stepper_controller.getActualVelocityInHz(MOTOR_INDEX);
  Serial.print("actual velocity (Hz): ");
  Serial.println(actual_velocity);

  delay(LOOP_DELAY);

  hold_velocity += delta_velocity;
  if ((hold_velocity > VELOCITY_MAX) || (hold_velocity < -VELOCITY_MAX))
  {
    delta_velocity = -delta_velocity;
    hold_velocity += 2 * delta_velocity;
  }
  stepper_controller.setHoldVelocityInHz(MOTOR_INDEX, hold_velocity);
}
