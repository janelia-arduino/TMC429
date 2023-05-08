#include <TMC429.h>


const long BAUD = 115200;
const int LOOP_DELAY = 2000;
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const int MOTOR = 0;

// Instantiate TMC429
TMC429 tmc429;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  tmc429.setup(CHIP_SELECT_PIN,CLOCK_FREQUENCY_MHZ);
}

void loop()
{
  bool communicating = tmc429.communicating();
  Serial.print("communicating: ");
  Serial.println(communicating);

  uint32_t version = tmc429.getVersion();
  Serial.print("version: ");
  Serial.println(version, HEX);

  tmc429.setVelocityMode(MOTOR);
  // tmc429.setRampMode(MOTOR);

  uint32_t velocity_max_upper_limit = tmc429.getVelocityMaxUpperLimitInHz();
  Serial.print("velocity_max_upper_limit: ");
  Serial.println(velocity_max_upper_limit);

  uint32_t velocity_min = 100;
  uint32_t velocity_max = 100000;
  uint32_t acceleration_max = 40000;
  tmc429.setLimitsInHz(MOTOR,velocity_min,velocity_max,acceleration_max);
  Serial.print("setLimitsInHz: ");
  Serial.print(velocity_min);
  Serial.print(", ");
  Serial.print(velocity_max);
  Serial.print(", ");
  Serial.println(acceleration_max);

  acceleration_max = tmc429.getAccelerationMaxInHzPerS(MOTOR);
  Serial.print("acceleration_max: ");
  Serial.println(acceleration_max);

  acceleration_max = tmc429.getAccelerationMaxUpperLimitInHzPerS(MOTOR);
  Serial.print("acceleration_max_upper_limit: ");
  Serial.println(acceleration_max);

  uint32_t acceleration_actual = tmc429.getActualAccelerationInHzPerS(MOTOR);
  Serial.print("acceleration_actual: ");
  Serial.println(acceleration_actual);

  velocity_min = tmc429.getVelocityMinInHz(MOTOR);
  Serial.print("velocity_min: ");
  Serial.println(velocity_min);

  velocity_max = tmc429.getVelocityMaxInHz(MOTOR);
  Serial.print("velocity_max: ");
  Serial.println(velocity_max);

  long target_velocity = -50000;
  tmc429.setTargetVelocityInHz(MOTOR,target_velocity);
  Serial.print("set target_velocity: ");
  Serial.println(target_velocity);

  target_velocity = tmc429.getTargetVelocityInHz(MOTOR);
  Serial.print("target_velocity: ");
  Serial.println(target_velocity);

  long actual_velocity = tmc429.getActualVelocityInHz(MOTOR);
  Serial.print("actual_velocity: ");
  Serial.println(actual_velocity);

  int32_t target_position = 12345;
  tmc429.setTargetPosition(MOTOR,target_position);
  Serial.print("set target_position: ");
  Serial.println(target_position);

  target_position = tmc429.getTargetPosition(MOTOR);
  Serial.print("set target_position: ");
  Serial.println(target_position);

  int32_t actual_position = tmc429.getActualPosition(MOTOR);
  Serial.print("actual_position: ");
  Serial.println(actual_position);

  Serial.print("at_target_position = ");
  Serial.println(tmc429.atTargetPosition(MOTOR));
  Serial.print("at_target_velocity = ");
  Serial.println(tmc429.atTargetVelocity(MOTOR));

  tmc429.disableRightSwitches();
  Serial.print("After disabling, right switches enabled = ");
  Serial.println(tmc429.rightSwitchesEnabled());
  tmc429.enableRightSwitches();
  Serial.print("After enabling, right switches enabled = ");
  Serial.println(tmc429.rightSwitchesEnabled());
  tmc429.setReferenceSwitchToLeft(MOTOR);

  tmc429.setSwitchesActiveLow();
  tmc429.disableSwitchSoftStop(MOTOR);
  Serial.print("After disabling, switch soft stop enabled = ");
  Serial.println(tmc429.switchSoftStopEnabled(MOTOR));
  tmc429.enableSwitchSoftStop(MOTOR);
  Serial.print("After enabling, switch soft stop enabled = ");
  Serial.println(tmc429.switchSoftStopEnabled(MOTOR));

  tmc429.disableLeftSwitchStop(MOTOR);
  Serial.print("After disabling, left switch stop enabled = ");
  Serial.println(tmc429.leftSwitchStopEnabled(MOTOR));
  tmc429.enableLeftSwitchStop(MOTOR);
  Serial.print("After enabling, left switch stop enabled = ");
  Serial.println(tmc429.leftSwitchStopEnabled(MOTOR));

  tmc429.disableRightSwitchStop(MOTOR);
  Serial.print("After disabling, right switch stop enabled = ");
  Serial.println(tmc429.rightSwitchStopEnabled(MOTOR));
  tmc429.enableRightSwitchStop(MOTOR);
  Serial.print("After enabling, right switch stop enabled = ");
  Serial.println(tmc429.rightSwitchStopEnabled(MOTOR));

  Serial.print("left_switch_active = ");
  Serial.println(tmc429.leftSwitchActive(MOTOR));
  Serial.print("right_switch_active = ");
  Serial.println(tmc429.rightSwitchActive(MOTOR));

  bool latch_position_waiting = tmc429.latchPositionWaiting(MOTOR);
  Serial.print("latch_position_waiting: ");
  Serial.println(latch_position_waiting);
  if (!latch_position_waiting)
  {
    int32_t latch_position = tmc429.getLatchPosition(MOTOR);
    Serial.print("latch_position: ");
    Serial.println(latch_position);
    tmc429.startLatchPositionWaiting(MOTOR);
  }

  tmc429.setPositionCompareMotor(MOTOR);

  TMC429::Status status = tmc429.getStatus();
  Serial.print("status.at_target_position_0 = ");
  Serial.println(status.at_target_position_0);
  Serial.print("status.switch_left_0 = ");
  Serial.println(status.switch_left_0);
  Serial.print("status.at_target_position_1 = ");
  Serial.println(status.at_target_position_1);
  Serial.print("status.switch_left_1 = ");
  Serial.println(status.switch_left_1);
  Serial.print("status.at_target_position_2 = ");
  Serial.println(status.at_target_position_2);
  Serial.print("status.switch_left_2 = ");
  Serial.println(status.switch_left_2);


  Serial.println();
  delay(LOOP_DELAY);
}
