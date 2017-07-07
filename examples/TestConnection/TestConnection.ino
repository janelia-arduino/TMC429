#include "Arduino.h"
#include "SPI.h"
#include "Streaming.h"
#include "TMC429.h"

const long BAUDRATE = 115200;
const int LOOP_DELAY = 2000;
const int CS_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 16;
const int MOTOR = 0;

// Instantiate TMC429
TMC429 tmc429;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUDRATE);

  tmc429.setup(CS_PIN,CLOCK_FREQUENCY_MHZ);

  uint32_t version = tmc429.getVersion();
  Serial << "version: " << _HEX(version) << "\n";

  bool check_version = tmc429.checkVersion();
  Serial << "check_version: " << check_version << "\n";

  tmc429.setStepDirOutput();

}

void loop()
{
  tmc429.setVelocityMode(MOTOR);
  // tmc429.setRampMode(MOTOR);

  uint32_t velocity_max_upper_limit = tmc429.getVelocityMaxUpperLimitInHz();
  Serial << "velocity_max_upper_limit: " << velocity_max_upper_limit << "\n";

  uint32_t velocity_min = 100;
  uint32_t velocity_max = 100000;
  uint32_t acceleration_max = 40000;
  tmc429.setLimitsInHz(MOTOR,velocity_min,velocity_max,acceleration_max);
  Serial << "setLimitsInHz: " << velocity_min << ", " << velocity_max << ", " << acceleration_max << "\n";

  acceleration_max = tmc429.getAccelerationMaxInHzPerS(MOTOR);
  Serial << "acceleration_max: " << acceleration_max << "\n";

  acceleration_max = tmc429.getAccelerationMaxUpperLimitInHzPerS(MOTOR);
  Serial << "acceleration_max_upper_limit: " << acceleration_max << "\n";

  uint32_t acceleration_actual = tmc429.getActualAccelerationInHzPerS(MOTOR);
  Serial << "acceleration_actual: " << acceleration_actual << "\n";

  velocity_min = tmc429.getVelocityMinInHz(MOTOR);
  Serial << "velocity_min: " << velocity_min << "\n";

  velocity_max = tmc429.getVelocityMaxInHz(MOTOR);
  Serial << "velocity_max: " << velocity_max << "\n";

  long target_velocity = -50000;
  tmc429.setTargetVelocityInHz(MOTOR,target_velocity);
  Serial << "set target_velocity: " << target_velocity << "\n";

  target_velocity = tmc429.getTargetVelocityInHz(MOTOR);
  Serial << "target_velocity: " << target_velocity << "\n";

  long actual_velocity = tmc429.getActualVelocityInHz(MOTOR);
  Serial << "actual_velocity: " << actual_velocity << "\n";

  int32_t target_position = 12345;
  tmc429.setTargetPosition(MOTOR,target_position);
  Serial << "set target_position: " << target_position << "\n";

  target_position = tmc429.getTargetPosition(MOTOR);
  Serial << "target_position: " << target_position << "\n";

  int32_t actual_position = tmc429.getActualPosition(MOTOR);
  Serial << "actual_position: " << actual_position << "\n";

  Serial << "at_target_position = " << tmc429.atTargetPosition(MOTOR) << "\n";
  Serial << "at_target_velocity = " << tmc429.atTargetVelocity(MOTOR) << "\n";

  tmc429.enableRightSwitches();
  tmc429.setReferenceSwitchToLeft(MOTOR);

  tmc429.setSwitchesActiveLow();
  tmc429.disableSwitchSoftStop(MOTOR);
  tmc429.enableLeftSwitchStop(MOTOR);
  tmc429.enableRightSwitchStop(MOTOR);
  Serial << "left_switch_active = " << tmc429.leftSwitchActive(MOTOR) << "\n";
  Serial << "right_switch_active = " << tmc429.rightSwitchActive(MOTOR) << "\n";

  bool latch_position_waiting = tmc429.latchPositionWaiting(MOTOR);
  Serial << "latch_position_waiting: " << latch_position_waiting << "\n";
  if (!latch_position_waiting)
  {
    int32_t latch_position = tmc429.getLatchPosition(MOTOR);
    Serial << "latch_position: " << latch_position << "\n";
    tmc429.startLatchPositionWaiting(MOTOR);
  }

  tmc429.setPositionCompareMotor(MOTOR);

  // test private methods:
  // TMC429::Status status = tmc429.getStatus();
  // Serial << "status.at_target_position_0 = " << status.at_target_position_0 << "\n";
  // Serial << "status.switch_left_0 = " << status.switch_left_0 << "\n";
  // Serial << "status.at_target_position_1 = " << status.at_target_position_1 << "\n";
  // Serial << "status.switch_left_1 = " << status.switch_left_1 << "\n";
  // Serial << "status.at_target_position_2 = " << status.at_target_position_2 << "\n";
  // Serial << "status.switch_left_2 = " << status.switch_left_2 << "\n";

  // TMC429::ReferenceConfiguration ref_conf = tmc429.getReferenceConfiguration(MOTOR);
  // Serial << "ref_conf.disable_stop_l: " << ref_conf.disable_stop_l << "\n";
  // Serial << "ref_conf.disable_stop_r: " << ref_conf.disable_stop_r << "\n";
  // Serial << "ref_conf.soft_stop: " << ref_conf.soft_stop << "\n";
  // Serial << "ref_conf.ref_rnl: " << ref_conf.ref_rnl << "\n";

  // TMC429::InterfaceConfiguration if_conf = tmc429.getInterfaceConfiguration();
  // Serial << "if_conf.inv_ref: " << if_conf.inv_ref << "\n";
  // Serial << "if_conf.sdo_int: " << if_conf.sdo_int << "\n";
  // Serial << "if_conf.step_half: " << if_conf.step_half << "\n";
  // Serial << "if_conf.inv_stp: " << if_conf.inv_stp << "\n";
  // Serial << "if_conf.inv_dir: " << if_conf.inv_dir << "\n";
  // Serial << "if_conf.en_sd: " << if_conf.en_sd << "\n";
  // Serial << "if_conf.pos_comp_sel: " << if_conf.pos_comp_sel << "\n";
  // Serial << "if_conf.en_refr: " << if_conf.en_refr << "\n";

  // TMC429::SwitchState switch_state = tmc429.getSwitchState();
  // Serial << "switch_state.r0: " << switch_state.r0 << "\n";
  // Serial << "switch_state.l0: " << switch_state.l0 << "\n";
  // Serial << "switch_state.r1: " << switch_state.r1 << "\n";
  // Serial << "switch_state.l1: " << switch_state.l1 << "\n";
  // Serial << "switch_state.r2: " << switch_state.r2 << "\n";
  // Serial << "switch_state.l2: " << switch_state.l2 << "\n";

  // TMC429::ClockConfiguration clk_config = tmc429.getClockConfiguration(MOTOR);
  // Serial << "clk_config.usrs: " << clk_config.usrs << "\n";
  // Serial << "clk_config.ramp_div: " << clk_config.ramp_div << "\n";
  // Serial << "clk_config.pulse_div: " << clk_config.pulse_div << "\n";

  // double proportionality_factor = tmc429.getProportionalityFactor(MOTOR);
  // Serial << "proportionality_factor: " << proportionality_factor << "\n";

  // double step_time = tmc429.getStepTimeInMicroS();
  // Serial << "step_time: " << step_time << "\n";

  Serial << "\n";
  delay(LOOP_DELAY);
}
