#include "Arduino.h"
#include "SPI.h"
#include "Streaming.h"
#include "TMC429.h"

const int BAUDRATE = 115200;
const int LOOP_DELAY = 2000;
const int CS_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const int MOTOR = 0;

// Instantiate TMC429
TMC429 step_dir_controller;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUDRATE);

  step_dir_controller.setup(CS_PIN,CLOCK_FREQUENCY_MHZ);

  uint32_t version = step_dir_controller.getVersion();
  Serial << "version: " << _HEX(version) << "\n";

  bool check_version = step_dir_controller.checkVersion();
  Serial << "check_version: " << check_version << "\n";

  step_dir_controller.setStepDirOutput();

}

void loop()
{
  step_dir_controller.setMode(MOTOR,TMC429::VELOCITY_MODE);

  TMC429::Mode mode = step_dir_controller.getMode(MOTOR);
  if (mode == TMC429::RAMP_MODE)
  {
    Serial << "mode: ramp_mode\n";
  }
  else if (mode == TMC429::SOFT_MODE)
  {
    Serial << "mode: soft_mode\n";
  }
  else if (mode == TMC429::VELOCITY_MODE)
  {
    Serial << "mode: velocity_mode\n";
  }
  else if (mode == TMC429::HOLD_MODE)
  {
    Serial << "mode: hold_mode\n";
  }

  uint32_t velocity_max_max = step_dir_controller.getVelocityMaxMaxInHz();
  Serial << "velocity_max_max: " << velocity_max_max << "\n";

  uint32_t velocity_min = 100;
  uint32_t velocity_max = 2000;
  uint32_t acceleration_max = 2000;
  step_dir_controller.setLimitsInHz(MOTOR,velocity_min,velocity_max,acceleration_max);
  Serial << "setLimitsInHz: " << velocity_min << ", " << velocity_max << ", " << acceleration_max << "\n";

  acceleration_max = step_dir_controller.getAccelerationMaxInHzPerS(MOTOR);
  Serial << "acceleration_max: " << acceleration_max << "\n";

  uint32_t acceleration_actual = step_dir_controller.getAccelerationActualInHzPerS(MOTOR);
  Serial << "acceleration_actual: " << acceleration_actual << "\n";

  velocity_min = step_dir_controller.getVelocityMinInHz(MOTOR);
  Serial << "velocity_min: " << velocity_min << "\n";

  velocity_max = step_dir_controller.getVelocityMaxInHz(MOTOR);
  Serial << "velocity_max: " << velocity_max << "\n";

  int16_t velocity_target = -321;
  step_dir_controller.setVelocityTargetInHz(MOTOR,velocity_target);
  Serial << "set velocity_target: " << velocity_target << "\n";

  velocity_target = step_dir_controller.getVelocityTargetInHz(MOTOR);
  Serial << "velocity_target: " << velocity_target << "\n";

  int16_t velocity_actual = step_dir_controller.getVelocityActualInHz(MOTOR);
  Serial << "velocity_actual: " << velocity_actual << "\n";

  uint32_t position_target = 12345;
  step_dir_controller.setPositionTarget(MOTOR,position_target);
  Serial << "set position_target: " << position_target << "\n";

  position_target = step_dir_controller.getPositionTarget(MOTOR);
  Serial << "position_target: " << position_target << "\n";

  uint32_t position_actual = step_dir_controller.getPositionActual(MOTOR);
  Serial << "position_actual: " << position_actual << "\n";

  TMC429::Status status = step_dir_controller.getStatus();
  Serial << "status.at_target_position_0 = " << status.at_target_position_0 << "\n";
  Serial << "status.switch_left_0 = " << status.switch_left_0 << "\n";
  Serial << "status.at_target_position_1 = " << status.at_target_position_1 << "\n";
  Serial << "status.switch_left_1 = " << status.switch_left_1 << "\n";
  Serial << "status.at_target_position_2 = " << status.at_target_position_2 << "\n";
  Serial << "status.switch_left_2 = " << status.switch_left_2 << "\n";

  step_dir_controller.disableLeftSwitchStop(MOTOR);
  step_dir_controller.disableRightSwitchStop(MOTOR);
  step_dir_controller.disableSoftStop(MOTOR);
  step_dir_controller.setReferenceSwitchToRight(MOTOR);
  TMC429::ReferenceConfiguration ref_conf = step_dir_controller.getReferenceConfiguration(MOTOR);
  Serial << "ref_conf.disable_stop_l: " << ref_conf.disable_stop_l << "\n";
  Serial << "ref_conf.disable_stop_r: " << ref_conf.disable_stop_r << "\n";
  Serial << "ref_conf.soft_stop: " << ref_conf.soft_stop << "\n";
  Serial << "ref_conf.ref_rnl: " << ref_conf.ref_rnl << "\n";

  bool position_latched = step_dir_controller.positionLatched(MOTOR);
  Serial << "position_latched: " << position_latched << "\n";

  step_dir_controller.setPositionCompareMotor(MOTOR);
  step_dir_controller.enableRightReferences();
  TMC429::InterfaceConfiguration if_conf = step_dir_controller.getInterfaceConfiguration();
  Serial << "if_conf.inv_ref: " << if_conf.inv_ref << "\n";
  Serial << "if_conf.sdo_int: " << if_conf.sdo_int << "\n";
  Serial << "if_conf.step_half: " << if_conf.step_half << "\n";
  Serial << "if_conf.inv_stp: " << if_conf.inv_stp << "\n";
  Serial << "if_conf.inv_dir: " << if_conf.inv_dir << "\n";
  Serial << "if_conf.en_sd: " << if_conf.en_sd << "\n";
  Serial << "if_conf.pos_comp_sel: " << if_conf.pos_comp_sel << "\n";
  Serial << "if_conf.en_refr: " << if_conf.en_refr << "\n";

  TMC429::SwitchState switch_state = step_dir_controller.getSwitchState();
  Serial << "switch_state.r0: " << switch_state.r0 << "\n";
  Serial << "switch_state.l0: " << switch_state.l0 << "\n";
  Serial << "switch_state.r1: " << switch_state.r1 << "\n";
  Serial << "switch_state.l1: " << switch_state.l1 << "\n";
  Serial << "switch_state.r2: " << switch_state.r2 << "\n";
  Serial << "switch_state.l2: " << switch_state.l2 << "\n";

  TMC429::ClockConfiguration clk_config = step_dir_controller.getClockConfiguration(MOTOR);
  Serial << "clk_config.usrs: " << clk_config.usrs << "\n";
  Serial << "clk_config.ramp_div: " << clk_config.ramp_div << "\n";
  Serial << "clk_config.pulse_div: " << clk_config.pulse_div << "\n";

  double step_time = step_dir_controller.getStepTimeInMicroS();
  Serial << "step_time: " << step_time << "\n";

  Serial << "\n";
  delay(LOOP_DELAY);
}
