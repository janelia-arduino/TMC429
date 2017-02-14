#include "Arduino.h"
#include "SPI.h"
#include "Streaming.h"
#include "TMC429.h"

const int BAUDRATE = 115200;
const int LOOP_DELAY = 2000;
const int CS_PIN = 10;

// Instantiate TMC429
TMC429 step_dir_controller = TMC429(CS_PIN);

void setup()
{
  // Setup serial communications
  Serial.begin(BAUDRATE);

  step_dir_controller.setup();
}

void loop()
{
  uint32_t type_version = step_dir_controller.getTypeVersion();
  Serial << "type_version: " << _HEX(type_version) << "\n";

  TMC429::Status status = step_dir_controller.getStatus();
  Serial << "status.at_target_position_0 = " << status.at_target_position_0 << "\n";
  Serial << "status.switch_left_0 = " << status.switch_left_0 << "\n";
  Serial << "status.at_target_position_1 = " << status.at_target_position_1 << "\n";
  Serial << "status.switch_left_1 = " << status.switch_left_1 << "\n";
  Serial << "status.at_target_position_2 = " << status.at_target_position_2 << "\n";
  Serial << "status.switch_left_2 = " << status.switch_left_2 << "\n\n";

  uint16_t velocity_min = 100;
  step_dir_controller.setVelocityMin(0,velocity_min);
  Serial << "set velocity_min: " << velocity_min << "\n";

  velocity_min = step_dir_controller.getVelocityMin(0);
  Serial << "velocity_min: " << velocity_min << "\n";

  uint16_t velocity_max = 2000;
  step_dir_controller.setVelocityMax(0,velocity_max);
  Serial << "set velocity_max: " << velocity_max << "\n";

  velocity_max = step_dir_controller.getVelocityMax(0);
  Serial << "velocity_max: " << velocity_max << "\n";

  step_dir_controller.setMode(0,TMC429::VELOCITY_MODE);

  TMC429::Mode mode = step_dir_controller.getMode(0);
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

  int16_t velocity_target = -321;
  step_dir_controller.setVelocityTarget(0,velocity_target);
  Serial << "set velocity_target: " << velocity_target << "\n";

  velocity_target = step_dir_controller.getVelocityTarget(0);
  Serial << "velocity_target: " << velocity_target << "\n";

  int16_t velocity_actual = step_dir_controller.getVelocityActual(0);
  Serial << "velocity_actual: " << velocity_actual << "\n";

  uint32_t position_target = 12345;
  step_dir_controller.setPositionTarget(0,position_target);
  Serial << "set position_target: " << position_target << "\n";

  position_target = step_dir_controller.getPositionTarget(0);
  Serial << "position_target: " << position_target << "\n";

  uint32_t position_actual = step_dir_controller.getPositionActual(0);
  Serial << "position_actual: " << position_actual << "\n";

  step_dir_controller.disableLeftSwitchStop(0);
  step_dir_controller.disableRightSwitchStop(0);
  step_dir_controller.disableSoftStop(0);
  step_dir_controller.setReferenceSwitchToRight(0);
  TMC429::ReferenceConfiguration ref_conf = step_dir_controller.getReferenceConfiguration(0);
  Serial << "ref_conf.disable_stop_l: " << ref_conf.disable_stop_l << "\n";
  Serial << "ref_conf.disable_stop_r: " << ref_conf.disable_stop_r << "\n";
  Serial << "ref_conf.soft_stop: " << ref_conf.soft_stop << "\n";
  Serial << "ref_conf.ref_rnl: " << ref_conf.ref_rnl << "\n";

  bool position_latched = step_dir_controller.positionLatched(0);
  Serial << "position_latched: " << position_latched << "\n";

  delay(LOOP_DELAY);
}
