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

  uint16_t velocity_min = 1000;
  step_dir_controller.setVelocityMin(0,velocity_min);
  Serial << "set velocity_min: " << velocity_min << "\n";

  velocity_min = step_dir_controller.getVelocityMin(0);
  Serial << "velocity_min: " << velocity_min << "\n";

  uint16_t velocity_max = 25000;
  step_dir_controller.setVelocityMax(0,velocity_max);
  Serial << "set velocity_max: " << velocity_max << "\n";

  velocity_max = step_dir_controller.getVelocityMax(0);
  Serial << "velocity_max: " << velocity_max << "\n";

  uint32_t position_target = 12345;
  step_dir_controller.setPositionTarget(0,position_target);
  Serial << "set position_target: " << position_target << "\n";

  position_target = step_dir_controller.getPositionTarget(0);
  Serial << "position_target: " << position_target << "\n";

  uint32_t position_actual = step_dir_controller.getPositionActual(0);
  Serial << "position_actual: " << position_actual << "\n";

  delay(LOOP_DELAY);
}
