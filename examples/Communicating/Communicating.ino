#include <TMC429.h>


const long BAUD = 115200;
const int LOOP_DELAY = 2000;
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;

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
  delay(LOOP_DELAY);
}
