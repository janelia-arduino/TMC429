#include <Arduino.h>
#include <SPI.h>
#include <Streaming.h>
#include <TMC429.h>

const long BAUD = 115200;
const int LOOP_DELAY = 20;
const int CLOCK_FREQUENCY_MHZ = 32;
const int CLOCK_PIN = 5;
const int CHIP_SELECT_PIN = 2;
const int SCLK_PIN = 18;
const int MOSI_PIN = 23;
const int MISO_PIN = 19;
const int SPI_BUS = VSPI;

class MyTMC429 : public TMC429 {
  private:
    SPIClass* spi = NULL;

    void spiBegin() override {
      spi = new SPIClass(SPI_BUS);
      spi->begin(SCLK_PIN, MISO_PIN, MOSI_PIN);
    }

    void spiBeginTransaction(SPISettings settings) {
      spi->beginTransaction(settings);
    }

    void spiEndTransaction() {
      spi->endTransaction();
    }

    uint8_t spiTransfer(uint8_t byte) {
      return spi->transfer(byte);
    }
};

// Instantiate TMC429
MyTMC429 tmc429;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  // tmc429 clock
  pinMode(CLOCK_PIN, OUTPUT);
  ledcSetup(0, CLOCK_FREQUENCY_MHZ * 1000000, 1);
  ledcAttachPin(CLOCK_PIN, 0);
  ledcWrite(0, 1);

  tmc429.setup(CHIP_SELECT_PIN, CLOCK_FREQUENCY_MHZ);
  tmc429.initialize();
}

void loop()
{
  uint32_t chipId = 0;
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId);
  Serial.printf("Motor clk freq: %f\n", ledcReadFreq(0));
  Serial.printf("CPU freq: %u Mhz\n", getCpuFrequencyMhz());

  bool communicating = tmc429.communicating();
  Serial << "communicating: " << communicating << "\n\n";
  Serial << "version: " << tmc429.getVersion() << "\n\n";
  Serial << "\n";
  delay(LOOP_DELAY);
}
