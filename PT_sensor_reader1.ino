
#include <SPI.h>

const int MUX_S0 = 2;
const int MUX_S1 = 3;
const int CS_PIN = 8;

void setup() {
  Serial.begin(9600);
  //Setting pins as outputs
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

   // Enable SPI peripheral and configure pins
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); //SPE==Enables SPI module, MSTR sets Master mode, SPR0 sets clock speed to 16
  SPSR &= ~(1 << SPI2X); // No double speed

  SPI.begin();
  digitalWrite(CS_PIN, HIGH);
}

void selectMUXChannel(int channel) {
  digitalWrite(MUX_S0, channel & 0x01);
  digitalWrite(MUX_S1, (channel >> 1) & 0x01);
}

uint16_t readADC() {
  digitalWrite(CS_PIN, LOW); // Activates CS and ADC starts shifting data
  delayMicroseconds(1);
  uint16_t value = SPI.transfer16(0x0000);
  digitalWrite(CS_PIN, HIGH);
  return value & 0x0FFF;
}

void loop() {
  // Read PT100
  selectMUXChannel(0);
  delay(10);
  uint16_t adc1 = readADC();
  float voltage1 = (adc1 / 4095.0) * 5.0;
  float resistance1 = (voltage1 * 100.0) / (5.0 - voltage1);
  float temp_pt100 = (resistance1 - 100.0) / 0.00385;

  Serial.print("PT100 Raw: "); Serial.print(adc1);
  Serial.print(" | Temp: "); Serial.print(temp_pt100); Serial.println(" °C");

  // Read PT1000
  selectMUXChannel(1);
  delay(10);
  uint16_t adc2 = readADC();
  float voltage2 = (adc2 / 4095.0) * 5.0;
  float resistance2 = (voltage2 * 1000.0) / (5.0 - voltage2);
  float temp_pt1000 = (resistance2 - 1000.0) / 0.00385;

  Serial.print("PT1000 Raw: "); Serial.print(adc2);
  Serial.print(" | Temp: "); Serial.print(temp_pt1000); Serial.println(" °C");

  delay(1000);
}
