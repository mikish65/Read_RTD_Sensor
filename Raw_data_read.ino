#define CS_PIN 8 // PB0 (Chip Select for ADC)

void setup() {
  // Set CS pin as output
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // ADC inactive

  // Set up Serial (optional, for debugging in Proteus via PE0/PD1)
  Serial.begin(9600);

  // Configure SPI: Enable SPI, Master mode, Clock = f_CPU / 16
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
  SPSR &= ~(1 << SPI2X); // No double speed
}

void loop() {
  uint16_t adc_raw = readADC();

  // Print raw value to Serial (e.g., for Proteus virtual terminal)
  Serial.print("Raw ADC Value: ");
  Serial.println(adc_raw);

  delay(1000); // Wait before next read
}

uint16_t readADC() {
  uint8_t high_byte, low_byte;
  uint16_t result;

  // Begin SPI communication
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5); // Brief delay if needed

  // Read MSB
  SPDR = 0x00;
  while (!(SPSR & (1 << SPIF)));
  high_byte = SPDR;

  // Read LSB
  SPDR = 0x00;
  while (!(SPSR & (1 << SPIF)));
  low_byte = SPDR;

  digitalWrite(CS_PIN, HIGH); // End communication

  // Combine 12-bit result (AD7352 sends left-justified 16-bit)
  result = ((high_byte << 8) | low_byte) >> 4;

  return result;
}
