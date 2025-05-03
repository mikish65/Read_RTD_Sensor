#define CS_PIN 8 // PB8==PB0; Chip Select pin on ADC conects to PB0 on MCU. 

void setup() {
  pinMode(CS_PIN, OUTPUT); //CS pin set as an output
  digitalWrite(CS_PIN, HIGH); //  CS pin made high to Deactivate

  // Enable SPI peripheral and configure pins
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); //SPE==Enables SPI module, MSTR sets Master mode, SPR0 sets clock speed to 16
  SPSR &= ~(1 << SPI2X); // No double speed
}

void loop() {
  uint8_t high_byte, low_byte; // Declaring variables to hold bytes received from ADC
  uint16_t adc_value;

  // Send a command to start ADC conversion
  digitalWrite(CS_PIN, LOW); // Activates CS and ADC starts shifting data

  // Wait for conversion to complete (e.g., short delay)
  delayMicroseconds(5);

  // Read the ADC data through SPI
  SPDR = 0x00; // Sending dummy bytes 
  while (!(SPSR & (1 << SPIF))); //
  high_byte = SPDR; //hold received byte

  SPDR = 0x00;
  while (!(SPSR & (1 << SPIF)));
  low_byte = SPDR;

 
  digitalWrite(CS_PIN, HIGH); // Deactivate chip select (CS)

  // Combine result to 12-bit
  adc_value = ((high_byte << 8) | low_byte) >> 4;
}
