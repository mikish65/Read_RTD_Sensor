
#include <avr/io.h>
#include <util/delay.h>

// Definitions
#define CS_PIN   PB0       // Chip Select connected to Port B, Pin 0
#define MUX_S0   PC0       // MUX Select Line 0
#define MUX_S1   PC1       // MUX Select Line 1

void SPI_init() {
    // Set MOSI (PB2), SCK (PB1), CS (PB0) as output
    DDRB |= (1 << PB2) | (1 << PB1) | (1 << CS_PIN);

    // Set CS high (ADC inactive)
    PORTB |= (1 << CS_PIN);

    // Enable SPI peripheral and configure pins
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); //Enables SPI module, MSTR sets Master mode, SPR0 sets clock speed to 16
    SPSR &= ~(1 << SPI2X); // No double speed
}

uint16_t read_ADC() {
    uint8_t high_byte, low_byte; // Declaring variables to hold bytes received from ADC
    uint16_t adc_value;

    // Activate ADC (CS LOW)
    PORTB &= ~(1 << CS_PIN); // Activates CS and ADC starts shifting data
    _delay_us(5);  // Small delay for ADC setup

   
    SPDR = 0x00; // Transmit dummy byte 
  // Read the ADC data through SPI
    while (!(SPSR & (1 << SPIF)));  
    high_byte = SPDR;

    
    SPDR = 0x00;// Transmit another dummy byte 
    while (!(SPSR & (1 << SPIF)));
    low_byte = SPDR;

    
    PORTB |= (1 << CS_PIN); // Deactivate ADC 

    // Combine to 12-bit result (AD7352 gives left-aligned 16-bit output)
    adc_value = ((high_byte << 8) | low_byte) >> 4;

    return adc_value;
}

void select_mux_channel(uint8_t channel) {
    // Configure MUX S0 (PC0), S1 (PC1) as output
    DDRC |= (1 << MUX_S0) | (1 << MUX_S1);

    // Clear both bits first
    PORTC &= ~((1 << MUX_S0) | (1 << MUX_S1));

    if (channel & 0x01) PORTC |= (1 << MUX_S0); // Bit 0
    if (channel & 0x02) PORTC |= (1 << MUX_S1); // Bit 1
}

int main(void) {
    SPI_init();
    select_mux_channel(0); // Start with channel 0 (e.g., PT100)

    while (1) {
        uint16_t adc_result = read_ADC();

        // You can send adc_result over UART here if needed
        _delay_ms(1000);  // Delay before next read
    }
}
