
#include <avr/io.h>
#include <util/delay.h>

// Constants
#define CS_PIN    PB0     // Chip Select for ADC (connected to PB0)
#define SCLK_PIN  PB1     // SPI Clock pin
#define MUX_S0    PC0     // MUX select bit 0
#define MUX_S1    PC1     // MUX select bit 1

// Function to initialize SPI
void SPI_init() {
    // Set PB0 (CS), PB1 (SCLK), PB2 (MOSI) as output
    DDRB |= (1 << CS_PIN) | (1 << SCLK_PIN) | (1 << PB2);  

    // Set CS high to deactivate ADC
    PORTB |= (1 << CS_PIN);  

    // Enable SPI, Master mode, fosc/16 clock
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR &= ~(1 << SPI2X);  // No double speed
}

// Function to read 12-bit ADC value over SPI
uint16_t read_ADC() {
    uint8_t high_byte, low_byte;
    uint16_t adc_value;

    // Activate ADC
    PORTB &= ~(1 << CS_PIN);
    _delay_us(5);  // Small delay before SPI read

    // Send dummy byte to receive first byte
    SPDR = 0x00;
    while (!(SPSR & (1 << SPIF)));
    high_byte = SPDR;

    // Send dummy byte to receive second byte
    SPDR = 0x00;
    while (!(SPSR & (1 << SPIF)));
    low_byte = SPDR;

    // Deactivate ADC
    PORTB |= (1 << CS_PIN);

    // Convert to 12-bit value
    adc_value = ((high_byte << 8) | low_byte) >> 4;

    return adc_value;
}

// Function to select a channel on the MUX (0 to 3)
void select_mux_channel(uint8_t channel) {
    DDRC |= (1 << MUX_S0) | (1 << MUX_S1);  // Set PC0, PC1 as outputs

    // Clear both bits
    PORTC &= ~((1 << MUX_S0) | (1 << MUX_S1));

    // Set bits based on channel number
    if (channel & 0x01) PORTC |= (1 << MUX_S0);
    if (channel & 0x02) PORTC |= (1 << MUX_S1);
}

// Main function
int main(void) {
    SPI_init();                 // Initialize SPI
    select_mux_channel(0);     // Initially select channel 0 (e.g., PT100)

    while (1) {
        uint16_t adc_result = read_ADC();  // Read raw 12-bit data from ADC

        // Use adc_result as needed (e.g., store, send, or process)
        _delay_ms(1000);
    }
}
