
#include <avr/io.h>
#include <util/delay.h>

// Constants
#define CS_PIN    PB0     // Chip Select for ADC (connected to PB0)
#define SCLK_PIN  PB1     // SPI Clock pin
#define MUX_S0    PC0     // MUX select bit 0
#define MUX_S1    PC1     // MUX select bit 1

// Function to initialize SPI
void SPI_init() {
    // Setting pins as output
    DDRB |= (1 << CS_PIN) | (1 << SCLK_PIN) | (1 << PB2);  
    PORTB |= (1 << CS_PIN);    
    
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR &= ~(1 << SPI2X);  // No double speed
}

// Function to read 12-bit ADC value over SPI
uint16_t read_ADC() {
    uint8_t high_byte, low_byte;
    uint16_t adc_value;

    PORTB &= ~(1 << CS_PIN); // Activating ADC
    _delay_us(5);  
   
    SPDR = 0x00;  // Send dummy byte
    while (!(SPSR & (1 << SPIF)));
    high_byte = SPDR;
   
    SPDR = 0x00;  // Send another dummy byte 
    while (!(SPSR & (1 << SPIF)));
    low_byte = SPDR;
   
    PORTB |= (1 << CS_PIN);  // Deactivating ADC

    // Convert to 12-bit value
    adc_value = ((high_byte << 8) | low_byte) >> 4;

    return adc_value;
}

// Function to select a channel on the MUX (0 to 3)
void select_mux_channel(uint8_t channel) {
    DDRC |= (1 << MUX_S0) | (1 << MUX_S1);  // Set PC0, PC1 as outputs

    PORTC &= ~((1 << MUX_S0) | (1 << MUX_S1));

    if (channel & 0x01) PORTC |= (1 << MUX_S0);
    if (channel & 0x02) PORTC |= (1 << MUX_S1);
}

// Main function
int main(void) {
    SPI_init();                 // Initialize SPI
    select_mux_channel(0);     // Initially select channel 0 (e.g., PT100)

    while (1) {
        uint16_t adc_result = read_ADC();  // Read raw 12-bit data from ADC
        _delay_ms(1000);
    }
}
