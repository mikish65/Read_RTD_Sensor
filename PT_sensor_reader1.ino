// Final code. Does the following operations: 
//     SPI initialization -- SPI_init()
//     Read ADC reaw digital data -- read_ADC()
//     Select Multiplexer channel -- select_mux_channel()
//     Convert raw data to temperature reading -- calculate_temperature()     

// Headers
#include <avr/io.h> // Access ATMega register definitions for its pins e.g Pc0 
#include <util/delay.h> // For enabling delay functions for delaying time
#include <stdio.h> // For formatting strings sent via UART

// Constants
#define CS_PIN    PB0     
#define SCLK_PIN  PB1     
#define MOSI_PIN  PB2     
#define MUX_S0    PC0     
#define MUX_S1    PC1     

// Functions 
void SPI_init();
uint16_t read_ADC();
void select_mux_channel(uint8_t channel);
float calculate_temperature(uint16_t adc_raw, uint16_t r_ref, uint16_t r0);

// UART init and send functions
// UART is initailized solely for the purpose of interfacing the MCU with the virtual terminal in the simulation or practically with a computer
// The communication between the MCU and ADC is left to the SPI protocol
void UART_init(uint16_t baud);
void UART_send_string(const char *str);

void SPI_init() {
    DDRB |= (1 << CS_PIN) | (1 << SCLK_PIN) | (1 << MOSI_PIN); 
    PORTB |= (1 << CS_PIN);  // Deselect ADC
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR &= ~(1 << SPI2X);
}

uint16_t read_ADC() {
    uint8_t high_byte, low_byte;  
    uint16_t adc_value;

    PORTB &= ~(1 << CS_PIN);      // Activate CS
    _delay_us(5);

    SPDR = 0x00;
    while (!(SPSR & (1 << SPIF)));
    high_byte = SPDR;

    SPDR = 0x00;
    while (!(SPSR & (1 << SPIF)));
    low_byte = SPDR;

    PORTB |= (1 << CS_PIN);       // Deactivating CS

    adc_value = ((high_byte << 8) | low_byte) >> 4;
    return adc_value;
}

void select_mux_channel(uint8_t channel) {
    DDRC |= (1 << MUX_S0) | (1 << MUX_S1);
    PORTC &= ~((1 << MUX_S0) | (1 << MUX_S1));
    if (channel & 0x01) PORTC |= (1 << MUX_S0);
    if (channel & 0x02) PORTC |= (1 << MUX_S1);
}

float calculate_temperature(uint16_t adc_raw, uint16_t r_ref, uint16_t r0) {
    float v_ref = 5.0; //Reference voltage set at 5V
    float v_adc = (adc_raw / 4095.0f) * v_ref; // Convert raw digital ADV data value (adv_raw) to voltage. ADC is 12bit, so values range 0-4095
    float resistance = (v_adc * r_ref) / (v_ref - v_adc); //volatge divider formula to give resistance value of RTD 
    float temp = (resistance - r0) / 0.00385f; //Callender-Van Dusen eqtn, temp coeefficeint, alpha = 0.00385; r0 = 100/ r0=1000;
    return temp;
}

void UART_init(uint16_t baud) {
    uint16_t ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);             // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data
}

void UART_send_string(const char *str) {
    while (*str) {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *str++;
    }
}

int main(void) {
    char buffer[64];
    SPI_init();
    UART_init(9600);

    while (1) { 
        // Read PT100
        select_mux_channel(0);
        _delay_ms(10);
        uint16_t adc1 = read_ADC();
        float temp_pt100 = calculate_temperature(adc1, 100, 100);
        snprintf(buffer, sizeof(buffer), "PT100 Raw: %u | Temp: %.2f °C\r\n", adc1, temp_pt100);
        UART_send_string(buffer);

        // Read PT1000
        select_mux_channel(1);
        _delay_ms(10);
        uint16_t adc2 = read_ADC();
        float temp_pt1000 = calculate_temperature(adc2, 1000, 1000);
        snprintf(buffer, sizeof(buffer), "PT1000 Raw: %u | Temp: %.2f °C\r\n", adc2, temp_pt1000);
        UART_send_string(buffer);

        _delay_ms(1000);
    }
}
