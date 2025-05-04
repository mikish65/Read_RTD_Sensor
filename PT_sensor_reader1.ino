#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// Constants
#define CS_PIN    PB0     
#define SCLK_PIN  PB1     
#define MOSI_PIN  PB2     
#define MUX_S0    PC0     
#define MUX_S1    PC1     

// Functions used
void SPI_init();
uint16_t read_ADC();
void select_mux_channel(uint8_t channel);
float calculate_temperature(uint16_t adc_raw, uint16_t r_ref, uint16_t r0);

// UART init and send functions
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
    float v_ref = 5.0;
    float v_adc = (adc_raw / 4095.0f) * v_ref;
    float resistance = (v_adc * r_ref) / (v_ref - v_adc);
    float temp = (resistance - r0) / 0.00385f;
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
