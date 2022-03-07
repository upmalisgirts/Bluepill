#include <stdint.h>

#define USART_BASE 0x40013800                                                       // USART1_BASE register address
#define GPIOA_BASE 0x40010800                                                       // GPIO Port A register address
#define GPIOC_BASE 0x40011000
#define SPI1_BASE 0x40013000                                                        // SPI1_BASE register address
#define GPIOA_CRH (*(volatile unsigned int *)(GPIOA_BASE + 0x04))                   // Port configuration register high (GPIOA_CRH), (For PA9 and PA10 cfg.)
#define GPIOA_CRL (*(volatile unsigned int *)(GPIOA_BASE))                          // Port configuration register high (GPIOA_CRL), (For PA4-PA7 cfg.)
#define USART_CR1 (*(volatile unsigned int *)(USART_BASE + 0x0C))                   // USART1 Control Register 1
#define SPI1_CR1 (*(volatile unsigned int *)(SPI1_BASE))
#define SPI1_CR2 (*(volatile unsigned int *)(SPI1_BASE + 0x04))
#define SPI1_SR (*(volatile unsigned int *)(SPI1_BASE + 0x08))
#define SPI1_DR (*(volatile unsigned int *)(SPI1_BASE + 0x0C))
#define RCC_APB2ENR (*(volatile unsigned int *)(0x40021018))                        // APB2 peripheral clock enable register (RCC_APB2ENR)
#define USART_BRR (*(volatile unsigned int *)(USART_BASE + 0x08))                   // USART1 Baud Rate Register
#define USART_SR (*(volatile unsigned int *)(USART_BASE))                           // USART1 Status Register
#define USART_DR (*(volatile unsigned int *)(USART_BASE + 0x04))                    // USART1 Data Register

#define LED_ON (*(volatile unsigned short *)(0x40011014)) = (1 << 13) //GPIO13_BRR
#define LED_OFF (*(volatile unsigned int *)(0x40011010)) = (1 << 13) //GPIO13_BSRR

struct Compensation {
    unsigned short dig_T1;
    signed short dig_T2;
    signed short dig_T3;
    unsigned short dig_P1;
    signed short dig_P2;
    signed short dig_P3;
    signed short dig_P4;
    signed short dig_P5;
    signed short dig_P6;
    signed short dig_P7;
    signed short dig_P8;
    signed short dig_P9;
    unsigned char dig_H1;
    signed short dig_H2;
    unsigned char dig_H3;
    signed short dig_H4;
    signed short dig_H5;
    signed char dig_H6;
};
void sendChar(char);
void sendMsg(const char *);
void configureClocks();
void configureGpio();
void configureLed();
void configureUsart1();
void configureSpi1();
void selectSlave();
void unselectSlave();
void writeSpi(uint8_t address, uint8_t value);
uint8_t readSpi(uint8_t address);
void configureBme();
void measureBme();
uint8_t * readCalibration();
// uint8_t * tests();
void clearRxBuffer();
int calculateTFine(int adc_T, struct Compensation BME);
int calculateTemperature(int adc_T, struct Compensation BME);
unsigned int calculatePressure(int adc_P, int temp_fine, struct Compensation BME);
unsigned int calculateHumidity(int adc_H, int temp_fine, struct Compensation BME);
int numberLength(int number);
const char * intToCharArray(int number);
int main(void) {
    configureClocks();
    configureGpio();
    configureUsart1();
    configureSpi1();
    unselectSlave();
   
    // (*(volatile unsigned int *)(0x40011004)) &= ~(0x0F << (((14 - 8) * 4) -1)); //GPIO14_CRH RESET
    // (*(volatile unsigned int *)(0x40011004)) |= (0x02 << (((14 - 8) * 4) + 2)); //GPIO14_CRH CNF
    // (*(volatile unsigned int *)(0x40011004)) |= (0x00 << ((14 - 8) * 4)); //GPIO14_CRH MODE - SET INPUT
    // (*(volatile unsigned int *)(0x40011010)) = (1 << 13); //GPIO13_BSRR
    // int light = 0;

    struct Compensation BME;

    uint8_t h_lsb, h_msb, t_lsb, t_msb, t_xlsb, p_lsb, p_msb, p_xlsb;
    int adc_H, adc_T, adc_P, temperature, humidity, pressure;
    
    configureBme();
    uint8_t * calibration;
    calibration = readCalibration();
    BME.dig_T1 = (*(calibration+1) << 8 | *(calibration));
    BME.dig_T2 = (*(calibration+3) << 8 | *(calibration+2));
    BME.dig_T3 = (*(calibration+5) << 8 | *(calibration+4));
    BME.dig_P1 = (*(calibration+7) << 8 | *(calibration+6));
    BME.dig_P2 = (*(calibration+9) << 8 | *(calibration+8));
    BME.dig_P3 = (*(calibration+11) << 8 | *(calibration+10));
    BME.dig_P4 = (*(calibration+13) << 8 | *(calibration+12));
    BME.dig_P5 = (*(calibration+15) << 8 | *(calibration+14));
    BME.dig_P6 = (*(calibration+17) << 8 | *(calibration+16));
    BME.dig_P7 = (*(calibration+19) << 8 | *(calibration+18));
    BME.dig_P8 = (*(calibration+21) << 8 | *(calibration+20));
    BME.dig_P9 = (*(calibration+23) << 8 | *(calibration+22));
    BME.dig_H1 = (*(calibration+24));
    BME.dig_H2 = (*(calibration+26) << 8 | *(calibration+25));
    BME.dig_H3 = (*(calibration+27));
    BME.dig_H4 = (*(calibration+28) << 4 | (*(calibration+29) & 0xF));
    BME.dig_H5 = (*(calibration+30) << 4 | *(calibration+29) >> 4);
    BME.dig_H6 = *(calibration+31);
    for (int i = 0; i < 800000; i++) {}
    while(1) {
        LED_ON;
        measureBme();
        h_msb = readSpi(0xFD);
        h_lsb = readSpi(0xFE);
        t_msb = readSpi(0xFA);
        t_lsb = readSpi(0xFB);
        t_xlsb = readSpi(0xFC);
        p_msb = readSpi(0xF7);
        p_lsb = readSpi(0xF8);
        p_xlsb = readSpi(0xF9);
        adc_H = (h_msb << 8 | h_lsb);                                           //16 bits
        adc_T = (t_msb << 12 | t_lsb << 4 | t_xlsb >> 4);                       //20 bits
        adc_P = (p_msb << 12 | p_lsb << 4 | p_xlsb >> 4);                       //20 bits
        temperature = calculateTemperature(adc_T, BME);
        pressure = calculatePressure(adc_P,calculateTFine(adc_T,BME),BME);
        humidity = calculateHumidity(adc_H,calculateTFine(adc_T, BME), BME);
        sendMsg("Temperature:");
        sendMsg(intToCharArray(temperature));
        sendMsg("Pressure:");
        sendMsg(intToCharArray(pressure));
        sendMsg("Humidity:");
        sendMsg(intToCharArray(humidity));
        LED_OFF;
        for (int i = 0; i < 40033361; i++) {}   //sleep ~60 seconds
    }
}

void sendChar(char c) {
    // Wait until Transmit Data Register is empty
    while (!(USART_SR & (1 << 7)));
    // Write to Data Register
    USART_DR = c;
}

void sendMsg(const char * msg) {
    int i = 0;
    while(msg[i] != '\0') {
        sendChar(msg[i]);
        i++;
    }
    sendChar('\n');
}

void configureClocks() {
    // Enable neccessary clocks
    RCC_APB2ENR |= (1 << 4);                                        // IOPCEN - I/O port C clock enable
    RCC_APB2ENR |= (1 << 2);                                        // IOPAEN - I/O port A clock enable
    RCC_APB2ENR |= (1 << 14);                                       // USART1EN - USART1 clock enable
    RCC_APB2ENR |= (1 << 12);                                       // SPI1EN - SPI1 clock enable
}

void configureGpio() {
    GPIOA_CRH |= (0x0BUL << 4);
    GPIOA_CRH |= (0x04UL << 8);
    // GPIOA_CRL &= ~(0x0F << 12);
    GPIOA_CRL &= ~(0x0F << 16);
    GPIOA_CRL |= (0x02UL << 16);                                    // SPI1 SS cfg.
    GPIOA_CRL &= ~(0x0F << 20);
    GPIOA_CRL |= (0x0BUL << 20);                                    // SPI1 SCK cfg.
    GPIOA_CRL |= (0x04UL << 24);                                    // SPI1 MISO cfg.
    GPIOA_CRL |= (0x0BUL << 28);                                    // SPI1 MOSI cfg.

    (*(volatile unsigned int *)(0x40011004)) &= ~(0x0F << (((13 - 8) * 4) -1)); //GPIO13_CRH RESET 
    (*(volatile unsigned int *)(0x40011004)) |= (0x00 << (((13 - 8) * 4) + 2)); //GPIO13_CRH CNF
    (*(volatile unsigned int *)(0x40011004)) |= (0x02 << ((13 - 8) * 4)); //GPIO13_CRH MODE - SET OUTPUT
}

void configureUsart1() {
    USART_CR1 |= (1 << 13);                                         // USART1 enable
    USART_CR1 |= (1 << 2);                                          // USART1 enable Rx
    USART_CR1 |= (1 << 3);                                          // USART1 enable Tx
    USART_BRR  = 8000000L/9600L;
}

void configureSpi1() {
    SPI1_CR1 = 0;
    SPI1_CR1 |= (0x7 << 3);
    SPI1_CR1 |= (1 << 2);                                           // SPI1 master select
    SPI1_CR2 |= (1 << 2);                                           // SPI1 SS enable
    SPI1_CR1 |= (1 << 6);                                           // SPI1 enable
}

void selectSlave() {
    (*(volatile unsigned int *)(0x40010814)) = (1 << 4);
}

void unselectSlave() {
    (*(volatile unsigned int *)(0x40010810)) = (1 << 4);
}

void writeSpi(uint8_t address, uint8_t value) {
    selectSlave();
    while ((SPI1_SR & (1 << 7)));
    SPI1_DR = address & 0x7F;
    while ((SPI1_SR & (1 << 7)));
    SPI1_DR = value;
    for (int i = 0; i < 200; ++i) __asm__("nop");
    unselectSlave();
    clearRxBuffer();
}

uint8_t readSpi(uint8_t address) {
    uint8_t read_val;
    selectSlave();
    while ((SPI1_SR & (1 << 7)));
    SPI1_DR = address;
    while (!(SPI1_SR & (1 << 0)));
    read_val = SPI1_DR;
    while ((SPI1_SR & (1 << 7)));
    SPI1_DR = 0x00;
    while (!(SPI1_SR & (1 << 0)));
    read_val = SPI1_DR;
    for (int i = 0; i < 200; ++i) __asm__("nop");
    unselectSlave();
    return read_val;
}

uint8_t * readCalibration() {
    static uint8_t values[32];
    int num = 0;
    for (uint8_t address = 0x88; address < 0xE8; address++) {
        values[num] = readSpi(address);
        num++;
        if (address == 0xA1) {
            address = 0xE0;
        }
    }
    return values;
}

void clearRxBuffer() {
    // uint8_t buf_val;
    while (!(SPI1_SR & (1 << 0)));
    // buf_val = SPI1_DR;
    SPI1_DR;
}

int calculateTFine(int adc_T, struct Compensation BME) {
    int var1, var2, temp_fine;
    var1 = ((((adc_T>>3) - ((int)BME.dig_T1<<1))) * ((int)BME.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int)BME.dig_T1)) * ((adc_T>>4) - ((int)BME.dig_T1))) >> 12) *((int)BME.dig_T3)) >> 14;
    temp_fine = var1 + var2;
    return temp_fine;
}

int calculateTemperature(int adc_T, struct Compensation BME) {
    int temp, temp_fine;
    temp_fine = calculateTFine(adc_T, BME);
    temp = (temp_fine * 5 + 128) >> 8;
    return temp;
}

unsigned int calculatePressure(int adc_P, int temp_fine, struct Compensation BME) {
    int var1, var2;
    unsigned int p; 
    var1 = (temp_fine >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (int)BME.dig_P6;
    var2 = var2 + ((var1 * (int)BME.dig_P5) << 1);
    var2 = (var2 >> 2) + ((int)BME.dig_P4 << 16);
    var1 = (((BME.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int)BME.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1))*((int)BME.dig_P1)) >> 15);
    if (var1 == 0) {
        return 0;
    }
    p = (((unsigned int)(((int)1048576) - adc_P) - (var2 >> 12)))*3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((unsigned int)var1);
    } else {
        p = (p/(unsigned int)var1) * 2;
    }
    var1 = (((int)BME.dig_P9) * ((int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int)(p >> 2)) * ((int)BME.dig_P8)) >> 13;
    p = (unsigned int)((int)p + ((var1 + var2 + BME.dig_P7) >> 4));
    return p;
}

unsigned int calculateHumidity(int adc_H, int temp_fine, struct Compensation BME) {
    int hum;
    hum = (temp_fine - (int)76800);
    hum = (((((adc_H << 14) - (((int)BME.dig_H4) << 20) - (((int)BME.dig_H5) * hum)) + ((int)16384)) >> 15) * (((((((hum * ((int)BME.dig_H6)) >> 10) * (((hum * ((int)BME.dig_H3)) >> 11) + ((int)32768))) >> 10) + ((int)2097152)) * ((int)BME.dig_H2) + 8192) >> 14));
    hum = (hum - (((((hum >> 15) * (hum >> 15)) >> 7) * ((int)BME.dig_H1)) >> 4));
    hum = (hum < 0 ? 0 : hum);
    hum = (hum > 419430400 ? 419430400 : hum);
    return (unsigned int)(hum >> 12);
}

void configureBme() {
    writeSpi(0xF2,0x01); //00000001
    writeSpi(0xF4,0x24); //00100100
}

void measureBme() {
    writeSpi(0xF4,0x25); //00100101 - switching BME mode to forced
    while (readSpi(0xF3) == 0x04); // wait until default value is cleared
    while (!(readSpi(0xF3) == 0x04)); // wait until the default value is set again
}

int numberLength(int number) {
    int val = 10;
    for (int i = 1; i < 10; i++) {
        if (val > number) {
            return i;
        }
        val = val * 10;
    }
    return 0;
}

const char * intToCharArray(int number) {
    int digitCount = numberLength(number);
    static char digitArray[10];
    digitArray[digitCount] = '\0';
    for (int i = 0; i < digitCount; i++) {
        digitArray[digitCount-i-1] = number%10+'0';
        number = number/10;
    }    
    return digitArray;
}