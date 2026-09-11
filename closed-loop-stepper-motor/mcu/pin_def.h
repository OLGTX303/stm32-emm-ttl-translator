#ifndef PIN_DEF_H
#define PIN_DEF_H

// SPI
#define MT6816_MISO 20
#define MT6816_CS   21
#define MT6816_SCK  22
#define MT6816_MOSI 23

#define SPI_PORT spi0

// UART
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define UART_485_EN 11
#define UART_ID     uart1

// A1 A2在同一个PWM slice
#define PWM_A1      17
#define PWM_A2      16
// B1 B2在同一个PWM slice
#define PWM_B1      25
#define PWM_B2      24

// 
#define LED1        0
#define LED2        1

// ADC0 - ADC2
#define GPIO_IOUT_A   27
#define GPIO_IOUT_B   26
#define GPIO_VBUS_SNS 28


#endif