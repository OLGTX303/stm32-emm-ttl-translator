#ifndef TRANSLATOR_USART_H
#define TRANSLATOR_USART_H
#include <stdint.h>
#include <stdbool.h>
void uart_init(void);
int uart_host_get(void);
int uart_motor_get(void);
bool uart_take_error(void);
#endif
