/* Bench-only raw UART bridge. Never use as the translator image. */
#include "board.h"
#include "usart.h"
#include "translator.h"
int main(void)
{
    int b;
    uint8_t byte;
    SystemCoreClockUpdate();
    board_init();
    for(;;) {
        while((b=uart_host_get())>=0) {
            byte=(uint8_t)b;
            while(!tr_bus_write(&byte,1)) { }
        }
        while((b=uart_motor_get())>=0) {
            byte=(uint8_t)b;
            while(!tr_host_write(&byte,1)) { }
        }
    }
}
