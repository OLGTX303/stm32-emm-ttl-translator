#include "board.h"
#include "usart.h"
#include "translator.h"

/* DWT registers are absent from this older CMSIS header. ARMv7-M addresses. */
#define CYCLE_CTRL (*(volatile uint32_t *)0xE0001000UL)
#define CYCLE_COUNT (*(volatile uint32_t *)0xE0001004UL)

static uint32_t micros(void)
{
    static uint32_t previous, accumulated, remainder;
    uint32_t cycles=CYCLE_COUNT, delta=cycles-previous;
    previous=cycles;
    /* Main loop must run at least once per CYCCNT rollover (~59 seconds). */
    accumulated+=delta/72U;
    remainder+=delta%72U;
    accumulated+=remainder/72U; remainder%=72U;
    return accumulated;
}
int main(void)
{
    int b;
    SystemCoreClockUpdate();
    /* Standard supplied board clock: 8 MHz HSE * 9 = 72 MHz.
     * Do not operate motors if the oscillator failed and HSI is in use. */
    if(SystemCoreClock!=72000000UL || (RCC->CFGR&RCC_CFGR_SWS)!=RCC_CFGR_SWS_PLL)
        while(1) { }
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    CYCLE_COUNT=0; CYCLE_CTRL|=1U;
    board_init();
    tr_init(micros());
    while(1) {
        if(uart_take_error()) tr_uart_fault();
        /* Drain motors first so an available reply cannot time out behind host work. */
        while((b=uart_motor_get())>=0) tr_motor_byte((uint8_t)b,micros());
        while((b=uart_host_get())>=0) tr_host_byte((uint8_t)b,micros());
        tr_poll(micros());
    }
}
