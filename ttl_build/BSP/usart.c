#include "stm32f10x.h"
#include "usart.h"
#include "translator.h"
#include <string.h>

#define RING_SIZE 512U
#define RING_MASK (RING_SIZE-1U)
typedef struct {
    uint8_t data[RING_SIZE];
    volatile uint16_t head, tail;
} Ring;
static Ring host_rx, host_tx, motor_rx, motor_tx;
static volatile bool uart_error;

static void receive(Ring *r, uint8_t b)
{
    uint16_t h=r->head;
    if((uint16_t)(h-r->tail)==RING_SIZE) { uart_error=true; return; }
    r->data[h&RING_MASK]=b;
    __DMB();
    r->head=(uint16_t)(h+1);
}
static int get(Ring *r)
{
    uint16_t t=r->tail;
    int b;
    if(t==r->head) return -1;
    b=r->data[t&RING_MASK];
    __DMB(); r->tail=(uint16_t)(t+1);
    return b;
}
static bool write(Ring *r, USART_TypeDef *uart, const uint8_t *p, uint16_t n)
{
    uint16_t h=r->head, i;
    if(n>RING_SIZE-(uint16_t)(h-r->tail)) return false;
    for(i=0;i<n;i++) r->data[(h+i)&RING_MASK]=p[i];
    __DMB(); r->head=(uint16_t)(h+n);
    uart->CR1|=USART_CR1_TXEIE;
    return true;
}
static void irq(USART_TypeDef *uart, Ring *rx, Ring *tx)
{
    uint32_t sr=uart->SR;
    if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)) {
        uint8_t b=(uint8_t)uart->DR; /* SR then DR clears RXNE/errors. */
        if(sr&(USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)) uart_error=true;
        else if(sr&USART_SR_RXNE) receive(rx,b);
    }
    if((sr&USART_SR_TXE) && (uart->CR1&USART_CR1_TXEIE)) {
        if(tx->tail==tx->head) uart->CR1&=~USART_CR1_TXEIE;
        else {
            uart->DR=tx->data[tx->tail&RING_MASK];
            tx->tail=(uint16_t)(tx->tail+1);
        }
    }
}
void USART1_IRQHandler(void) { irq(USART1,&motor_rx,&motor_tx); }
void USART3_IRQHandler(void) { irq(USART3,&host_rx,&host_tx); }
int uart_host_get(void) { return get(&host_rx); }
int uart_motor_get(void) { return get(&motor_rx); }
bool uart_take_error(void)
{
    bool error;
    __disable_irq(); error=uart_error; uart_error=false;
    if(error) { host_rx.tail=host_rx.head; motor_rx.tail=motor_rx.head; }
    __enable_irq(); /* Only called by main with interrupts enabled. */
    return error;
}
bool tr_bus_write(const uint8_t *p, uint16_t n) { return write(&motor_tx,USART1,p,n); }
bool tr_bus_idle(void) { return motor_tx.head==motor_tx.tail && (USART1->SR&USART_SR_TC)!=0; }
bool tr_host_write(const uint8_t *p, uint16_t n)
{
    return write(&host_tx,USART3,p,n);
}
void uart_init(void)
{
    memset(&host_rx,0,sizeof(host_rx)); memset(&host_tx,0,sizeof(host_tx));
    memset(&motor_rx,0,sizeof(motor_rx)); memset(&motor_tx,0,sizeof(motor_tx));
    uart_error=false;
}
