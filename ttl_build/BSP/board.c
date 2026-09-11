#include "board.h"
#include "usart.h"
#include "motor_config.h"

void clock_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|
                          RCC_APB2Periph_AFIO|RCC_APB2Periph_USART1,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
}
void nvic_init(void)
{
    NVIC_InitTypeDef n;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    n.NVIC_IRQChannelCmd=ENABLE; n.NVIC_IRQChannelSubPriority=0;
    n.NVIC_IRQChannelPreemptionPriority=0; n.NVIC_IRQChannel=USART3_IRQn;
    NVIC_Init(&n);
    n.NVIC_IRQChannelPreemptionPriority=1; n.NVIC_IRQChannel=USART1_IRQn;
    NVIC_Init(&n);
}
void usart_init(void)
{
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;
    uart_init();
    g.GPIO_Pin=GPIO_Pin_9; g.GPIO_Speed=GPIO_Speed_50MHz; g.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA,&g);
    g.GPIO_Pin=GPIO_Pin_10; g.GPIO_Mode=GPIO_Mode_IPU; GPIO_Init(GPIOA,&g);
    g.GPIO_Pin=GPIO_Pin_10; g.GPIO_Mode=GPIO_Mode_AF_PP; GPIO_Init(GPIOB,&g);
    g.GPIO_Pin=GPIO_Pin_11; g.GPIO_Mode=GPIO_Mode_IPU; GPIO_Init(GPIOB,&g);
    USART_StructInit(&u);
    u.USART_BaudRate=MOTOR_DEFAULT_BAUD;
    USART_Init(USART1,&u);
    u.USART_BaudRate=HOST_BAUD;
    USART_Init(USART3,&u);
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART1,ENABLE); USART_Cmd(USART3,ENABLE);
}
void board_init(void)
{
    clock_init(); usart_init(); nvic_init();
}
