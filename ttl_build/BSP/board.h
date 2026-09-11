#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f10x.h"

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

void nvic_init(void);
void clock_init(void);
void usart_init(void);
void board_init(void);

#endif
