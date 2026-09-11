#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"

static bool stop_requested(void)
{
	if(rxFrameFlag)
	{
		rxFrameFlag = false;
		if(rxCount == 4 && rxCmd[0] == 'S' && rxCmd[1] == 'T' && rxCmd[2] == 'O' && rxCmd[3] == 'P')
		{
			Emm_V5_Stop_Now(1, false);
			usart_SendText("RX STOP\r\n");
			return true;
		}
	}
	return false;
}

static bool wait_or_stop(uint32_t milliseconds)
{
	while(milliseconds >= 10)
	{
		if(stop_requested()) { return true; }
		delay_ms(10);
		milliseconds -= 10;
	}
	return stop_requested();
}

int main(void)
{
	const uint8_t motor = 1;

	board_init();
	fifo_initQueue();
	delay_ms(2000);
	usart_SendText("AUTO READY\r\n");

	while(1)
	{
		usart_SendText("AUTO START\r\n");
		Emm_V5_En_Control(motor, true, false);
		usart_SendText("ENABLE OK\r\n");
		delay_ms(100);
		Emm_V5_Modify_Ctrl_Mode(motor, false, 2);
		usart_SendText("MODE CLOSED\r\n");
		delay_ms(100);
		Emm_V5_Read_Sys_Params(motor, S_VBUS);
		Emm_V5_Read_Sys_Params(motor, S_VEL);
		Emm_V5_Read_Sys_Params(motor, S_CPOS);

		Emm_V5_Vel_Control(motor, 0, 50, 0, false);
		usart_SendText("VELOCITY 50RPM\r\n");
		if(wait_or_stop(1000)) { usart_SendText("STOPPED\r\n"); }
		Emm_V5_Stop_Now(motor, false);
		usart_SendText("VELOCITY STOP\r\n");
		delay_ms(200);

		Emm_V5_Pos_Control(motor, 0, 100, 0, 800, false, false);
		usart_SendText("POSITION 800PULSES\r\n");
		if(wait_or_stop(2000)) { usart_SendText("STOPPED\r\n"); }
		Emm_V5_Stop_Now(motor, false);
		usart_SendText("POSITION STOP\r\n");
		delay_ms(200);

		Emm_V5_En_Control(motor, false, false);
		usart_SendText("DISABLE OK\r\nAUTO DONE\r\n");
		if(wait_or_stop(1000)) { usart_SendText("STOPPED\r\n"); }
	}
}
