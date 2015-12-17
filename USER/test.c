#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "exti.h"
#include "wdg.h"
#include "timer.h"
#include "lcd.h"
#include "rtc.h"
#include "wkup.h"
#include "adc.h"
#include "modbus.h"
//Mini STM32开发板范例代码13
//ADC 实验
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
int main(void)
{
  Stm32_Clock_Init(9);//系统时钟设置
	delay_init(72);		//延时初始化
	uart_init(36,9600); //串口1初始化
	LED_Init();
	Adc_Init();
	MB_Init();

	while(1)
	{
		MB_Poll();
	}
}






















