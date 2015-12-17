#include "sys.h"
#include "usart.h"
#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//串口1初始化
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/5/27
//版本：V1.3
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
	int handle;
	/* Whatever you require here. If the only file you are using is */
	/* standard output using printf() for debugging, no file handling */
	/* is required. */
};
/* FILE is typedef’ d in stdio.h. */
FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
_sys_exit(int x)
{
	x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕
	USART3->DR = (u8) ch;
	return ch;
}
#endif
//end
//////////////////////////////////////////////////////////////////
#define EN_USART1_RX
#ifdef EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误

u8 uart_rev_buff[Rev_Buff_Max];     //接收缓冲
extern u16 uart_rev_count;	//ADU接收字节个数
extern u8 volatile uart_rev_flag;

void USART3_IRQHandler(void)
{
	u8 res;
	if(USART3->SR&(1<<5))//接收到数据
	{
		// res=USART1->DR;
		// USART1->DR=res;
        // while(!(USART1->SR&(1<<6)));//等待发送完成
		//if(uart_rev_flag == 1)	//等待一帧数据解析完毕
		{
			if (uart_rev_count<Rev_Buff_Max)
			{
				res=USART3->DR;
				uart_rev_buff[uart_rev_count]=res;
				uart_rev_count++;
				/*当接收的字节后超过3.5个字节时间没有新的字节认为本次接收完成*/
				//3.5个字符时间=3.5*8*1/9600秒=2.917ms
				//3.5个字符时间，判断一帧数据是否结束
				//Timerx_Init(6,7199);	//0.5ms
				Timerx_Init(20,7199);	//0.5ms
			}
			else
			{
				res=USART3->DR;	//如果数据帧长度超过缓存，则丢弃字符
			}
		}
		//else
			;

	}
}
#endif
void uart_send_char(u8 ch)
{
	USART3->DR=ch;
    while(!(USART3->SR&(1<<6)));//等待发送完成
		//USART1->SR &= ~(1<<6);
}
//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
//CHECK OK
//091209
//USART3
void uart_init(u32 pclk2,u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分
    mantissa<<=4;
	mantissa+=fraction;
	RCC->APB2ENR|=1<<3;   //使能PORTB口时钟
	RCC->APB1ENR|=1<<18;  //使能串口时钟
	GPIOB->CRH&=0XFFFF00FF;
	GPIOB->CRH|=0X00008B00;//IO状态设置

	RCC->APB1RSTR|=1<<18;   //复位串口3
	RCC->APB1RSTR&=~(1<<18);//停止复位
	//波特率设置
 	USART3->BRR=mantissa; // 波特率设置
	USART3->CR1|=0X200C;  //1位停止,无校验位.
#ifdef EN_USART1_RX		  //如果使能了接收
	//使能接收中断
	USART3->CR1|=1<<8;    //PE中断使能
	USART3->CR1|=1<<5;    //接收缓冲区非空中断使能
	MY_NVIC_Init(1,1,USART3_IRQChannel,2);//组2，最低优先级
#endif
}
