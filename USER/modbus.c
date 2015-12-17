#include "modbus.h"
#include "timer.h"
#include "usart.h"
#include "adc.h"

u8 volatile uart_rev_flag=0;	//接收完成标志
u8 slave_id=0x01;	//从机地址
u16 uart_rev_count=0;	//ADU接收字节个数
u16 uart_send_count=0;	//ADU发送数据个数
u8 uart_send_buff[MB_SEND_BUF_MAX]={0};	//modbus发送数据
extern u8 uart_rev_buff[Rev_Buff_Max];     //接收缓冲
u16 MB_Reg[MB_REG_MAX];	//modbus寄存器数组,电流读取值

void MB_Init()
{
	Timerx_Init(20,7199);	//0.5ms一个单位
	TIM3->CR1&=~(1<<0);
	
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟
	   	 
	GPIOB->CRH&=0XFFF0FFFF; 
	GPIOB->CRH|=0X00030000;//推挽输出 	 
  GPIOB->ODR&=~(1<<12);      //输出低
}

/*
 * info:CRC16校验
 * para:*ptr 校验数据 len 校验数据长度
*/
u16 getCRC16(volatile u8 *ptr,u8 len)
{
    u8 i;
    u16 crc = 0xFFFF;
    if(len==0)
    {
        len = 1;
    }
    while(len--)
    {
        crc ^= *ptr;
        for(i=0; i<8; i++)
    	{
            if(crc&1)
        	{
                crc >>= 1;
                crc ^= 0xA001;
        	}
        	else
    		{
                crc >>= 1;
        	}
        }
        ptr++;
    }
    return(crc);
}

/*
 * info : 清空接收缓存
*/
void flush_revBuff(void)
{
	u16 i;
	for (i = 0; i < Rev_Buff_Max; ++i)
	{
		uart_rev_buff[i]=0;
	}
}

void flush_sendBuff(void)
{
	u16 i;
	
	for (i = 0; i < MB_SEND_BUF_MAX; ++i)
	{
		uart_send_buff[i]=0;
	}
}

/*
 * info:协议栈解析
*/
void MB_Poll(void)
{
    if(uart_rev_flag == 1)//接收完成标志=1处理，否则退出
    {
        if(uart_rev_buff[0] == slave_id)//地址错误不应答
    	{
            u16 crc;
            u8 crc_det[2];
            crc = getCRC16(uart_rev_buff,uart_rev_count-2); 	//crc校验
            crc_det[0] = (u8)crc;  //低八位
            crc_det[1] = (u8)(crc >> 8); //高八位
            if((uart_rev_buff[uart_rev_count-1] == crc_det[1])&&(uart_rev_buff[uart_rev_count-2] == crc_det[0]))//crc校验错误不应答
        	{
                switch(uart_rev_buff[1])
            	{
                    case 0x03: //读多个寄存器
	            		MB_Read_MutiReg();
	                    break;
                    case 0x06: //写单个寄存器
                    	MB_Write_SingleReg();
                        break;
                    default:
                    	uart_send_char(0x03);	//请求命令错误
                }
    		}
    		else
    		{
    			//uart_send_char(0x02);	//crc校验错误
    		}
    	}
    	else
    	{
    		//uart_send_char(0x01);	//从机地址错误
    	}

        flush_revBuff();	//清空接收缓存
				uart_rev_count=0;
				uart_rev_flag = 0;
				//USART1->CR1|=1<<8;
				//USART1->CR1|=1<<5;
    }
}

/*
 * info : 从机响应数据帧发送
 * para : dat 数据起始地址  send_count 发送数据长度
*/
void MB_Reply(u8* dat,u16 send_count)
{
	u16 i;
	GPIOB->ODR|=1<<12;      //输出高
	for (i = 0; i < send_count; ++i)
	{
		uart_send_char(*(dat+i));
	}
	GPIOB->ODR&=~(1<<12);      //输出低
}

#define RATIO (3.3/(0.02*100))
void GetVonvertionValue(void)
{
	u8 i;
	for(i=0;i<MB_REG_MAX;++i)
	{
		MB_Reg[i] = Get_Adc(i);
		MB_Reg[i] = MB_Reg[i]*RATIO;
	}
}

/*
 * info : 读取多个寄存器，即16位变量
 * 数据帧规则-->地址、寄存器数量Big endian : 先高后低
 *           -->其他 Little dndian ：先低后高
*/
void MB_Read_MutiReg(void)
{
	u16 i,index_buf,crc_det;
	u16 regAddr=0;	//线圈起始地址
	u16 regReadCnt=0;	//读取寄存器数量
	u16 readCnt;

	flush_sendBuff();
	GetVonvertionValue();	//读取电流值

	regAddr=(uart_rev_buff[2]<<8)+uart_rev_buff[3]-MB_REG_START;	//减去起始地址
	regReadCnt=(uart_rev_buff[4]<<8)+uart_rev_buff[5];
	if ((regAddr+regReadCnt)<=MB_REG_MAX)	//防止modbus寄存器访问越界
	{
		uart_send_buff[0]=slave_id;	//从机地址
		uart_send_buff[1]=0x03;	//功能码
		uart_send_buff[2]=regReadCnt*2;	//字节数-->16位变量字符发送
		index_buf=3;	//从第三个字节开始
		for (readCnt = 0 ,i = regAddr; readCnt < regReadCnt; ++readCnt)	//读取寄存器状态
		{
			//先高后低
			uart_send_buff[index_buf]=(u8)(MB_Reg[i]>>8);
			index_buf++;
			uart_send_buff[index_buf]=(u8)MB_Reg[i];
			index_buf++;
			++i;
		}
		crc_det=getCRC16(uart_send_buff,index_buf);
		uart_send_buff[index_buf]=(u8)(crc_det);
		index_buf++;
		uart_send_buff[index_buf]=(u8)(crc_det>>8);
		uart_send_count=1+1+1+regReadCnt*2+2;	//从机地址+功能码+字节数+寄存器状态+crc校验
		MB_Reply(uart_send_buff,uart_send_count);	//从机响应
	}
	else
	{
		uart_send_char(0x04);	//访问数组越界
	}
}
/*
 * info : 写单个寄存器，即16位变量
 * @单个寄存器的数值先高后低
*/
void MB_Write_SingleReg(void)
{
	u16 regAddr,index_buf,crc_det,regValue;

	flush_sendBuff();

	regAddr=(uart_rev_buff[2]<<8)+uart_rev_buff[3]-MB_REG_START;	//减去起始地址
	regValue=(uart_rev_buff[4]<<8)+uart_rev_buff[5];
	if (regAddr<MB_REG_MAX)	//防止modbus寄存器访问越界
	{
		MB_Reg[regAddr]=regValue;	//修改寄存器的值

		uart_send_buff[0]=slave_id;	//从机地址
		uart_send_buff[1]=0x06;	//功能码
		uart_send_buff[2]=uart_rev_buff[2];//寄存器地址
		uart_send_buff[3]=uart_rev_buff[3];
		uart_send_buff[4]=uart_rev_buff[4];//寄存器值
		uart_send_buff[5]=uart_rev_buff[5];

		index_buf=6;//数据校验个数

		crc_det=getCRC16(uart_send_buff,index_buf);
		uart_send_buff[6]=(u8)(crc_det);
		uart_send_buff[7]=(u8)(crc_det>>8);
		uart_send_count=1+1+2+2+2;	//从机地址+功能码+寄存器地址+寄存器值+crc校验
		MB_Reply(uart_send_buff,uart_send_count);	//从机响应
	}
	else
	{
		uart_send_char(0x04);	//访问数组越界
	}
}
