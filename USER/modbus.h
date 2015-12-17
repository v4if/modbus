#ifndef __MODBUS_H__
#define __MODBUS_H__
#include "stm32f10x_type.h"

#define MB_REG_MAX 8	//寄存器数量
#define MB_REG_START 0 //起始寄存器地址
#define MB_SEND_BUF_MAX 100	//发送最大数量
unsigned short getCRC16(volatile u8 *ptr,u8 len);
void MB_Init(void);
void MB_Poll(void);
void flush_revBuff(void);
void MB_Read_MutiReg(void);
void MB_Write_SingleReg(void);

#endif
