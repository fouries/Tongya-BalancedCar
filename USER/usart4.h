#ifndef __USRAT4_H
#define __USRAT4_H 
#include "sys.h"	  	
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
extern long Usart4_Receive;
void uart4_init(u32 bound);
void UART4_IRQHandler(void);
void sendcmdtomotor(u8 addr,int val);
void readstatus(void);
void setspeed(int val);
void stoptmotor(void);
void startmotor(void);
//void set_angle(int val);
void set_angle(long val);
#endif

