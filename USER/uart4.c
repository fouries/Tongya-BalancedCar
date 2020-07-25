#include "usart4.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
long Usart4_Receive;
u8 motorret,premotorret;

/**************************************************************************
函数功能：串口4初始化
入口参数： bound:波特率
返回  值：无
**************************************************************************/
void uart4_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART2时钟
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOA2复用为USART1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOA3复用为USART1
	
	//USART2端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA2 3

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口
	
  USART_Cmd(UART4, ENABLE);  //使能串口
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
//	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);	
//	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); 
}
void sendcmdtomotor(u8 addr,int val)
{
	u8 dat1,dat2,chk;
	dat1=(val>>8)&0xff;
	dat2=val&0xff;
	chk=(addr+dat1+dat2)&0xff;
	while((UART4->SR&0X40)==0); 
	UART4->DR =addr; 
	while((UART4->SR&0X40)==0); 
	UART4->DR =dat1;  
	while((UART4->SR&0X40)==0); 
	UART4->DR =dat2;  
	while((UART4->SR&0X40)==0); 
	UART4->DR =chk;  		
}
/**************************************************************************
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
void UART4_IRQHandler(void)
{	
	u8 tmp;
	static int flag=0,point=1;
	static int dat=0;
	static u8 k=0,arr[2];
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //接收到数据
	{	  
		USART_ClearFlag(UART4, USART_IT_RXNE);
		tmp=USART_ReceiveData(UART4);
		if(tmp==0xe8){
			flag=1;
			return ;
		}
		if(tmp==0xe9){
			flag=2;
			return ;
		}
		if(flag==1){
			arr[k++]=tmp;
			if(k==2){
				dat=arr[0];
				dat=(dat<<8)|arr[1];
				k=0;
				flag=0;
				if(dat!=0){
					point=-1;
					dat=0;
				}
			}
		}
		if(flag==2){
			arr[k++]=tmp;
			if(k==2){
					dat=arr[0];
					dat=(dat<<8)|arr[1];
					if(point==-1){
						dat=dat-1;
						dat=~dat;
					}
					dat=dat*point;
					k=0;
					flag=0;
					point=1;
			}
		}
	Usart4_Receive=dat;
	}
   											 
} 
void startmotor()
{
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x00; 
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x00;
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x01; 
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x01; 
}
void stoptmotor()
{
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x00; 
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x00;
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x00; 
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x00; 
}
void motor_init()
{
	sendcmdtomotor(0x02,9);
}
void readstatus()
{
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x80; 
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x00;
	while((UART4->SR&0X40)==0); 
	UART4->DR =0x80; 
}
/*void set_angle(int val)
{
	sendcmdtomotor(0x50,0);
	delay_ms(15);
	sendcmdtomotor(0x05,val);
}*/
void set_angle(long val)
{
	int gao,di;
	if(val<0){
		val=val*(-1);
		val=~val+1;
	}
	gao=(val>>16)&0xffff;
	di=val&0xffff;
	sendcmdtomotor(0x50,gao);
	delay_ms(15);
	sendcmdtomotor(0x05,di);
}

