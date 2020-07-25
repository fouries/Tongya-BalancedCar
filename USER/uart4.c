#include "usart4.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
long Usart4_Receive;
u8 motorret,premotorret;

/**************************************************************************
�������ܣ�����4��ʼ��
��ڲ����� bound:������
����  ֵ����
**************************************************************************/
void uart4_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��USART2ʱ��
 
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOA2����ΪUSART1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOA3����ΪUSART1
	
	//USART2�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA2��GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PA2 3

   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART4, &USART_InitStructure); //��ʼ������
	
  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
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
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void UART4_IRQHandler(void)
{	
	u8 tmp;
	static int flag=0,point=1;
	static int dat=0;
	static u8 k=0,arr[2];
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //���յ�����
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

