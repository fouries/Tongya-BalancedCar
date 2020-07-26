#include "stm32f4xx.h"    // Test vscode
#include "sys.h"  
u8 Way_Angle=1;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop=0,Flag_Show=0,Flag_Hover=0;    //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage,adc6;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
u32 Distance;                               //���������
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //��ʱ�͵��εȱ���
float Acceleration_Z;                       //Z����ٶȼ�  
float Balance_Kp=10,Balance_Kd=0.04,Balance_Ki=0.0,Velocity_Kp=0.35,Velocity_Ki=0.55;//PID����
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash�������
float Zhongzhi=-0.4;                          //��е��ֵ
u32 Remoter_Ch1=1500,Remoter_Ch2=1500;      //��ģң�ؽ��ձ���
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;     //
int dac_value=1;
long steer_val=0;
void gwsdelay_ms(int ms);
int main(void)
{
//	long tmpval=30000;
	delay_init(168);                //=====��Ƶ168M
	uart_init(128000);              //=====��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====����ϵͳ�ж����ȼ�����2
	LED_Init();                     //=====LED��ʼ��
	KEY_Init();                     //=====������ʼ��
  OLED_Init();                    //=====OLED��ʼ��
//	TIM3_Cap_Init(0XFFFF,84-1);	    //=====��������ʼ��
	//TIM8_Cap_Init(0XFFFF,168-1);	  //=====��ģң�ؽ��ճ�ʼ
	Encoder_Init_TIM2();            //=====��������ʼ��
	Encoder_Init_TIM4();            //=====��������ʼ��
	uart2_init(115200);               //=====����2��ʼ��
	delay_ms(500);                  //=====��ʱ�ȴ�ϵͳ�ȶ�
	IIC_Init();                     //=====IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP 
	Adc_Init();                     //=====ģ�����ɼ���ʼ��
	//NRF24L01_Init();    					//=====��ʼ��NRF24L01  
  //while(NRF24L01_Check());      //=====NRF24L01ģ���Լ죬����֮��������NRF24L01ģ�飬����ż���ִ��
	CAN1_Mode_Init(1,7,6,3,CAN_Mode_Normal);//=====CAN��ʼ��  
	//MiniBalance_PWM_Init(8400,1);   //=====PWM��ʼ��
	MiniBalance_EXTI_Init();        //=====�ⲿ�жϳ�ʼ��
	PS2_Init();									    //=====ps2�����˿ڳ�ʼ�� ������20200525
	PS2_SetInit();		 					    //=====ps2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
	DAC_Config();										//GWS 20200525
	Adcmotor_Init(); 								//gws 20200526
	uart4_init(57600);							//gws 20200526
	delay_ms(1000);
	startmotor();	
	DAC_Cmd(DAC_Channel_1,DISABLE);
  while(1){
		PS2_KEY=PS2_DataKey();
			if(Flash_Send==1)        //д��PID������Flash,��app���Ƹ�ָ��
				{
					Flash_Write();	
					Flash_Send=0;	
				}	
			if(Flag_Show==0)        	  //ʹ��MiniBalance APP��OLED��ʾ��
				{
						//APP_Show();	
					adc6=Get_motor_adc(6);
					printf("%d:%d:%d:%d %d\r\n",(int)Angle_Balance,adc6,Encoder_Left,Encoder_Right,PS2_KEY);
						oled_show();          //===��ʾ����
				}
				else                      //ʹ��MiniBalance��λ�� ��λ��ʹ�õ�ʱ����Ҫ�ϸ��ʱ�򣬹ʴ�ʱ�ر�app��ز��ֺ�OLED��ʾ��
				{
						DataScope();          //����MiniBalance��λ��
				}	
				delay_flag=1;	
				delay_50=0;
				while(delay_flag);	     //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ	
			//	tmpval=tmpval*(-1);
				set_angle(steer_val);
				gwsdelay_ms(20);
				//readstatus();
				if(PS2_KEY==PSB_PAD_UP){
					set_angle(0);
					stoptmotor();
					DAC_Cmd(DAC_Channel_1,DISABLE);
				}
				if(PS2_KEY==PSB_PAD_DOWN){
					DAC_Cmd(DAC_Channel_1,DISABLE);
				}
				if(PS2_KEY==PSB_GREEN){
					DAC_Cmd(DAC_Channel_1,ENABLE);
				}
/*			if(PS2_KEY==PSB_PAD_UP){
				dac_value=dac_value+100;
				if(dac_value>=4096)
					dac_value=4095;
				steer_val=10000;
				set_angle(steer_val);
			}
			if(PS2_KEY==PSB_PAD_DOWN){
				dac_value=dac_value-100;
				if(dac_value<=1000)
					dac_value=1000;
				steer_val=0;
				set_angle(steer_val);
			}	
			if(PS2_KEY==PSB_PAD_RIGHT){
				delay_ms(15);
				stoptmotor();
			}
			if(PS2_KEY==PSB_PAD_LEFT){
				delay_ms(15);
				steer_val=-10000;
				set_angle(steer_val);
			}*/
		//	DAC_SetChannel1Data(DAC_Align_12b_R,dac_value);
			//DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
	}
}
void gwsdelay_us(int us){
	int i;
	for(i=0;i<us;i++){
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();
	}
}
void gwsdelay_ms(int ms){
	int i;
	for(i=0;i<ms;i++){
		gwsdelay_us(1000);
	}
}
