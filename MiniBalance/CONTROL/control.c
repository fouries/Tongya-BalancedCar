#include "control.h"	
#include "filter.h"	
#include "MPU6050.h"
#include "inv_mpu.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target,flag_count=0;
u32 Flash_R_Count;
int Voltage_Temp,Voltage_Count,Voltage_All;
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	//long tmp_val=0;
	 if(INT==0)		
	{   
		   EXTI->PR=1<<15;                                                      //����жϱ�־λ   
		   Flag_Target=!Flag_Target;
			flag_count++;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ50ms�ľ�׼��ʱ
			 }
		  if(Flag_Target==1&&flag_count<3)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ�����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
			{
				if(++Flash_R_Count<=250&&Angle_Balance>30)Flash_Read();             //=====��ȡFlash��PID����		
				Voltage_Temp=Get_battery_volt();		                                //=====��ȡ��ص�ѹ		
				Voltage_Count++;                                                    //=====ƽ��ֵ������
				Voltage_All+=Voltage_Temp;                                          //=====��β����ۻ�
				if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====��ƽ��ֵ		
				return 0;	                                               
			}                                                                   //10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
		//	flag_count=0;
			Encoder_Left=Read_Encoder(2);                                    		//===��ȡ��������ֵ
			Encoder_Right=Read_Encoder(4);                                   		//===��ȡ��������ֵ
	  	Get_Angle(Way_Angle);                                               //===������̬	
			if(Angle_Balance>6.6||Angle_Balance<-7.4){
					set_angle(0);
					stoptmotor();
					DAC_Cmd(DAC_Channel_1,DISABLE);
			}
 		//	Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                   //===ƽ��PID����	
	   // Velocity_Pwm=servo(Balance_Pwm+Velocity_Pwm);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
		//	Moto1=myabs(Balance_Pwm)/Balance_Kp;	                        //===�ٶȿ��� �������
			if(Moto1>4096)  Moto1=4096;	                                    //===�ٶ��޷�
		  if(Moto1<800)  Moto1=800;	 																		//===�ٶ��޷�
 	 // 	Moto2=Balance_Pwm+Velocity_Pwm;                            //===�������ֵ������PWM
	//		if(Moto2<-30000) Moto2=-30000;	                                //===����Ƕ�����
		//  if(Moto2>30000)  Moto2=30000;	 																//===����Ƕ�����
   		Xianfu_Pwm();                                                       //===PWM�޷�
   //   if(Turn_Off(Angle_Balance,Voltage)==0)                              //===����������쳣
			//{
				steer_val=MyBalance(Angle_Balance,Gyro_Balance);
				if(steer_val<-30000) steer_val=-30000;	                                //===����Ƕ�����
				if(steer_val>30000)  steer_val=30000;	 																//===����Ƕ�����
		/*	if(tmp_val-steer_val>2000||tmp_val-steer_val<-2000){
					steer_val=tmp_val;
			}*/
				Moto1=Incremental_PI_A(Encoder_Right,Moto1);
			//	printf("%.2f:%ld\r\n",Angle_Balance,steer_val);
				Set_Pwm(Moto1,Moto2);                                               //===��ֵ��PWM�Ĵ���
		//	}
			Key();                                                              //===ɨ�谴��״̬ ����˫�����Ըı�С������״̬			
	}       	
	 return 0;	  
} 
long MyBalance(float Angle, float Gyro){
	long value;
	value=(int)((Angle_Balance-Zhongzhi)*27+0.00*Gyro_Balance)*222.25;
	return value;
}
/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias;
	 static float Integration;
	 int balance;
	 Bias=Angle-Zhongzhi;       //===���ƽ��ĽǶ���ֵ �ͻ�е���
   Integration+=Bias;
	 if(Integration<300)  Integration=300;	
	 if(Integration>300)  Integration=300;	  
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd+Integration*Balance_Ki;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

int servo(int servo_input)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral;
	  //=============ң��ǰ�����˲���=======================// 
		if(1==Flag_Right)    	Movement=-100;	      //===ǰ����־λ��1 
		else if(1==Flag_Left)	Movement=100;         //===���˱�־λ��1
	  else  Movement=0;	
   //=============�ٶ�PI������=======================//	
		Encoder_Least =servo_input-0;                    									//===��ȡ����ƫ��
		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>6000)  	Encoder_Integral=6000;             		//===�����޷�
		if(Encoder_Integral<-6000)	Encoder_Integral=-6000;              	//===�����޷�	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki/100;    //===�ٶȿ���	
	  if(Flag_Stop==1)Encoder_Integral=0;
	  return Velocity;
}
int Incremental_PI_A(int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=28*(Bias-Last_bias)+28*Bias;   //����ʽPI������
	 if(Pwm>4096)Pwm=4096;
	 if(Pwm<800)Pwm=800;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
/**************************************************************************
�������ܣ�ת�����  �޸�ת���ٶȣ����޸�Turn_Amplitude����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
	/* static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9f,Turn_Count;
	  float Turn_Amplitude=100/Flag_sudu,Kp=52,Kd=0;     
	  //=============ң��������ת����=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);
			Turn_Convert=50/Encoder_temp;
			if(Turn_Convert<0.6f)Turn_Convert=0.6f;
			if(Turn_Convert>3)Turn_Convert=3;
		}	
	  else
		{
			Turn_Convert=0.9f;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;
		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert; 
		else Turn_Target=0;
	
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5f;        
		else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  	//=============ת��PD������=======================//
		Turn=-Turn_Target*Kp-gyro*Kd;                 //===���Z�������ǽ���PD����
	  return Turn;*/
		return 0;
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	dac_value=moto1;
	//steer_val=moto2;
	//set_angle(steer_val);
	if(dac_value>4096)  dac_value=4096;	                                    //===�ٶ��޷�
	if(dac_value<2500)  dac_value=2500;	 																		//===�ٶ��޷�
	DAC_SetChannel1Data(DAC_Align_12b_R,dac_value);
	DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=30000;    //===PWM������8400 ������8200
    if(Moto1<800) Moto1=800;	
		if(Moto1>4095)  Moto1=4095;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
	
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(50); 
	if(tmp==1)Flag_Stop=!Flag_Stop;//��������С������ͣ
	if(tmp==2)Flag_Show=!Flag_Show;//˫������С������ʾ״̬
	tmp2=Long_Press();                   
  if(tmp2==1) Bi_zhang=!Bi_zhang;		//��������С���Ƿ���볬����ģʽ 
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	    u8 temp;
			if(angle<(-20+Zhongzhi)||angle>(20+Zhongzhi)||1==Flag_Stop||voltage<1110)//��ص�ѹ����11.1V�رյ��
			{	                                                 //===��Ǵ���40�ȹرյ��
				temp=1;                                            //===Flag_Stop��1�رյ��
				dac_value=1;
				DAC_SetChannel1Data(DAC_Align_12b_R,dac_value);
				DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
				steer_val=0;
				set_angle(0);
				delay_ms(15);
				stoptmotor();
      }
			else
      temp=0;
      return temp;			
}
	
/**************************************************************************
�������ܣ���ȡ�Ƕ� �����㷨�������ǵĵ�У�����ǳ����� 
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
void Get_Angle(u8 way)
{ 
	   	Temperature=Read_Temperature();      //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	    if(way==1)                           //===DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
			{	
					Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
					Angle_Balance=-Roll;             //===����ƽ�����
					Gyro_Balance=-gyro[0];            //===����ƽ����ٶ�
					Gyro_Turn=gyro[2];               //===����ת����ٶ�
				  Acceleration_Z=accel[2];         //===����Z����ٶȼ�
			}			
      else
      {
		}
}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ����С���Ƿ�����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   

	return 0;
}
/**************************************************************************
�������ܣ����С���Ƿ񱻷���
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   

	return 0;
}


/**************************************************************************
�������ܣ��ɼ�ң�������ź�
��ڲ�������
����  ֵ����
**************************************************************************/
void  Get_MC6(void)
{ 

}	

