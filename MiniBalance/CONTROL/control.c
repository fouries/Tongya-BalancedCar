#include "control.h"	
#include "filter.h"	
#include "MPU6050.h"
#include "inv_mpu.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target,flag_count=0;
u32 Flash_R_Count;
int Voltage_Temp,Voltage_Count,Voltage_All;
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	//long tmp_val=0;
	 if(INT==0)		
	{   
		   EXTI->PR=1<<15;                                                      //清除中断标志位   
		   Flag_Target=!Flag_Target;
			flag_count++;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //给主函数提供50ms的精准延时
			 }
		  if(Flag_Target==1&&flag_count<3)                                                  //5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
			{
				if(++Flash_R_Count<=250&&Angle_Balance>30)Flash_Read();             //=====读取Flash的PID参数		
				Voltage_Temp=Get_battery_volt();		                                //=====读取电池电压		
				Voltage_Count++;                                                    //=====平均值计数器
				Voltage_All+=Voltage_Temp;                                          //=====多次采样累积
				if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====求平均值		
				return 0;	                                               
			}                                                                   //10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
		//	flag_count=0;
			Encoder_Left=Read_Encoder(2);                                    		//===读取编码器的值
			Encoder_Right=Read_Encoder(4);                                   		//===读取编码器的值
	  	Get_Angle(Way_Angle);                                               //===更新姿态	
			if(Angle_Balance>6.6||Angle_Balance<-7.4){
					set_angle(0);
					stoptmotor();
					DAC_Cmd(DAC_Channel_1,DISABLE);
			}
 		//	Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                   //===平衡PID控制	
	   // Velocity_Pwm=servo(Balance_Pwm+Velocity_Pwm);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		//	Moto1=myabs(Balance_Pwm)/Balance_Kp;	                        //===速度控制 弯道加速
			if(Moto1>4096)  Moto1=4096;	                                    //===速度限幅
		  if(Moto1<800)  Moto1=800;	 																		//===速度限幅
 	 // 	Moto2=Balance_Pwm+Velocity_Pwm;                            //===计算右轮电机最终PWM
	//		if(Moto2<-30000) Moto2=-30000;	                                //===舵机角度限制
		//  if(Moto2>30000)  Moto2=30000;	 																//===舵机角度限制
   		Xianfu_Pwm();                                                       //===PWM限幅
   //   if(Turn_Off(Angle_Balance,Voltage)==0)                              //===如果不存在异常
			//{
				steer_val=MyBalance(Angle_Balance,Gyro_Balance);
				if(steer_val<-30000) steer_val=-30000;	                                //===舵机角度限制
				if(steer_val>30000)  steer_val=30000;	 																//===舵机角度限制
		/*	if(tmp_val-steer_val>2000||tmp_val-steer_val<-2000){
					steer_val=tmp_val;
			}*/
				Moto1=Incremental_PI_A(Encoder_Right,Moto1);
			//	printf("%.2f:%ld\r\n",Angle_Balance,steer_val);
				Set_Pwm(Moto1,Moto2);                                               //===赋值给PWM寄存器
		//	}
			Key();                                                              //===扫描按键状态 单击双击可以改变小车运行状态			
	}       	
	 return 0;	  
} 
long MyBalance(float Angle, float Gyro){
	long value;
	value=(int)((Angle_Balance-Zhongzhi)*27+0.00*Gyro_Balance)*222.25;
	return value;
}
/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias;
	 static float Integration;
	 int balance;
	 Bias=Angle-Zhongzhi;       //===求出平衡的角度中值 和机械相关
   Integration+=Bias;
	 if(Integration<300)  Integration=300;	
	 if(Integration>300)  Integration=300;	  
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd+Integration*Balance_Ki;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

int servo(int servo_input)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral;
	  //=============遥控前进后退部分=======================// 
		if(1==Flag_Right)    	Movement=-100;	      //===前进标志位置1 
		else if(1==Flag_Left)	Movement=100;         //===后退标志位置1
	  else  Movement=0;	
   //=============速度PI控制器=======================//	
		Encoder_Least =servo_input-0;                    									//===获取最新偏差
		Encoder *= 0.7;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>6000)  	Encoder_Integral=6000;             		//===积分限幅
		if(Encoder_Integral<-6000)	Encoder_Integral=-6000;              	//===积分限幅	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki/100;    //===速度控制	
	  if(Flag_Stop==1)Encoder_Integral=0;
	  return Velocity;
}
int Incremental_PI_A(int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=28*(Bias-Last_bias)+28*Bias;   //增量式PI控制器
	 if(Pwm>4096)Pwm=4096;
	 if(Pwm<800)Pwm=800;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
/**************************************************************************
函数功能：转向控制  修改转向速度，请修改Turn_Amplitude即可
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	/* static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9f,Turn_Count;
	  float Turn_Amplitude=100/Flag_sudu,Kp=52,Kd=0;     
	  //=============遥控左右旋转部分=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
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
	
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5f;        
		else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  	//=============转向PD控制器=======================//
		Turn=-Turn_Target*Kp-gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
	  return Turn;*/
		return 0;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	dac_value=moto1;
	//steer_val=moto2;
	//set_angle(steer_val);
	if(dac_value>4096)  dac_value=4096;	                                    //===速度限幅
	if(dac_value<2500)  dac_value=2500;	 																		//===速度限幅
	DAC_SetChannel1Data(DAC_Align_12b_R,dac_value);
	DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=30000;    //===PWM满幅是8400 限制在8200
    if(Moto1<800) Moto1=800;	
		if(Moto1>4095)  Moto1=4095;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
	
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(50); 
	if(tmp==1)Flag_Stop=!Flag_Stop;//单击控制小车的启停
	if(tmp==2)Flag_Show=!Flag_Show;//双击控制小车的显示状态
	tmp2=Long_Press();                   
  if(tmp2==1) Bi_zhang=!Bi_zhang;		//长按控制小车是否进入超声波模式 
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	    u8 temp;
			if(angle<(-20+Zhongzhi)||angle>(20+Zhongzhi)||1==Flag_Stop||voltage<1110)//电池电压低于11.1V关闭电机
			{	                                                 //===倾角大于40度关闭电机
				temp=1;                                            //===Flag_Stop置1关闭电机
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
函数功能：获取角度 三种算法经过我们的调校，都非常理想 
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{ 
	   	Temperature=Read_Temperature();      //===读取MPU6050内置温度传感器数据，近似表示主板温度。
	    if(way==1)                           //===DMP的读取在数据采集中断读取，严格遵循时序要求
			{	
					Read_DMP();                      //===读取加速度、角速度、倾角
					Angle_Balance=-Roll;             //===更新平衡倾角
					Gyro_Balance=-gyro[0];            //===更新平衡角速度
					Gyro_Turn=gyro[2];               //===更新转向角速度
				  Acceleration_Z=accel[2];         //===更新Z轴加速度计
			}			
      else
      {
		}
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：检测小车是否被拿起
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   

	return 0;
}
/**************************************************************************
函数功能：检测小车是否被放下
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   

	return 0;
}


/**************************************************************************
函数功能：采集遥控器的信号
入口参数：无
返回  值：无
**************************************************************************/
void  Get_MC6(void)
{ 

}	

