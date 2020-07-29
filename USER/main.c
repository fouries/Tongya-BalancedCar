#include "stm32f4xx.h"
#include "sys.h"  
u8 Way_Angle=1;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //蓝牙遥控相关的变量
u8 Flag_Stop=0,Flag_Show=0,Flag_Hover=0;    //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //显示温度
int Voltage,adc6;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
u32 Distance;                               //超声波测距
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //延时和调参等变量
float Acceleration_Z;                       //Z轴加速度计  
float Balance_Kp=10,Balance_Kd=0.04,Balance_Ki=0.0,Velocity_Kp=0.35,Velocity_Ki=0.55;//PID参数
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash相关数组
float Zhongzhi=-0.4;                          //机械中值
u32 Remoter_Ch1=1500,Remoter_Ch2=1500;      //航模遥控接收变量
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;     //
int dac_value=1;
long steer_val=0;
void gwsdelay_ms(int ms);
int main(void)
{
//	long tmpval=30000;
	delay_init(168);                //=====主频168M
	uart_init(128000);              //=====延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//=====设置系统中断优先级分组2
	LED_Init();                     //=====LED初始化
	KEY_Init();                     //=====按键初始化
  OLED_Init();                    //=====OLED初始化
//	TIM3_Cap_Init(0XFFFF,84-1);	    //=====超声波初始化
	//TIM8_Cap_Init(0XFFFF,168-1);	  //=====航模遥控接收初始
	Encoder_Init_TIM2();            //=====编码器初始化
	Encoder_Init_TIM4();            //=====编码器初始化
	uart2_init(115200);               //=====串口2初始化
	delay_ms(500);                  //=====延时等待系统稳定
	IIC_Init();                     //=====IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
  DMP_Init();                     //=====初始化DMP 
	Adc_Init();                     //=====模拟量采集初始化
	//NRF24L01_Init();    					//=====初始化NRF24L01  
  //while(NRF24L01_Check());      //=====NRF24L01模块自检，开启之后必须接入NRF24L01模块，程序才继续执行
	CAN1_Mode_Init(1,7,6,3,CAN_Mode_Normal);//=====CAN初始化  
	//MiniBalance_PWM_Init(8400,1); //=====PWM初始化
	MiniBalance_EXTI_Init();        //=====外部中断初始化
	PS2_Init();									    //=====ps2驱动端口初始化 古文生20200525
	PS2_SetInit();		 					    //=====ps2配置初始化,配置“红绿灯模式”，并选择是否可以修改
	DAC_Config();										//GWS 20200525
	Adcmotor_Init(); 								//gws 20200526
	uart4_init(57600);							//gws 20200526
	delay_ms(1000);
	startmotor();	
	DAC_Cmd(DAC_Channel_1,DISABLE);
  while(1){
		PS2_KEY=PS2_DataKey();
			if(Flash_Send==1)        //写入PID参数到Flash,由app控制该指令
				{
					Flash_Write();	
					Flash_Send=0;	
				}	
			if(Flag_Show==0)        	  //使用MiniBalance APP和OLED显示屏
				{
						//APP_Show();	
					adc6=Get_motor_adc(6);
					printf("%d:%d:%d:%d %d\r\n",(int)Angle_Balance,adc6,Encoder_Left,Encoder_Right,PS2_KEY);
						oled_show();          //===显示屏打开
				}
				else                      //使用MiniBalance上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
				{
						DataScope();          //开启MiniBalance上位机
				}	
				delay_flag=1;	
				delay_50=0;
				while(delay_flag);	     //通过MPU6050的INT中断实现的50ms精准延时	
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
