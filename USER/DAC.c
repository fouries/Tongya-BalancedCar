#include "dac.h"	

void DAC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&GPIO_InitStructure);
  
}
void DAC_Config(void)
{
	DAC_InitTypeDef DAC_InitStructure;
	
	DAC_GPIO_Config();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1,&DAC_InitStructure);
  
  DAC_Cmd(DAC_Channel_1,ENABLE);
  DAC_SetChannel1Data(DAC_Align_12b_R,0x0000);
  
  DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
}

