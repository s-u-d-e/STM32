#include <stdio.h>
#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"

void GPIO_Configuration(void);
void ADC_Configuration(void);
void TIM2_Configuration(void);
void TIM3_Configuration(void);
void EXTI0_Configuration(void);
void USART_Configuration(void);
void Transmit_UART(char *read_data);

int reference;
double V_out,Vout;
char Vo[20],Cont[20],ref[20];

volatile double Ka = 2.5, Kp = 2.8, Ki = 0.8;
double Controller=0;
// PI Controller
double PI(double r, double y) {
static double uk_1 = 0, ek_1 = 0;// ek_2 = 0;
double ek = r - y;
double uk = uk_1+(2.8)*ek-(2.72)*ek_1 ; 
uk_1 = uk;
ek_1 = ek;
return uk; 
}
//P controller
double P(double r, double y) {
static double uk_1 = 0, ek_1 = 0;// ek_2 = 0;
double ek = r - y;
double uk = uk_1 + (Ka) * ek - (Ka) * ek_1 ; 
uk_1 = uk;
ek_1 = ek;
return uk; 
}
//Transfer function
double Gz(double uk){
static double yk_1=0, yk_2=0, uk_1=0, uk_2=0;
double yk = 1.822*yk_1-0.8262*yk_2+0.002134*uk_1+0.002002*uk_2;
uk_2 = uk_1;
uk_1 = uk;
yk_2 = yk_1;
yk_1 = yk;
return yk;
}


int main(){
	GPIO_Configuration();
	ADC_Configuration ();
	EXTI0_Configuration();
	TIM2_Configuration();
	USART_Configuration();
	TIM3_Configuration();
	while(1){
		//real time reference
		V_out=ADC_GetConversionValue (ADC1);
		Vout=V_out*3.3/4095; //converting to volt value
		sprintf(Vo, "%f\r", Vout);
		sprintf(Cont, "%f\r", Controller);
		sprintf(ref, "%f\r", reference);
		if(Vout<0){
			Vout=0;
		}
		if(reference<0){
			Vout=0;
		}
		Transmit_UART(Vo);
		//Transmit_UART(Cont);
		//Transmit_UART(ref);
		switch(reference){
			case 0:
				TIM2->CCR3=0;
			break;
			case 1://36000->3.3V 36000/3.3=1V
				TIM2->CCR3=10909*Controller; 
			break;}
	}
}
void GPIO_Configuration(void){
	//Init structure
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);	
	//Button
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOA, GPIO_PinSource0);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//Vout
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//Vin
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//UART RX
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//UART TX
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void EXTI0_Configuration(void){
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM2_Configuration(void){
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_InitStructure.TIM_Prescaler=9;
	TIM_InitStructure.TIM_Period=35999;
	TIM_InitStructure.TIM_ClockDivision=0;
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit (TIM2, &TIM_InitStructure );
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse =0;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure );
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable );
	
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}
void TIM3_Configuration(void){
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_InitStructure.TIM_Prescaler=199;
	TIM_InitStructure.TIM_Period=35999;
	TIM_InitStructure.TIM_ClockDivision=0;
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_InitStructure.TIM_RepetitionCounter =0;
	TIM_TimeBaseInit (TIM3, &TIM_InitStructure );
	TIM_ITConfig(TIM3, TIM_IT_Update ,ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void ADC_Configuration(void){
	//Init structure
	ADC_InitTypeDef ADC_InitStructure;
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1; 
  ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);
	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1); 
	while(ADC_GetResetCalibrationStatus(ADC1)){;}
  ADC_StartCalibration(ADC1); 
  while(ADC_GetCalibrationStatus(ADC1)){;}
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
void USART_Configuration(void){
	//Init struct
	USART_InitTypeDef USART_InitStructure;
	//Enable clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	//USART settings
	USART_InitStructure.USART_BaudRate=19200;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);
	//Enable USART1
	USART_Cmd(USART1, ENABLE);
}
void EXTI0_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line0) !=RESET){
		if(reference==0){
			reference=1;}
		else{
			reference=0;}
   EXTI_ClearITPendingBit(EXTI_Line0);}
 }
void TIM3_IRQHandler(void){
	if(TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) != RESET){
		if(reference==1){
			Controller=PI(1,Vout);
			//Vout= Gz(Controller);
			}
		else if(reference==0){
			Controller=PI(0,Vout);
			//Vout=Gz(Controller);
			}
	TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
 }
 }
void Transmit_UART(char read_data[]){
	while( *read_data){
		while(!(USART1->SR & 0x00000040));
		USART_SendData(USART1,*read_data);
		*read_data++;
    }
}