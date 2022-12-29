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

#define RED_LED GPIO_Pin_1 //PA1
#define BUTTON GPIO_Pin_3 //PA3
#define POTENTIOMETER GPIO_Pin_0//PA0

char data;
uint32_t adcData;
uint32_t potval;
double treshold;
uint16_t Temp_first, Temp_second;
double Temp;
char Temperature[4];
int count=0;

void GPIO_Configuration(void);
void TIM3_Config(void);
void EXTI3_Config(void);
void USART_Config(void);
void ADC_Configuration(void);
void I2C_Config(void);
void Transmit_UART(char *read_data);
static __IO uint16_t MS_count;

static volatile uint32_t ticks;
void delayMs(uint32_t nTime) 
{ 
 	ticks = nTime; 
 	while(ticks); 
}
int main(){
  GPIO_Configuration(); 
	ADC_Configuration();
	USART_Config(); 
	EXTI3_Config (); //to adjust treshold
	I2C_Config();
	
	while(1){
		adcData=ADC_GetConversionValue(ADC1);
		potval=adcData*50/4095;	
		SysTick_Config(SystemCoreClock/1000);
		MS_count=0;
		// Wait if busy
		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
		// Generate START condition
		I2C_GenerateSTART(I2C1, ENABLE);
		while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));
		// Send device address for read
		I2C_Send7bitAddress(I2C1,0x90, I2C_Direction_Receiver);
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
		// Read the first data
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		Temp_first = I2C_ReceiveData(I2C1);
		// Disable ACK and generate stop condition
		I2C_AcknowledgeConfig(I2C1, DISABLE);
		I2C_GenerateSTOP(I2C1, ENABLE);
		// Read the second data
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		Temp_second= I2C_ReceiveData(I2C1);
		Temp= Temp_first+ (double) (Temp_second>>7)*0.5;
		
		sprintf(Temperature, "%f\r", Temp);
		Transmit_UART(Temperature);
		
		if(data=='0' || Temp<treshold){
			GPIO_WriteBit(GPIOA, RED_LED,Bit_RESET);}
		if(data=='1' && Temp>treshold){
			GPIO_WriteBit(GPIOA, RED_LED,Bit_SET);}
		else {}			
		}

		if(MS_count>1000){
			MS_count=0;
		}
	
}
void GPIO_Configuration (void){
	//Init struct
	GPIO_InitTypeDef GPIO_InitStructure;
	//Enable clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);

  //Led structures
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = RED_LED;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//Button structure
	GPIO_InitStructure.GPIO_Pin = BUTTON;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOA, GPIO_PinSource3);

	//Potentiometer structure	
	GPIO_InitStructure.GPIO_Pin = POTENTIOMETER;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
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
	
	// SDA, SCL
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void EXTI3_Config(void){
	//Init struct
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//EXTI initialization
	EXTI_DeInit();
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	//Nested vector interrupt settings
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART_Config(void){
	//Init struct
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
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
	//Enable data receive interrupt & USART1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void ADC_Configuration(void){
    //Init struct
    ADC_InitTypeDef ADC_InitStructure;
		
		//Enable clock
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);
		// ADC1 initialization
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent ;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1; 
    ADC_Init(ADC1, &ADC_InitStructure);

		ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
		// Enable ADC1
		ADC_Cmd(ADC1, ENABLE);
		
		ADC_ResetCalibration(ADC1); 
    while(ADC_GetResetCalibrationStatus(ADC1)); 
    ADC_StartCalibration(ADC1); 
    while(ADC_GetCalibrationStatus(ADC1));
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		
}
void I2C_Config(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	I2C_InitTypeDef I2C_InitStructure;
	
	I2C_InitStructure.I2C_Mode= I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle= I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1= 0x00;
	I2C_InitStructure.I2C_Ack= I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed= 100000;
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

void EXTI3_IRQHandler(void){
	if(EXTI_GetITStatus (EXTI_Line3) !=RESET){
		treshold=potval;
		EXTI_ClearITPendingBit(EXTI_Line3);}
}
void USART1_IRQHandler(void){
		if(USART_GetITStatus (USART1, USART_IT_RXNE)==SET){
			data=USART_ReceiveData(USART1);
			USART_ClearITPendingBit (USART1, USART_IT_RXNE);}
}	
void Transmit_UART(char read_data[]){
	while( *read_data){
		while(!(USART1->SR & 0x00000040));
		USART_SendData(USART1,*read_data);
		*read_data++;
    }
}
void SysTick_Handler(void){
	MS_count++;
}