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
#define BLUE_LED GPIO_Pin_2 //PA2
#define GREEN_LED GPIO_Pin_6 //PA6
#define BUTTON_DOWN GPIO_Pin_0 //PA0
#define BUTTON_UP GPIO_Pin_3 //PA3

double setpoint=22;
int ledstatus;
int buton1, buton2;
int count,Controller;
uint8_t Temp_first, Temp_second;
uint16_t Temp;
float Temp1,Tempdif,dutyratio;
char pwmduty[20],temp[20],setp[20];

void GPIO_Configuration(void);
void EXTI0_Config(void);
void EXTI3_Config(void);
void I2C_Config(void);
void TIM2_Configuration(void);
void TIM3_Configuration(void);
void USART_Config(void);

void USART1_PutChar(char c);
void USART1_PutString(char *s);

double c(double dif){
	dutyratio=(dif/0.125)*5;
	if(dutyratio >=100){
		dutyratio=100;}
	else if(dutyratio<100){}
		
	if(dutyratio>=100){
		GPIO_SetBits(GPIOA,GREEN_LED);}
	else if(dutyratio<100){
		GPIO_ResetBits (GPIOA, GREEN_LED);}
	return dutyratio ;
}
float readTemperature (void){
		// Wait if busy
		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
		//Enable ACK
		I2C_AcknowledgeConfig(I2C1, ENABLE);
		// Generate START condition
		I2C_GenerateSTART(I2C1, ENABLE);
		while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));
		// Send device address for write
    I2C_Send7bitAddress(I2C1, 0x90, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) ;
		// Write to device (temperature request)
    I2C_SendData(I2C1, 0x90);
		while ((!I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(I2C1,I2C_FLAG_BTF)));
		// Generate start condition again (Re-start)
    I2C_GenerateSTART(I2C1, ENABLE);
		while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB));
		// Send device address for read
		I2C_Send7bitAddress(I2C1, 0x90, I2C_Direction_Receiver);
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
		
		Temp = (uint16_t)((Temp_first<<8) | Temp_second);
		Temp1 = (float)(Temp>>5)*0.125;	
		return Temp1;
}

int main(){
	GPIO_Configuration();
	I2C_Config();
	EXTI0_Config();
	EXTI3_Config();
	TIM2_Configuration();
	USART_Config();
	TIM3_Configuration();
	
	while(1){
		
		if(Temp1<setpoint){
			Tempdif=setpoint-Temp1;
			ledstatus=0;}
		else if(Temp1>setpoint){
			Tempdif=Temp1-setpoint;
			ledstatus=1;}
		
		buton1=GPIO_ReadInputDataBit (GPIOA,BUTTON_DOWN);
		buton2=GPIO_ReadInputDataBit (GPIOA,BUTTON_UP);
			
		sprintf(setp,"%f\r",setpoint );
		if (buton1 || buton2){
			USART1_PutString("New_Setpoint:");
			USART1_PutString(setp);
			USART1_PutChar('\n');
		}
			
		Controller=c(Tempdif);
		
		sprintf(temp,"%f\r",Temp1);
		sprintf(pwmduty,"%d\r",Controller);
		USART1_PutString("Temperature:");
		USART1_PutString(temp);
		USART1_PutChar('\n');
		USART1_PutString("PWM_duty_ratio:");
		USART1_PutString(pwmduty);
		USART1_PutChar('\n');
		
		switch (ledstatus){
			case 0:
				TIM2->CCR2=Controller;
				TIM2->CCR3=0;
				USART1_PutString("Heating");
				USART1_PutChar('\n');
			break;
			case 1: 
				TIM2->CCR2=0;
				TIM2->CCR3=Controller;
				USART1_PutString("Cooling");
				USART1_PutChar('\n');
			break;
		}
	}
}
void GPIO_Configuration (void){
	//Init struct
	GPIO_InitTypeDef GPIO_InitStructure;
	//Enable clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=BUTTON_DOWN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOA, GPIO_PinSource0);
	
	GPIO_InitStructure.GPIO_Pin=BUTTON_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOA, GPIO_PinSource3);
	
	GPIO_InitStructure.GPIO_Pin=RED_LED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=BLUE_LED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GREEN_LED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// SDA, SCL
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
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
void TIM2_Configuration(void){
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_InitStructure.TIM_Prescaler=719;
	TIM_InitStructure.TIM_Period=99;
	TIM_InitStructure.TIM_ClockDivision=0;
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit (TIM2, &TIM_InitStructure );
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse =0;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure );
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable );
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse =0;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure );
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable );
	
	TIM_ITConfig(TIM2,TIM_IT_CC2|TIM_IT_CC3, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}
void TIM3_Configuration(void){
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_InitStructure.TIM_Prescaler=999;
	TIM_InitStructure.TIM_Period=35999;
	TIM_InitStructure.TIM_ClockDivision=0;
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_InitStructure.TIM_RepetitionCounter =0;
	TIM_TimeBaseInit (TIM3, &TIM_InitStructure );
	TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void EXTI3_Config(void){
	//Init struct
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//EXTI initialization
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
void EXTI0_Config(void){
	//Init struct
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//EXTI initialization
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	//Nested vector interrupt settings
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
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
void USART_Config(void){
	//Init struct
	USART_InitTypeDef USART_InitStructure;
	//Enable clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	//USART settings
	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);
	//Enable data receive interrupt & USART1
	USART_Cmd(USART1, ENABLE);
}
void EXTI0_IRQHandler(void){
	if(EXTI_GetITStatus (EXTI_Line0) !=RESET){
		setpoint=setpoint-0.5;
		EXTI_ClearITPendingBit(EXTI_Line0);}
}
void EXTI3_IRQHandler(void){
	if(EXTI_GetITStatus (EXTI_Line3) !=RESET){
		setpoint=setpoint+0.5;
		EXTI_ClearITPendingBit(EXTI_Line3);}
}
void TIM3_IRQHandler(void){
	if(TIM_GetFlagStatus(TIM3, TIM_IT_Update) != RESET){
		count++;
		readTemperature();
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
 }
 }
void USART1_PutChar(char c){
	// Wait until transmit data register is empty
	while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
	// Send a char using USART2
	USART_SendData(USART1, c);
}

void USART1_PutString(char *s){
	// Send a string
	while (*s)
	{
		USART1_PutChar(*s++);
	}
}