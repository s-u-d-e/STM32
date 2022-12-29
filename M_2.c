#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_adc.h"

#define RED_LED GPIO_Pin_2 //PA2
#define BUTTON GPIO_Pin_3 //PA3
#define POTENTIOMETER GPIO_Pin_0//PA0

int durum=0,butonstate,buton;
int butontime=0,presstime=0;
double volt=0;
double Brightness=0;
uint32_t potval=0;
double count;

int shortbutonpress=0;
int longbutonpress=0;

static __IO uint16_t buttonCounter;

void GPIO_Configuration(void);
void EXTI0_Configuration(void);
void EXTI3_IRQHandler(void);
void ADC_Configuration(void);
void ADC1_2_IRQHandler(void);
void TIM2_Configuration(void);
void TIM3_Configuration(void);

int main(){
    GPIO_Configuration();
    TIM2_Configuration();
	TIM3_Configuration();
	ADC_Configuration();
	EXTI0_Configuration(); 
		
	
	while(1){   
	buton=GPIO_ReadInputDataBit(GPIOA,BUTTON); 
	potval=ADC_GetConversionValue (ADC1 );
	volt=(double) potval/4095;
	Brightness=volt*7200;
		

			
	presstime=buttonCounter-butontime; //the time while button is being pressed
		if(buton==1){ 
			if(presstime<1000){//shortpress
					butonstate=1;
			}
			if(presstime>=1000){ //longpress
					butonstate=2;
			}
		}
			
    switch (durum)
    {
        case 0: //LED off
            TIM2->CCR3=0;
			if(butonstate==1){
				if((shortbutonpress)%2==0){
						durum=1;}
			} // if buton is pressed short LED is turning on
        break;
        case 1: //LED on
            TIM2->CCR3=Brightness;
			if(butonstate==1){
				if(shortbutonpress%2==1){
					durum=0;}} //turn off
			if(butonstate==3){
					durum=2;} //double click					// turn off
        break;
		case 2: //LED is blinking
			TIM2->CCR3=7200;
			for(count=0;count<0.5;count++){}
				TIM2->CCR3=0;
			for(count=0;count<0.5;count++){}	
				if(butonstate==3){
					durum=1;} // constant turn on
		break;
		default:
		break;
    }
}

}
void GPIO_Configuration (void){
	//Init struct
	GPIO_InitTypeDef GPIO_InitStructure;
	//Enable clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);

  	//Led structures
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
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
}


void ADC_Configuration(void){
    //Init struct
    ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//Enable clock
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);
	// ADC1 initialization
	ADC_InitStructure.ADC_Mode = ADC_IT_EOC ; //end of conversion
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1; 
    ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
	// Enable ADC1 and external trigger conversion mode of ADC1
	ADC_Cmd(ADC1, ENABLE);
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
		
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE); 
		
	NVIC_EnableIRQ(ADC1_2_IRQn);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
    ADC_ResetCalibration(ADC1); 
    while(ADC_GetResetCalibrationStatus(ADC1)); 
    ADC_StartCalibration(ADC1); 
    while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		
}
void  ADC1_2_IRQHandler(void){
	potval=ADC1->DR;
}
void TIM2_Configuration(void){
	//Init struct
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	//Enable clock for TIM2
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	//TIM2 initialization
	TIM_TimeBaseInitStruct.TIM_Prescaler = 9;
	TIM_TimeBaseInitStruct.TIM_Period = 35999;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_TimeBaseInitStruct.TIM_Prescaler = 9;
	TIM_TimeBaseInitStruct.TIM_Period = 3599;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High; 
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//Enable TIM2 channel 2
	TIM_ITConfig(TIM2, TIM_IT_CC2|TIM_IT_CC3, ENABLE);
	//Start TIM2
	TIM_Cmd(TIM2, ENABLE);
}
void TIM3_Configuration(){
	// Init struct
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	// Enable clock for TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	// TIM3 initialization for overflow every 500ms
	// Update Event (Hz) = timer_clock / ((TIM_Prescaler + 1) * (TIM_Period + 1))
	// Update Event (Hz) = 72MHz / ((3599 + 1) * (9999 + 1)) = 2Hz (0.5s)
	TIM_TimeBaseInitStruct.TIM_Prescaler = 3599;
	TIM_TimeBaseInitStruct.TIM_Period = 19999;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
	TIM_OCStructInit (&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High; 

	// Enable TIM3 interrupt
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	// Start TIM3
	TIM_Cmd(TIM3, ENABLE);
	
	// Nested vectored interrupt settings
	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}
void TIM3_IRQHandler(){
	// Checks whether the TIM3 interrupt has occurred or not
	if (TIM_GetITStatus(TIM3, TIM_IT_Update))
	{
		// Toggle LED on PB4
		GPIOB->ODR ^= GPIO_Pin_4;
     	count++;
		// Clears the TIM3 interrupt pending bit
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}	
void EXTI0_Configuration (void) {
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
void EXTI3_IRQHandler(void){
	
	if(EXTI_GetITStatus(EXTI_Line3) !=RESET){
       butontime=buttonCounter; //takes the time when buton is pressed
				
		if(butonstate==1){
			shortbutonpress++; //to measure the number of short pressing
			}
		else if(butonstate==2){
			longbutonpress++; //to measure the number of long pressing
			}
		if (buton==1){
			for(count=0;count<0.2;count++){
				if (buton==1){
					butonstate=3;}
			}
    }
    EXTI_ClearITPendingBit(EXTI_Line3);
 	}
}
void SysTick_Handler (void){   
		buttonCounter++; //counts while buton is pressed
}