#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"

//Defining GPIO pins for LEDs, button and potentiometer
#define RED_LED GPIO_Pin_3
#define YELLOW_LED GPIO_Pin_1
#define GREEN_LED GPIO_Pin_2
#define BUTTON GPIO_Pin_0 //PA0
#define BUTTON_UP GPIO_Pin_5
#define BUTTON_DOWN GPIO_Pin_10



int durum=0;
int led_state=0,buton1,buton2;

double volt=0;
double Brightness=0;

double Full=7200;
double  High=0.7*7200;
double Medium=0.4*7200;
double  Low=0.1*7200;
/*Prototypes of configuration functions*/
void GPIO_Configuration(void);
void EXTI_Configuration(void);
void EXTI0_IRQHandler(void);
void TIM2_Configuration(void);

int main(){
	
		//Calling the needed functions definitons
		GPIO_Configuration();
    	TIM2_Configuration();
		EXTI_Configuration(); 
		
		buton1=GPIO_ReadInputDataBit(GPIOA,BUTTON_UP);
		buton2=GPIO_ReadInputDataBit(GPIOA,BUTTON_DOWN);


		
	while(1){   
			switch(led_state){
				case 0:
				Brightness=Low;
				break;

				case 1:
				Brightness=Medium;
				break;

				case 2:
				Brightness=High;
				break;

				case 3:
				Brightness=Full;
				break;

			}
	

    switch (durum)
    {
        case 0:
            TIM2->CCR4=Brightness; 
			TIM2->CCR2=0;
			TIM2->CCR3=0;
        break;

        case 1:
            TIM2->CCR4=0; 
			TIM2->CCR2=Brightness;
			TIM2->CCR3=0;
        break;

        case 2:
            TIM2->CCR4=0; 
			TIM2->CCR2=0;
			TIM2->CCR3=Brightness;
        break;

        case 3:
			TIM2->CCR4=0; 
			TIM2->CCR2=0;
			TIM2->CCR3=0;
        break;
				
		default:
		break;
    }
}

}
void GPIO_Configuration (void){
	//Typing function definitons
	GPIO_InitTypeDef GPIO_InitStructure;
	//setting clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);

  	/*Led structures (A3~TIM2_CH4| A1~TIM2_CH2| A2~TIM2_CH3)*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Pin = RED_LED|YELLOW_LED|GREEN_LED;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//Button structure (A0) 
	GPIO_InitStructure.GPIO_Pin = BUTTON;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig (GPIO_PortSourceGPIOA, GPIO_PinSource0);

	//Button structure (A10) 
	GPIO_InitStructure.GPIO_Pin = BUTTON_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig (GPIO_PortSourceGPIOA, GPIO_PinSource5);

	//Button structure (A11) 
	GPIO_InitStructure.GPIO_Pin = BUTTON_DOWN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig (GPIO_PortSourceGPIOA, GPIO_PinSource10);
	
	
}

void EXTI_Configuration (void) {
	
	//Typing function definitons
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	EXTI_DeInit();
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	//for interrupt
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


}

void TIM2_Configuration(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	/*Clock settings*/
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Prescaler = 9;
	TIM_TimeBaseInitStruct.TIM_Period = 35999;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
		
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High; 
  	TIM_OCInitStructure.TIM_Pulse = 0;
	
  	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_ITConfig(TIM2, TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	
}
	
void EXTI0_IRQHandler(void){
	
	if(EXTI_GetITStatus(EXTI_Line0) !=RESET){
        if(durum>=3){
            durum=0;
        }
        else{
            durum++;
        }

    }
    EXTI_ClearITPendingBit(EXTI_Line0);}
void EXTI9_5_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line5) !=RESET){
        if(led_state>=3){
            led_state=0;
        }
        else{
            led_state++;
        }

    }
    EXTI_ClearITPendingBit(EXTI_Line5);
	}
void EXTI15_10_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line10) !=RESET){
        if(led_state<=0){
            led_state=3;
        }
        else{
            led_state--;
        }

    }
    EXTI_ClearITPendingBit(EXTI_Line10);
 
 
 
 
 }