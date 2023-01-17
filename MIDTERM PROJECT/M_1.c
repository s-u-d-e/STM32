#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"

#define RED_LED GPIO_Pin_1
#define BUTTON GPIO_Pin_3 //PA3

int durum=0,butonstate,buton;
int butontime=0,presstime=0;

double Brightness_I=0;
double Brightness_D=0;

int shortbutonpress=0;
int longbutonpress=0;

double count;

static __IO uint16_t MS_count;
static __IO uint16_t MS1_count;
static __IO uint16_t buttonCounter;

void GPIO_Configuration(void);
void EXTI3_Configuration(void);
void TIM2_Configuration(void);
void TIM3_Configuration(void); 
void EXTI3_IRQHandler(void);

int main(){
    GPIO_Configuration();
    TIM2_Configuration();
		TIM3_Configuration();
    EXTI3_Configuration(); 
    SysTick_Config(SystemCoreClock/1000);
		MS_count=0;
	
		while(1){   
		buton=GPIO_ReadInputDataBit(GPIOA,BUTTON); 
			
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
            TIM2->CCR2=0;
					if(butonstate==1){
						if((shortbutonpress)%2==0){
						durum=1;}} // if buton is pressed short LED is turning on
					if(butonstate==2){
						if((longbutonpress)%2==0){
							durum=2;}} // in case 1 if buton is long pressed first time LED's brightness is increasing
					if(butonstate==2){
						if((longbutonpress)%2==1) {
							durum=3;}} // in case 1 if buton is long pressed second time LED's brightness is decreasing
        break;
				case 1: //LED on
						TIM2->CCR2=1480;
					if(butonstate==1){
						if(shortbutonpress%2==1){
							durum=0;}} // turn off
					if(butonstate==3){
							durum=4;}		//double click
					break;
				case 2: //LED's brightness is increasing
						TIM2->CCR2=Brightness_I ;
					if(butonstate==1){
						if((shortbutonpress)%2==1){
							durum=0;}} // turn off
					
				break;
				case 3: //LED's brightness is decreasing
						TIM2->CCR2=Brightness_D;
					if(butonstate==1){
						if((shortbutonpress)%2==1){ 
							durum=0;}} //turn off
				break;
				case 4: //LED is blinking
					TIM2->CCR2=7200;
					for(MS_count=0;MS_count<0.5;MS_count++){}
					TIM2->CCR2=0;
					for(MS_count=0;MS_count<0.5;MS_count++){}
					 if(butonstate==3){
						durum=2;}// constant turn on
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
}
void TIM2_Configuration(void){
	// Init struct
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	// Enable clock for TIM2
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	// TIM3 initialization
	TIM_TimeBaseInitStruct.TIM_Prescaler = 9;
	TIM_TimeBaseInitStruct.TIM_Period = 35999;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity= TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	// Enable TIM2 channel 2
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	// Start TIM2
	TIM_Cmd(TIM2, ENABLE);
	
}
void TIM3_Configuration(){
	// Init struct
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	// Enable clock for TIM3
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
void EXTI3_Configuration (void){
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
	// Nested vectored interrupt settings
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
			longbutonpress++; //to measure the number oh long pressing
			}
		if (buton==1){
					for(count=0;count<0.2;count++){
						if (buton==1){
							butonstate=3;} //third button state
				}}
    EXTI_ClearITPendingBit(EXTI_Line3);

	}
}
void SysTick_Handler (void){   
			
		buttonCounter++; //counts while buton is pressed
	
		if(buton==1 && durum==2){ // counter for increasing the brightness
				MS_count++;
        Brightness_I=MS_count*20;
		}  
		if(buton==1&& durum==3){ // counter for decreasing the brightness
				MS1_count--;
				Brightness_D=MS1_count*20;
    }
}