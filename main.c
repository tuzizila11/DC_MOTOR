#include "stm32f4xx.h"                  // Device header

#define PRESCALER 				16-1								
#define COUNTER						16000
#define COUNTER_FREQ 	  	SystemCoreClock / (PRESCALER + 1)

static uint8_t 		S1 			= 1;
static uint16_t 	period 	= 0;
static float 			freq, y 		= 0;

void sysConfig(void);
void encoderInit(void);
void buttonInit(void);
void motorInit(void);
void pwmInputModeInit(void);
void motorSpeed(void);

//Change direction of motor
void EXTI0_IRQHandler(void)																																										
{
	if (EXTI->PR & EXTI_PR_PR0){
		
		if (S1 == 0) {
			S1 = 1;
		}
		else {
			S1 = 0;
		}
		
		switch (S1)
    {
    	case 1:
				GPIOD->ODR 		 &= ~(GPIO_ODR_OD13);
				GPIOD->ODR 		 |= GPIO_ODR_OD14;			
    		break;
    	default:
				GPIOD->ODR 		 &= ~(GPIO_ODR_OD14);
				GPIOD->ODR 		 |= GPIO_ODR_OD13;
    		break;
    }
		
		EXTI->PR |= EXTI_PR_PR0;
	}
}

void TIM3_IRQHandler(void)
{		
	if (TIM3->DIER & TIM_DIER_CC1IE) {																																												//Check if Capture/compare 1 interrupt flag is enable
		if (TIM3->SR & TIM_SR_CC1IF) {																																													//Check for Capture/compare 1 interrupt flag is set
			TIM3->SR &= ~(TIM_SR_CC1IF);																																													// clear interrupt status
			y = COUNTER_FREQ;
			motorSpeed();
		}
	}
	
}

/*
void TIM4_IRQHandler(void)
{		
	if (TIM4->DIER & 0x01) {
		if (TIM4->SR & 0x01) {
			TIM4->SR &= ~(1U << 0);																																													// clear interrupt status
		}
	}
	
}*/

int main(void)
{		
	sysConfig();
	motorInit();
	pwmInputModeInit();

	while(1)
	{
		/*Do nothing*/
	}	
}

void sysConfig(void)
{
	//Peripheral Clock Enable
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;																		// Enable PORT A for USER BUTTON, PORT B for for Encoder BUTTON, PORT C for ENCODER and PORT D for BOARD LED
	RCC->APB1ENR 	|= RCC_APB1ENR_TIM3EN  | RCC_APB1ENR_TIM4EN;																													// Enable TIM3 and TIM4
	RCC->APB2ENR 	|= RCC_APB2ENR_SYSCFGEN;
	
	//Peripheral Initialization
	GPIOC->MODER 	|= GPIO_MODER_MODE6_1  | GPIO_MODER_MODE7_1;																													// Mode set to ALT FUNC 
	GPIOC->AFR[0]	|= GPIO_AFRL_AFRL6_1   | GPIO_AFRL_AFRL7_1;																														// Alt func. AF2(TIM3) for PC6 and PC7
	GPIOC->PUPDR	|= GPIO_PUPDR_PUPD6_0  | GPIO_PUPDR_PUPD7_0;																													// Pullup enable PC6 and PC7		
	GPIOD->MODER 	|= GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_0 |GPIO_MODER_MODE14_0 |GPIO_MODER_MODE15_0;								// ENABLE PD12 as Alt Func Mode and PD13,14,15 as Digital OUTPUT
	GPIOD->AFR[1] |= GPIO_AFRH_AFRH4_1;																																									// Alt Func TIM4_CH1
}

void encoderInit(void){
	//Encode timer setup
	TIM3->ARR 		 = 100 - 1;
	TIM3->CCMR1		|= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;																															 	// Capture/Compare 1 selection. IC1 is mapped on TI1 and TI2
	TIM3->CCER		&= ~(TIM_CCER_CC1P 	| TIM_CCER_CC2P);																																 	// Noninverted input signal
	TIM3->SMCR 		|= TIM_SMCR_SMS_0; 																																									 	// Encoder mode select				
	TIM3->CR1			|= TIM_CR1_CEN  		| TIM_CR1_ARPE;																																		// Enable TIM3
}
 
void buttonInit(void){
	//External Interupt for BLUE USER Button
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PA;

	EXTI->IMR		 |= EXTI_IMR_MR0;

	//EXTI->RTSR	 |= EXTI_RTSR_TR1;

	EXTI->FTSR	 |= EXTI_FTSR_TR0;																																											// Trigger on falling edge

	NVIC_SetPriority(EXTI0_IRQn,5);

	NVIC_EnableIRQ(EXTI0_IRQn);
}

void motorInit(void){
	//Motor PWM setup
	TIM4->CCER 		|= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;																			// Timer enable CH1, PB6
	TIM4->PSC		   = 160 - 1;
	TIM4->ARR 		 = 100;
	TIM4->CCR1 		 = 50;																																																// PWM CH
	TIM4->CCMR1 	|= 0x0068;
	
	TIM4->EGR  		|= 1;
	TIM4->DIER 		|= TIM_DIER_UIE;
	
	//GPIOD->ODR 		 = GPIO_ODR_OD15 | GPIO_ODR_OD14;																																			// Direction CH and Standby 

	//NVIC_SetPriority(TIM4_IRQn,6);																																											//TIM4 Interupt
	//NVIC_EnableIRQ(TIM4_IRQn);
	
	TIM4->CR1			|= TIM_CR1_CEN;																																												// ENABLE TIM4
}

void pwmInputModeInit(void){
	TIM3->PSC			 = PRESCALER;																																													
	TIM3->ARR 	   = COUNTER;																																															
	TIM3->CCMR1 	|= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1; 																															// TI1 selected for TIM3_CCR1 and TIM3_CCR2
	TIM3->SMCR		|= TIM_SMCR_TS_0 		| TIM_SMCR_TS_2 | TIM_SMCR_SMS_2;																									// Filtered Timer Input 1 (TI1FP1),  Slave mode selection Reset Mode
	TIM3->CCER		|= TIM_CCER_CC2P 		| TIM_CCER_CC1E | TIM_CCER_CC2E;																									// Circuit is sensitive to TIxFP1 falling edge 
	TIM3->CR1 		|= TIM_CR1_CEN;																																												// ENABLE TIM3
	TIM3->EGR 		|= TIM_EGR_CC1G;																																											// Capture/compare 1 generation
	TIM3->DIER		|= TIM_DIER_CC1IE;																																										// Capture/compare 1 Interupt enable
	
	NVIC_SetPriority(TIM3_IRQn,4);																																											//TIM3 Interupt
	NVIC_EnableIRQ(TIM3_IRQn);
}

void motorSpeed(void){
	period 			= TIM3->CCR1;
	freq 				= (float)(COUNTER_FREQ / period);
	GPIOD->ODR ^= GPIO_ODR_OD13;
}

