//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)
#define myTIM3_PRESCALER ((uint16_t)47999)

void myGPIOA_Init(void);
void myLCD_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myADC_DAC_Init(void);
void write_LCD(void);


// Declare/initialize your global variables here...
// NOTE: You'll need at least one global variable
// (say, timerTriggered = 0 or 1) to indicate
// whether TIM2 has started counting or not.
int timerTriggered = 0;
int resistance = 0;
float counter_value = 0;
float freq = 0;
int r_thou = 0;
int r_hun = 0;
int r_ten = 0;
int	r_one = 0;
int f_thou = 0;
int f_hun = 0;
int f_ten = 0;
int	f_one = 0;

int main(int argc, char* argv[])
{

	trace_printf("This is the project code...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myLCD_Init();		/* Initialize LCD */
	myADC_DAC_Init();	/* Initialize ADC and DAC */
	trace_printf("All initialized\n");
	while (1)
	{
		while((ADC1->ISR & 0x1) == 0);	//Wait for ADC to be ready
		while((ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC);	// Wait for channel conversion to complete
		//Read ADC_DR. This clears EOC
		resistance = ADC1->DR;	//Read ADC data register. This clears EOC bit
		DAC->DHR12R1 = resistance;	//resistance;//ADC Data register value sent to DAC
		resistance = (resistance * 5000)/4095;	//Scale for a more accurate resistance reading

		//PARSE RESISTANCE
		r_thou = resistance/1000;
		r_hun = (resistance - r_thou*1000)/100;
		r_ten = (resistance - r_thou*1000 - r_hun*100)/10;
		r_one = (resistance - r_thou*1000 - r_hun*100 - r_ten*10);
		//ADD 48 to each for ASCII
		r_thou = r_thou + 48;
		r_hun = r_hun + 48;
		r_ten = r_ten + 48;
		r_one = r_one + 48;
		//r_one = rand()%(10) + 48;

		//PARSE FREQUENCY
		f_thou = freq/1000;
		f_hun = (freq - f_thou*1000)/100;
		f_ten = (freq - f_thou*1000 - f_hun*100)/10;
		f_one = (freq - f_thou*1000 - f_hun*100 - f_ten*10);
		//ADD 48 to each for ASCII
		f_thou = f_thou + 48;
		f_hun = f_hun + 48;
		f_ten = f_ten + 48;
		f_one = f_one + 48;
		write_LCD();
	}

	return 0;

}

void write_LCD ()
{
	//**TOP LINE**
	GPIOB->ODR = 0x8000;	//Sets address to 00 (top line)
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	//Now passing top line
	GPIOB->ODR = 0x4620;	//Sends F
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);	//Wait for LCD response
	GPIOB->ODR ^= GPIO_ODR_4;	//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0)	//Wait for LCD response

	GPIOB->ODR = 0x3A20;	//Sends :
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);	//Wait for LCD response
	GPIOB->ODR ^= GPIO_ODR_4;	//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);	//Wait for LCD response

	GPIOB->ODR = 0x0020 | (f_thou<<8);	//Sends First digit
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0x0020 | (f_hun<<8);	//Sends Second digit
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0x0020 | (f_ten<<8);	//Sends third digit
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0x0020 | (f_one<<8);	//Sends fourth digit
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0x4820;	//Sends H
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != Bit_SET);	//Wait for "done" to be raised
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != Bit_RESET);	//Wait for "done" to be lowered

	GPIOB->ODR = 0x7A20; 	//Sends z
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != Bit_SET);	//Wait for "done" to be raised
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != Bit_RESET);	//Wait for "done" to be lowered


	//**BOTTOM LINE**

	GPIOB->ODR = 0xC000;	//Sets address to 40 (bottom line)
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	//Now passing bottom line
	GPIOB->ODR = 0x5220;	//246;	//Sends R
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0)

	GPIOB->ODR = 0x3A20;	//Sends :
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0x0020 | (r_thou<<8);	//Sends First digit
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0x0020 | (r_hun<<8);	//Sends Second digit
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0x0020 | (r_ten<<8);	//Sends third digit
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0x0020 | (r_one<<8);	//Sends fourth digit
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0x4F20;	//Sends H
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != Bit_SET);	//Wait for "done" to be raised
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != Bit_RESET);	//Wait for "done" to be lowered

	GPIOB->ODR = 0x6820; 	//Sends z
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != Bit_SET);	//Wait for "done" to be raised
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) != Bit_RESET);	//Wait for "done" to be lowered
}

void myGPIOA_Init()
{
	/* Enable clock for GPIO peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	// Clock enables port A
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	// Clock enables port B
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;	// Clock enables port C

	/* Configure PC1 analog input, PA1 as input, PA4 as analog output */
	// Relevant register: GPIOA->MODER
	GPIOC->MODER |= GPIO_MODER_MODER1;//PC1 analog input
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);//PA1 input
	GPIOA->MODER |= GPIO_MODER_MODER4;//PA4 analog output

	/*Configure PB4-PB15*/
	GPIOB->MODER &= ~(GPIO_MODER_MODER7);//Set PB7 as input first
	GPIOB->MODER |= (GPIO_MODER_MODER4_0|
					GPIO_MODER_MODER5_0|
					GPIO_MODER_MODER6_0|
					GPIO_MODER_MODER8_0|
					GPIO_MODER_MODER9_0|
					GPIO_MODER_MODER10_0|
					GPIO_MODER_MODER11_0|
					GPIO_MODER_MODER12_0|
					GPIO_MODER_MODER13_0|
					GPIO_MODER_MODER14_0|
					GPIO_MODER_MODER15_0);
	//Set PB4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15 as outputs
	// 8 to 15 are data


	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}


void myTIM2_Init() //unchanged from part 1
{
	////trace_printf("Starting TIM2 init\n");
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);//[7] = 1, [4] = 0,
	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;
	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = TIM_EGR_UG;
	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);
	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);
	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}

void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;	//Line 4414 header file
	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;	//is the interrupt coming in on line 1???
	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;		//unmasked on line 1...If its on line 1??
	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);
	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void myLCD_Init()
{
	GPIOB->ODR = 0b0011100000000000;	//DL = 1, N = 1, F = 0
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0b0000110000000000;	//D = 1, C = 0, B = 0
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0b0000011000000000;	//I/D = 1, S = 0
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);

	GPIOB->ODR = 0b0000000100000000;	//Clears display
	GPIOB->ODR |= GPIO_ODR_4;	//Raise "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	GPIOB->ODR ^= GPIO_ODR_4;		//Lower "Enable" bit 4
	while((GPIOB->IDR & GPIO_IDR_7) != 0);
}

void myADC_DAC_Init()
{
	////trace_printf("Starting ADC init\n");
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // Enable ADC clock. Interfacex slide 19. Sets bit 9 to 1
	RCC->APB1ENR |= RCC_APB1ENR_DACEN; // Enable DAC clock. Interfacex slide 19. Sets bit 29 to 1 //DAC_CR_EN1;	//Enable DAC... What is the DAC register???

	ADC1->CFGR1 &= ~(ADC_CFGR1_RES); // Sets data resolution to 12. Interfacex slide 22
	//Data is already right aligned
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD; // Sets Overrun Management mode to overwrite ADC_DR contents
	ADC1->CFGR1 |= ADC_CFGR1_CONT; // Sets Continuous conversion mode

	ADC1->CR |= ADC_CR_ADEN; // Sets ADC enable bit to 1. Interfacex slide 20, pg 231 ref manual
	ADC1->SMPR |= ADC_SMPR_SMP; // Sets Sampling time to 239.5 ADC clock cycles. Can only be done when ADSTART = 0
	ADC1->CHSELR |= ADC_CHSELR_CHSEL11; // Set bit 11 to 1 to choose channel 11 for ADC conversion. Interfacex slide 21

	while((ADC1->ISR & ADC_ISR_ADRDY) != 1);//(ADC1->ISR == ADC_ISR_ADRDY);	//Wait until ADC ready flag is 1

	ADC1->CR |= ADC_CR_ADSTART; // Starts ADC. Interfacex slide 20, pg 231 ref manual
	DAC->CR = DAC_CR_EN1;	// Enabled DAC
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");
		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);
		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;//TIM2_CR1_CEN
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//
		// 1. If this is the first edge:
		if(timerTriggered == 0){
			//	- Clear count register (TIM2->CNT).
			TIM2->CNT = 0;
			//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;
			timerTriggered = 1;
		}
		//    Else (this is the second edge):
		else{
			//	- Stop timer (TIM2->CR1).
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			/////////////////////////////////EXTI->IMR &= ~(EXTI_IMR_MR1);
			//	- Read out count register (TIM2->CNT).
			counter_value = TIM2->CNT;
			//	- Calculate signal period and frequency.
			freq = 48000000 / counter_value;
			timerTriggered = 0;
			//	- Print calculated values to the console.
			//	  NOTE: Function trace_printf does not work
			//	  with floating-point numbers: you must use
			//	  "unsigned int" type to print your signal
			//	  period and frequency.
		}
		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.
		//
		EXTI->IMR |= EXTI_IMR_MR1;
		EXTI->PR = EXTI_PR_PR1;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

