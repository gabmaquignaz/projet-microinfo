#include <stm32f4xx.h>
#include <gpio.h>
#include <main.h>
#include <stdbool.h>

#define TIMER_CLOCK         84000000    // APB1 clock

#define PRESCALER_TIM6     840         // timer frequency: 86,957kHz
#define COUNTER_MAX_TIM6    70     // timer max counter -> 90Hz

#define PRESCALER_TIM7      840       // timer frequency: 100kHz
#define COUNTER_MAX_TIM7     70

#define PRESCALER_TIM4      966     // timer frequency: 100kHz
#define COUNTER_MAX_TIM4    966

#define DUTY_CYCLE			50	// COUNTER_MAX/2 -> 50% duty cycle
#define MODE1				0b110
#define CC3P					9
#define CC3E					8

void timer6_start(void)
{
    // Enable TIM6 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Enable TIM6 interrupt vector
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    // Configure TIM6
    TIM6->PSC = PRESCALER_TIM6 - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM6->ARR = COUNTER_MAX_TIM6 - 1;	// Note: timer reload takes 1 cycle, thus -1
    TIM6->DIER |= TIM_DIER_UIE;  	// Enable update interrupt
    TIM6->CR1 |= TIM_CR1_CEN;    	// Enable timer
}

void timer7_start(void)
{
    // Enable TIM7 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // Enable TIM7 interrupt vector
    NVIC_EnableIRQ(TIM7_IRQn);

    // Configure TIM7
    TIM7->PSC = PRESCALER_TIM7 - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM7->ARR = COUNTER_MAX_TIM7 - 1;	// Note: timer reload takes 1 cycle, thus -1
    TIM7->DIER |= TIM_DIER_UIE;  	// Enable update interrupt
    TIM7->CR1 |= TIM_CR1_CEN;    	// Enable timer
}

void PWM_config(void){
	// Enable TIM4 clock
	    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	    // Enable TIM4 interrupt vector:
	    // Configure TIM4
	    TIM4->PSC = PRESCALER_TIM4 - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
	    TIM4->ARR = COUNTER_MAX_TIM4 - 1;    // Note: timer reload takes 1 cycle, thus -1
	    //TIM7->DIER |= TIM_DIER_UIE;          // Enable update interrupt
	    TIM4->CR1 |= TIM_CR1_CEN;            // Enable timer

	// PWM Param
		//select duty cycle
		TIM4->CCR3 = DUTY_CYCLE;

		//select PMW Mode 1
		TIM4->CCMR2 &=  ~(7 << 4);
		TIM4->CCMR2 |= (6 << 4);

		//select mode = output and polarity for Capture/compare
		//CC3NP kept clear

		//CC3P = 0 (active high)
		TIM4->CCER &= ~(1<< CC3P);

		//CC3E = 1 (activate output)
		TIM4->CCER |= (1<< CC3E);
}

/*
*   Commented because used for the motors
*/

// // Timer 7 Interrupt Service Routine
// void TIM7_IRQHandler(void)
// {
	/*
	*
	*   BEWARE !!
	*   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
	*
	*   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
	*   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
	*
	*   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
	*
	*/

//     /* do something ... */
//     gpio_toggle(BODY_LED);

//     // Clear interrupt flag
//     TIM7->SR &= ~TIM_SR_UIF;
//     TIM7->SR;	// Read back in order to ensure the effective IF clearing
// }
