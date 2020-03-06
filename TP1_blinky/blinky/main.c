#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <stdio.h>
#include <main.h>
#include <stdbool.h>
#include <timer.h>


// Init function required by __libc_init_array
void _init(void) {}


bool selec_read_pin(GPIO_TypeDef *port, unsigned int pin){
	bool pin_value;
	pin_value = (port->IDR) & (1 << pin);
	return pin_value;
}

int selector_read_all (void){
	int selector_position = 0;
	selector_position += selec_read_pin(SELEC0);
	selector_position += (selec_read_pin(SELEC1) << 1);
	selector_position += (selec_read_pin(SELEC2) << 2);
	selector_position += (selec_read_pin(SELEC3) << 3);
	return selector_position;
}

int main(void)
{
    SystemClock_Config();

    // Enable GPIOD peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOCEN;

    //selector init
    	gpio_config_input(GPIOC, 13);
    gpio_config_input(GPIOC, 14);
    gpio_config_input(GPIOC, 15);
    gpio_config_input(GPIOD, 4);

    // LED used init
    	gpio_config_output_opendrain(LED1);
    gpio_config_output_opendrain(LED3);
    gpio_config_output_opendrain(LED5);
    gpio_config_output_opendrain(LED7);

    //gpio_config_output_push_pull(FRONT_LED);
    //gpio_config_output_push_pull(BODY_LED);

    gpio_set(LED1);
    gpio_set(LED3);
    gpio_set(LED5);
    gpio_set(LED7);


    //enable clock
    RCC->AHB1ENR		|= RCC_APB1ENR_TIM7EN;
    NVIC_EnableIRQ(TIM7_IRQn);
    timer7_start();

   // long int sel = 0;
    while (1) {
    		//sel = selector_read_all();
    		//if(sel < 8) seq_a();
    		//else seq_b();
    }
}
