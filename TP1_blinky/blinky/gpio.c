#include <stm32f407xx.h>
#include <gpio.h>
#include <main.h>

void gpio_config_input(GPIO_TypeDef *port, unsigned int pin)
{
    // Input mode : MODERy = 00
    port->MODER &= ~(3 << (pin * 2));

    // Input type pull-down : PUPDRy = 10
    port->PUPDR &= ~(1 << (pin * 2));
}

void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin)
{
    // Output type open-drain : OTy = 1
    port->OTYPER |= (1 << pin);

    // Output data low : ODRy = 0
    port->ODR &= ~(1 << pin);

    // Floating, no pull-up/down : PUPDRy = 00
    port->PUPDR &= ~(3 << (pin * 2));

    // Output speed highest : OSPEEDRy = 11
    port->OSPEEDR |= (3 << (pin * 2));

    // Output mode : MODERy = 01
    port->MODER = (port->MODER & ~(3 << (pin * 2))) | (1 << (pin * 2));
}

void gpio_config_output_push_pull(GPIO_TypeDef *port, unsigned int pin)
{
    // Output type push pull : OTy = 0
    port->OTYPER &= ~(1 << pin);

    // Output data low : ODRy = 0
    port->ODR &= ~(1 << pin);

    // Floating, no pull-up/down : PUPDRy = 00
    port->PUPDR &= ~(3 << (pin * 2));

    // Output speed highest : OSPEEDRy = 11
    port->OSPEEDR |= (3 << (pin * 2));

    // Output mode : MODERy = 01
    port->MODER = (port->MODER & ~(3 << (pin * 2))) | (1 << (pin * 2));
}

void gpio_set(GPIO_TypeDef *port, unsigned int pin)
{
    port->BSRR = (1 << pin);
}

void gpio_clear(GPIO_TypeDef *port, unsigned int pin)
{
    port->BSRR = (1 << (pin + 16));
}

void gpio_toggle(GPIO_TypeDef *port, unsigned int pin)
{
    if (port->ODR & (1<<pin)) {
        gpio_clear(port, pin);
    } else {
        gpio_set(port, pin);
    }
}

void wait_500_ms(void){
	int wait = 2300;
	for(int i = 0; i < wait; i++){
		for(int j = 0; j < wait; j++){
			;
		}
	}
}

void seq_a (void){
	gpio_set(LED1);
	wait_500_ms();
	gpio_set(LED3);
	wait_500_ms();
	gpio_set(LED5);
	wait_500_ms();
	gpio_set(LED7);
	wait_500_ms();
	gpio_clear(LED1);
	wait_500_ms();
	gpio_clear(LED3);
	wait_500_ms();
	gpio_clear(LED5);
	wait_500_ms();
	gpio_clear(LED7);
	wait_500_ms();
}

void blink (GPIO_TypeDef *port, unsigned int pin){
	gpio_set(port, pin);
	wait_500_ms();
	gpio_clear(port, pin);
	wait_500_ms();
}

void seq_b (void){
	blink(LED1);
	blink(LED3);
	blink(LED5);
	blink(LED7);
}
