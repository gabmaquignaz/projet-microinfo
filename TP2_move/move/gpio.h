#ifndef GPIO_H
#define GPIO_H

#include <stm32f407xx.h>
#include <stdbool.h>

#define SELEC0		GPIOC, 13
#define SELEC1		GPIOC, 14
#define SELEC2		GPIOC, 15
#define SELEC3		GPIOD, 4

void gpio_config_input_pd(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_output_pushpull(GPIO_TypeDef *port, unsigned int pin);
void gpio_set(GPIO_TypeDef *port, unsigned int pin);
void gpio_clear(GPIO_TypeDef *port, unsigned int pin);
void gpio_toggle(GPIO_TypeDef *port, unsigned int pin);
bool gpio_read(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_output_af_pushpull(GPIO_TypeDef *port, unsigned int pin);
bool selec_read_pin(GPIO_TypeDef *port, unsigned int pin);
int selector_read_all (void);

#endif /* GPIO_H */
