#ifndef GPIO_H
#define GPIO_H

#include <stm32f407xx.h>

void gpio_config_input(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_output_push_pull(GPIO_TypeDef *port, unsigned int pin);
void gpio_set(GPIO_TypeDef *port, unsigned int pin);
void gpio_clear(GPIO_TypeDef *port, unsigned int pin);
void gpio_toggle(GPIO_TypeDef *port, unsigned int pin);

void wait_500_ms(void);
void seq_a (void);
void seq_b (void);
void blink (GPIO_TypeDef *port, unsigned int pin);


#endif /* GPIO_H */
