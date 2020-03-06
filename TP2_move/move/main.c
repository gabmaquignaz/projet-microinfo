#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <motor.h>
#include <timer.h>
#include <selector.h>

#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

// Init function required by __libc_init_array
void _init(void) {}

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}

void move_straight(float dist, float speed){
	motor_set_position(dist,dist,speed,speed);

}


int main(void)
{
    SystemClock_Config();

    /**
    // Enable GPIOD and GPIOE peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;

    //set moder to AF
    gpio_config_output_af_pushpull(FRONT_LED);

    //config PWM
    PWM_config();

    **/

    motor_init();
    move_straight(10,10);

    while (1) {

    }
}

