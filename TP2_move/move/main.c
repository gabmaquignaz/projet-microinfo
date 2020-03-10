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

//angle in °, rot_speed in °/s
void turn (float angle, float rot_speed){

	float lin_dist = (PI/180)*angle*WHEEL_DISTANCE/2;
	float lin_speed = (PI/180)*rot_speed*WHEEL_DISTANCE/2;

	motor_set_position(lin_dist, -lin_dist, lin_speed, lin_speed);
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

    //triangle pattern
    while (1) {
    		turn(120,100);
    	    move_straight(5,10);
    }
}

