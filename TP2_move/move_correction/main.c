#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <timer.h>
#include <motor.h>
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


int main(void)
{
    SystemClock_Config();

    // Enable GPIOD peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;

    //config the GPIO for PWM (alternate function 2)
    gpio_config_output_af_pushpull(FRONT_LED, 2);
    //config the timer with a given duty_cycle
    timer4_PWM_start(0);

    // BODY_LED init
    gpio_config_output_pushpull(BODY_LED);

    init_selector();
    motor_init();

    //waits before moving to let us position the robot before it moves
    delay(SystemCoreClock/8);

    //move 20cm forward at 5cm/s 
    motor_set_position(20, 20, 5, 5);
    while(motor_position_reached() != POSITION_REACHED);
    //clockwise rotation of 180° 
    motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, -5, 5);
    while(motor_position_reached() != POSITION_REACHED);
    //move 20cm forward at 5cm/s 
    motor_set_position(20, 20, 5, 5);
    while(motor_position_reached() != POSITION_REACHED);
    //counterclockwise rotation of 180°
    motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, 5, -5);
    while(motor_position_reached() != POSITION_REACHED);

    while (1) {
        delay(SystemCoreClock/32);
        gpio_toggle(BODY_LED);
        timer4_set_duty_cycle(get_selector() / (float)MAX_VALUE_SELECTOR);
    }
}

