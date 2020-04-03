
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include"sensors/VL53L0X/VL53L0X.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <vision.h>



#define MEAN_RANGE 		50

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();

    //start ToF with high accuracy, also starts I2C
    VL53L0X_start();
    VL53L0X_configAccuracy(VL53L0X_ADDR, VL53L0X_HIGH_ACCURACY);

    //starts the camera
    dcmi_start();
	po8030_start();
	process_image_start();

    uint16_t sum = 0;
    float mean = 0;

    //Infinite loop
    while (1) {
    	 /**
    		for(uint8_t i = 0; i < MEAN_RANGE; i++){
    			uint16_t curent = VL53L0X_get_dist_mm();
    			sum+= curent;
    			chprintf((BaseSequentialStream *)&SD3, "%d, ", curent);
    			chThdSleepMilliseconds(100);
    		}
    		mean = (float)sum/MEAN_RANGE;
    	 	chprintf((BaseSequentialStream *)&SD3, "-> Dist = %.1f mm\n", mean);

    	 	sum = 0;
    	 	mean = 0;
    	 	*/
        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
