#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_types.h" //header file for interrupts
#include "i2c_with_tmp117.h"

void app_main(void)
{
    // Initialize I2C and temperature sensors
    tmp117_init();

    while(1) {
        // auto start-up when power is applied
        //do a system check to confirm there are no faults 
            //quickly run pump & confirm no faults 
            //cycle through all LED's and confirm no faults
            //activate bluetooth module if needed and confirm no faults

        //automatic mode
            //read sensor data and determine if pump needs to be turned on
                float temp1 = tmp117_read_temperature();
                float temp2 = tmp117_read_temperature_device2();
                //activate pump and respective LED's if needed
            //await interrupts
                // manual mode w/ respective LED changes
                // emergencies/clog
                // reset button pressed

            //if manual mode is launched
                //await interrupts
                    // pump increase/decrease
                    // pump turn on/off
                    // emergencies/clog
                    // reset button pressed

            //if emergency/clog is detected
                //turn off pump    
                //flash LED's
                // notify user via bluetooth if needed

            //if reset button is pressed
                //turn off pump
                //return to automatic mode

        //update battery LED's
            //check voltage via buck converter information and update LED's accordingly
                //60-100% = 3 LED's
                //30-60% = 2 LED's
                //10-30% = 1 LED
                //0-10% = 1 LED flashing
        // auto start-up when power is applied
            //do a system check to confirm there are no faults
                //quickly run pump & confirm no faults 
                //cycle through all LED's and confirm no faults
                //activate bluetooth module if needed and confirm no faults

        //automatic mode
            //read sensor data and determine if pump needs to be turned on
                //activate pump and respective LED's if needed
            //await interrupts
                // manual mode w/ respective LED changes
                // emergencies/clog
                // reset button pressed

            //if manual mode is launched
                //await interrupts
                    // pump increase/decrease
                    // pump turn on/off
                    // emergencies/clog
                    // reset button pressed

            //if emergency/clog is detected
                //turn off pump    
                //flash LED's
                // notify user via bluetooth if needed

            //if reset button is pressed
                //turn off pump
                //return to automatic mode

        //update battery LED's
            //check voltage via buck converter information and update LED's accordingly
                //60-100% = 3 LED's
                //30-60% = 2 LED's
                //10-30% = 1 LED
                //0-10% = 1 LED flashing
    }
}
            //await interrupts
                // manual mode w/ respective LED changes
                // emergencies/clog
                // reset button pressed

            //if manual mode is launched
                //await interrupts
                    // pump increase/decrease
                    // pump turn on/off
                    // emergencies/clog
                    // reset button pressed

            //if emergency/clog is detected
                //turn off pump    
                //flash LED's
                // notify user via bluetooth if needed

            //if reset button is pressed
                //turn off pump
                //return to automatic mode

        //update battery LED's
            //check voltage via buck converter information and update LED's accordingly
                //60-100% = 3 LED's
                //30-60% = 2 LED's
                //10-30% = 1 LED
                //0-10% = 1 LED flashing
        // auto start-up when power is applied
            //do a system check to confirm there are no faults
                //quickly run pump & confirm no faults 
                //cycle through all LED's and confirm no faults
                //activate bluetooth module if needed and confirm no faults

        //automatic mode
            //read sensor data and determine if pump needs to be turned on
                //activate pump and respective LED's if needed
            //await interrupts
                // manual mode w/ respective LED changes
                // emergencies/clog
                // reset button pressed

            //if manual mode is launched
                //await interrupts
                    // pump increase/decrease
                    // pump turn on/off
                    // emergencies/clog
                    // reset button pressed

            //if emergency/clog is detected
                //turn off pump    
                //flash LED's
                // notify user via bluetooth if needed

            //if reset button is pressed
                //turn off pump
                //return to automatic mode

        //update battery LED's
            //check voltage via buck converter information and update LED's accordingly
                //60-100% = 3 LED's
                //30-60% = 2 LED's
                //10-30% = 1 LED
                //0-10% = 1 LED flashing
    }
}

