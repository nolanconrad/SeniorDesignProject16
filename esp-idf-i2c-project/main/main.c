#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_types.h" //header file for interrupts

void app_main(void)
{
    // auto start-up when power is applied

    //do a system check to confirm there are no faults 

    //begin automatic mode
        //await interrupts
            // manual mode
            // pump turn on/off 
            // emergencies/clog
            // reset button pressed

    //update battery LED's


}

