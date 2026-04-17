#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"

#define PUMP_PIN GPIO_NUM_8

static int pump_mode = 0; //this makes it never change and is stored in global memory 

void pump_Operation_init(void) {
    // Configure pump pin as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Set up PWM for speed control
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,  // 5kHz frequency
    };
    ledc_timer_config(&timer_conf);
    
    // Set initial mode to stopped
    pump_mode = 0;
}

void pump_Set_Mode(int mode) {
    pump_mode = mode;
}

//dont need this right now but can use it to call from other funcitons later
/*  
    int pump_Get_Mode(void) {
        return pump_mode;
    }
*/

void man_Pump_On(int intensity) {
    (void)intensity;
    gpio_set_level(PUMP_PIN, 1); //change this
    // Set PWM duty cycle based on intensity (0-255)
}

void automatic_Pump_On(void) {
    gpio_set_level(PUMP_PIN, 1); //change this
    // TODO: implement sensor-driven pump intensity logic.
}

void pump_on(void) {
    man_Pump_On(255);
}

void pump_off(void) {
    gpio_set_level(PUMP_PIN, 0); //this needs to just turn off the pump for good
}


void pump_Operation(void) {
    switch(pump_mode) {
        case 1:
            //automatic mode
            automatic_Pump_On();
            break; 
        case 2:
            //manual mode low
            man_Pump_On(85);
            break;
        case 3:
            //manual mode medium
            man_Pump_On(170);
            break;
        case 4:
            //manual mode high
            man_Pump_On(255);
            break;
        case 5:
            //stop pump
            pump_off();
            break;
        default:
            pump_off();
            break;
    }
}