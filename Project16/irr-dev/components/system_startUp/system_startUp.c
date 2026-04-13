#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_types.h" 
#include "i2c_with_tmp117.h"
#include "pump_Operation.h"

#include "driver/gpio.h" //header file for GPIO control

#define LED1 GPIO_NUM_21
#define LED2 GPIO_NUM_18
#define LED3 GPIO_NUM_17

static void led_on(gpio_num_t pin)
{
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 1);
}

static void led_off(gpio_num_t pin)
{
    gpio_set_level(pin, 0);
}


void system_startUp(void) {
    // auto start-up when power is applied
    //do a system check to confirm there are no faults 
        //quickly run pump & confirm no faults
        pump_on();
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
        pump_off();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //confirm that the pump is in proper operation

        //cycle through all LED's and confirm no faults
        led_on(LED1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        led_on(LED2);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        led_on(LED3);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        led_off(LED1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        led_off(LED2);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        led_off(LED3);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        //activate bluetooth module if needed and confirm no faults
        //interface with battery and find out battery level
}