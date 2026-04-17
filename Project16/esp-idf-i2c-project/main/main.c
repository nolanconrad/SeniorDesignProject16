#include "i2c_with_tmp117.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TMP117_TEST";

void app_main(void)
{
    tmp117_init();

    while (1) {
        float t1 = tmp117_read_temperature();          // sensor at 0x48
        float t2 = tmp117_read_temperature_device2();  // sensor at 0x49
        ESP_LOGI(TAG, "TMP117: t1=%.2f C, t2=%.2f C", t1, t2);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}