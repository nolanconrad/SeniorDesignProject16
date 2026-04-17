#include "mode_manager.h"
#include "esp_log.h"

static const char *TAG = "MODE_MANAGER";

static operating_mode_t current_mode = MODE_STARTUP;
static int automatic_enabled = 0;  // Default: disabled, user must enable
static int manual_level = 1;       // 1=low, 2=medium, 3=high

void mode_manager_init(void)
{
    ESP_LOGI(TAG, "Mode manager initialized");
    current_mode = MODE_STARTUP;
    automatic_enabled = 0;  // Start with manual mode
    manual_level = 1;
}

void mode_manager_set_mode(operating_mode_t mode)
{
    const char *mode_names[] = {"STARTUP", "MANUAL_LOW", "MANUAL_MEDIUM", "MANUAL_HIGH", "AUTOMATIC", "ERROR", "COOLDOWN"};
    ESP_LOGI(TAG, "Switching to mode: %s", mode_names[mode]);
    current_mode = mode;
}

operating_mode_t mode_manager_get_mode(void)
{
    return current_mode;
}

int mode_manager_is_automatic_enabled(void)
{
    return automatic_enabled;
}

void mode_manager_set_automatic_enabled(int enabled)
{
    ESP_LOGI(TAG, "Automatic mode: %s", enabled ? "ENABLED" : "DISABLED");
    automatic_enabled = enabled;
}

int mode_manager_get_manual_level(void)
{
    return manual_level;
}

void mode_manager_cycle_manual_level(void)
{
    manual_level++;
    if (manual_level > 3) manual_level = 1;
    
    operating_mode_t new_mode;
    switch(manual_level) {
        case 1: new_mode = MODE_MANUAL_LOW; break;
        case 2: new_mode = MODE_MANUAL_MEDIUM; break;
        case 3: new_mode = MODE_MANUAL_HIGH; break;
        default: new_mode = MODE_MANUAL_LOW;
    }
    mode_manager_set_mode(new_mode);
    ESP_LOGI(TAG, "Manual level: %d", manual_level);
}

int mode_manager_transition(operating_mode_t new_mode)
{
    // Safety checks before transition
    if (current_mode == MODE_ERROR) {
        ESP_LOGE(TAG, "Cannot transition from ERROR mode");
        return 0;
    }

    // Can only enable automatic if user has explicitly enabled it
    if (new_mode == MODE_AUTOMATIC && !automatic_enabled) {
        ESP_LOGW(TAG, "Automatic mode not enabled by user");
        return 0;
    }

    mode_manager_set_mode(new_mode);
    return 1;
}
