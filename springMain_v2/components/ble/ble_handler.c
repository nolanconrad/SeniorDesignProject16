#include "ble_handler.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "nvs_flash.h"
#include "pumpOperation.h"

#define TAG "BLE"

// ── Swap these when you have real UUIDs ──────────────────────────
#define SERVICE_UUID        0xFF01       // 16-bit short UUID for now
#define CHARACTERISTIC_UUID 0xFF02

#define PROFILE_NUM         1
#define PROFILE_APP_ID      0
#define GATTS_NUM_HANDLE    4

static bool is_connected = false;
static uint16_t conn_id = 0;
static esp_gatt_if_t gatts_if_global = 0;
static uint16_t char_handle = 0;

// ── Advertising data ─────────────────────────────────────────────
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 0,
    .p_service_uuid      = NULL,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// ── GAP callback (advertising) ───────────────────────────────────
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                               esp_ble_gap_cb_param_t* param) {
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&adv_params);
    }
}

// ── GATT callback ────────────────────────────────────────────────
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t* param) {
    switch (event) {

    case ESP_GATTS_REG_EVT: {
        esp_ble_gap_set_device_name("ESP32-Nano");
        esp_ble_gap_config_adv_data(&adv_data);

        esp_gatt_srvc_id_t service_id = {};
        service_id.is_primary = true;
        service_id.id.inst_id = 0;
        service_id.id.uuid.len = ESP_UUID_LEN_16;
        service_id.id.uuid.uuid.uuid16 = SERVICE_UUID;
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        esp_ble_gatts_start_service(param->create.service_handle);

        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = { .uuid16 = CHARACTERISTIC_UUID }
        };
        esp_attr_value_t char_val = {
            .attr_max_len = 20,
            .attr_len     = 0,
            .attr_value   = NULL
        };
        esp_ble_gatts_add_char(
            param->create.service_handle,
            &char_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            ESP_GATT_CHAR_PROP_BIT_READ |
            ESP_GATT_CHAR_PROP_BIT_WRITE |
            ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            &char_val,
            NULL
        );
        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT:
        char_handle = param->add_char.attr_handle;
        gatts_if_global = gatts_if;
        break;

    case ESP_GATTS_CONNECT_EVT:
        is_connected = true;
        conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "Client connected");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        ESP_LOGI(TAG, "Client disconnected, restarting advertising");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT: {
        // This fires when browser sends "ON" or "OFF"
        char buf[32] = {0};
        memcpy(buf, param->write.value, param->write.len);
        ESP_LOGI(TAG, "Received: %s", buf);

        if (strcmp(buf, "ON") == 0) {
            // do your ON logic here
        } else if (strcmp(buf, "OFF") == 0) {
            // do your OFF logic here
        }
        break;
    }

    default:
        break;
    }
}

// ── Public functions ─────────────────────────────────────────────
void ble_init(void) {
    nvs_flash_init();
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(PROFILE_APP_ID);

    ESP_LOGI(TAG, "BLE ready, waiting for connection...");
}

void ble_notify(const char* message) {
    if (!is_connected) return;
    esp_ble_gatts_send_indicate(
        gatts_if_global,
        conn_id,
        char_handle,
        strlen(message),
        (uint8_t*)message,
        false
    );
}

bool ble_is_connected(void) {
    return is_connected;
}