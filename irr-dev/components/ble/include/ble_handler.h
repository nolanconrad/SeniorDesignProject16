#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Call this in main.c's app_main()
void ble_init(void);

// Call this to send a notification to the browser
void ble_notify(const char* message);

// Optional: check connection state from main.c
bool ble_is_connected(void);

#ifdef __cplusplus
}
#endif