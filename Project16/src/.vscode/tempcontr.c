#include <stdio.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/ledc.h"          // PWM (LEDC)
#include "driver/i2c_master.h"    // new I2C master driver



// ---- User + safety constants ----
float userTargetC = 34.0f;             // adjustable, but clamped
const float TARGET_MIN_C = 28.0f;
const float TARGET_MAX_C = 38.0f;

const float SKIN_MAX_C = 40.0f;        // hard stop
const float SKIN_MIN_C = 18.0f;        // hard stop (optional but recommended)

const float HYST_C = 0.5f;             // hysteresis band
const uint8_t PWM_MIN_RUN = 70;         // pump won't spin below this (tune)
const uint8_t PWM_MAX = 255;

// Ramp limiting
const uint8_t PWM_STEP = 5;            // max change per control update
uint8_t pwmNow = 0;

// Optional: "kick" for startup
const uint16_t KICK_MS = 300;
const uint8_t KICK_PWM = 255;
uint32_t pumpOnMs = 0;
bool pumpWasOn = false;

uint8_t tempControlPWM(float skinC) {
  // 0) Clamp target so user can't request unsafe values
  if (userTargetC < TARGET_MIN_C) userTargetC = TARGET_MIN_C;
  if (userTargetC > TARGET_MAX_C) userTargetC = TARGET_MAX_C;

  // 1) Safety rails (could also route to COOLDOWN state)
  if (!isfinite(skinC)) return 0;
  if (skinC >= SKIN_MAX_C) return 0;
  if (skinC <= SKIN_MIN_C) return 0;

  // 2) Hysteresis-based control
  //    - If too hot: increase PWM proportional-ish
  //    - If comfortably cool: reduce / possibly stop
  float err = skinC - userTargetC;

  // In band -> hold current PWM (avoids chattering)
  if (fabs(err) <= HYST_C) {
    return pwmNow;
  }

  // Too hot -> map error to PWM (simple piecewise)
  if (err > HYST_C) {
    // Example mapping: bigger error => higher PWM
    if (err > 3.0f) return PWM_MAX;
    if (err > 2.0f) return 200;
    if (err > 1.0f) return 160;
    return 120;
  }

  // Too cold -> reduce strongly / off
  if (err < -HYST_C) {
    if (pwmNow <= PWM_MIN_RUN) return 0;
    return PWM_MIN_RUN; // keep a trickle if you prefer
  }

  return 0;
}

uint8_t rampTo(uint8_t current, uint8_t target) {
  if (target > current) {
    uint16_t x = current + PWM_STEP;
    return (x > target) ? target : (uint8_t)x;
  } else if (target < current) {
    int16_t x = (int16_t)current - PWM_STEP;
    return (x < target) ? target : (uint8_t)x;
  }
  return current;
}

void applyPump(uint8_t cmd) {
  // enforce min-run if nonzero
  uint8_t out = (cmd == 0) ? 0 : (cmd < PWM_MIN_RUN ? PWM_MIN_RUN : cmd);

  bool pumpOn = (out > 0);
  if (pumpOn && !pumpWasOn) {
    pumpOnMs = millis();
    // kick to ensure spin
    analogWrite(PIN_PUMP_PWM, KICK_PWM);
  }

  // after kick window, write actual PWM
  if (pumpOn && (millis() - pumpOnMs) >= KICK_MS) {
    analogWrite(PIN_PUMP_PWM, out);
  } else if (!pumpOn) {
    analogWrite(PIN_PUMP_PWM, 0);
  }

  pumpWasOn = pumpOn;
}

// Call this every ~100ms
void controlTick(float skinC) {
  uint8_t desired = tempControlPWM(skinC);
  pwmNow = rampTo(pwmNow, desired);
  applyPump(pwmNow);
}
