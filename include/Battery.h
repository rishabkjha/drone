// ============================================================================
//  Battery.h  -  2S pack on GPIO 0 (18K / 10K divider) -> app battery level.
//  Global object `battery`.
// ============================================================================
#pragma once
#include "Config.h"

class Battery {
public:
  float Volts = 0;

  void setup() {
    analogReadResolution(12);
    analogSetPinAttenuation(BATT_PIN, ADC_11db);
    pinMode(BATT_PIN, INPUT);
    Volts = readVolts();
    LastMs = millis();
  }
  void update() {
    uint32_t now = millis();
    if (now - LastMs < 20) return;          // 50 Hz
    LastMs = now;
    Volts += 0.05f * (readVolts() - Volts);
  }
  uint8_t level() const {
    float f = (Volts - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V);
    f = constrain(f, 0.0f, 1.0f);
    return (uint8_t)(f * 255.0f + 0.5f);
  }

private:
  uint32_t LastMs = 0;
  float readVolts() {
    uint32_t mv = 0;
    for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(BATT_PIN);
    return (mv / 8.0f) / 1000.0f * BATT_DIVIDER * BATT_CAL;
  }
};

Battery battery;
