#include <Arduino.h>

// Define GPIO pins for 4 motor PWM outputs
const uint8_t MOTOR_PINS[4] = {5, 6, 3, 4}; 

// PWM Parameters
const uint32_t PWM_FREQ = 5000; // 5 kHz frequency
const uint8_t PWM_RES   = 8;    // 8-bit resolution (0 - 255)

void setPWMZero() {
// Initialize PWM channels on ESP32
  for (uint8_t i = 0; i < 4; i++) {
    // LEDC API compatibility for PlatformIO Espressif32 core v2.x & v3.x
    #if ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcAttachChannel(MOTOR_PINS[i], PWM_FREQ, PWM_RES, i);
      ledcWrite(MOTOR_PINS[i], 0);
    #else
      ledcSetup(i, PWM_FREQ, PWM_RES);
      ledcAttachPin(MOTOR_PINS[i], i);
      ledcWrite(i, 0);
    #endif
  }
}

void setMotorPWM(int motorNum, uint32_t pwmVal) {
  pwmVal = map(pwmVal, 1000, 2000, 0, 255);
  uint8_t channel = motorNum - 1;
  ledcWrite(channel, pwmVal);
}

void setup() {
  Serial.begin(115200);
  setPWMZero();
}

void loop() {
  setMotorPWM(1,1000);
}
