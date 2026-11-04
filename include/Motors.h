// ============================================================================
//  Motors.h  -  4 ESCs (ESP32Servo, 400 Hz) and the quad X mixer.
//  Global object `motors`.
// ============================================================================
#pragma once
#include <ESP32Servo.h>
#include "Config.h"

class Motors {
public:
  enum { FL, FR, RL, RR };
  int Us[4] = { 1000, 1000, 1000, 1000 };

  void setup() {
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
    for (int i = 0; i < 4; i++) {
      esc[i].setPeriodHertz(ESC_FREQ_HZ);
      esc[i].attach(PINS[i], ESC_MIN_US, ESC_MAX_US);
      esc[i].writeMicroseconds(ESC_MIN_US);   // stop signal right away
    }
  }

  void write(int i, int us) {
    Us[i] = constrain(us, ESC_MIN_US, ESC_MAX_US);
    esc[i].writeMicroseconds(Us[i]);
  }
  void all(int us) { for (int i = 0; i < 4; i++) write(i, us); }

  // Quad X mixer.
  // R + = more roll-right wanted: left motors up, right down.
  // P + = more nose-down wanted: rear motors up, front down.
  // Y + = more clockwise yaw wanted: the anticlockwise props (FL, RR) up.
  void mix(float T, float R, float P, float Y, int minUs) {
    write(FL, constrain((int)(T + R - P + Y), minUs, ESC_MAX_US));
    write(FR, constrain((int)(T - R - P - Y), minUs, ESC_MAX_US));
    write(RL, constrain((int)(T + R + P - Y), minUs, ESC_MAX_US));
    write(RR, constrain((int)(T - R + P + Y), minUs, ESC_MAX_US));
  }

  // Boot check: each motor alone, named on the Serial Monitor. If the one that
  // spins is not where the line says, the ESC wires are on the wrong pins.
  void readySpin() {
    for (int i = 0; i < 4; i++) {
      all(ESC_MIN_US);
      Serial.printf("Motor test: GPIO %d should be %s\n", PINS[i], NAMES[i]);
      write(i, READY_SPIN_US);
      delay(1000);
      all(ESC_MIN_US);
      delay(400);
    }
  }

private:
  const int PINS[4] = { ESC_FL_PIN, ESC_FR_PIN, ESC_RL_PIN, ESC_RR_PIN };
  const char* const NAMES[4] = { "FRONT LEFT (ACW)", "FRONT RIGHT (CW)", "REAR LEFT (CW)", "REAR RIGHT (ACW)" };
  Servo esc[4];
};

Motors motors;
