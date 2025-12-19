/*************************************************************
 * TB6600 stepper control + encoder reading - ESP32
 *************************************************************/

#include <Arduino.h>
#include <commands.h>
#include "stepper_driver.h"

#pragma endregion

#define RPM 350
int delayMS = 60L * 1000000L / (RPM * 200); // 200 steps per revolution

void stepMotor(int steps, bool dir) {
  digitalWrite(STEPPER_DIR_PIN, dir);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(delayMS);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delayMicroseconds(delayMS);
  }
}