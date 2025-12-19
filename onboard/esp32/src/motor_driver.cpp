#include <Arduino.h>
#include <commands.h>

#define USE_BASE
#define L298_MOTOR_DRIVER
#include "motor_driver.h"

#ifdef USE_BASE

#ifdef L298_MOTOR_DRIVER

// Define PWM channels for ESP32
const int PWM_FREQ = 20000;  // 20 kHz
const int PWM_RESOLUTION = 8; // 8-bit resolution

const int FRONT_LEFT_CHANNEL = 0;
const int FRONT_RIGHT_CHANNEL = 1;
const int BACK_LEFT_CHANNEL = 2;
const int BACK_RIGHT_CHANNEL = 3;

void initMotorController() {
  // Setup PWM channels
  ledcSetup(FRONT_LEFT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(FRONT_RIGHT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(BACK_LEFT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(BACK_RIGHT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  // Attach channels to pins
  ledcAttachPin(FRONT_LEFT_MOTOR_PWM, FRONT_LEFT_CHANNEL);
  ledcAttachPin(FRONT_RIGHT_MOTOR_PWM, FRONT_RIGHT_CHANNEL);
  ledcAttachPin(BACK_LEFT_MOTOR_PWM, BACK_LEFT_CHANNEL);
  ledcAttachPin(BACK_RIGHT_MOTOR_PWM, BACK_RIGHT_CHANNEL);
}

void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;

  if (spd < 0) {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255) spd = 255;

  if (i == FRONT_LEFT) { 
    if (reverse == 0) { ledcWrite(FRONT_LEFT_CHANNEL, spd); digitalWrite(FRONT_LEFT_MOTOR_DIR, HIGH); }
    else               { ledcWrite(FRONT_LEFT_CHANNEL, spd); digitalWrite(FRONT_LEFT_MOTOR_DIR, LOW); }
  }
  else if (i == FRONT_RIGHT) {
    if (reverse == 0) { ledcWrite(FRONT_RIGHT_CHANNEL, spd); digitalWrite(FRONT_RIGHT_MOTOR_DIR, HIGH); }
    else               { ledcWrite(FRONT_RIGHT_CHANNEL, spd); digitalWrite(FRONT_RIGHT_MOTOR_DIR, LOW); }
  }
  else if (i == BACK_LEFT) { 
    if (reverse == 0) { ledcWrite(BACK_LEFT_CHANNEL, spd); digitalWrite(BACK_LEFT_MOTOR_DIR, HIGH); }
    else               { ledcWrite(BACK_LEFT_CHANNEL, spd); digitalWrite(BACK_LEFT_MOTOR_DIR, LOW); }
  }
  else if (i == BACK_RIGHT) {
    if (reverse == 0) { ledcWrite(BACK_RIGHT_CHANNEL, spd); digitalWrite(BACK_RIGHT_MOTOR_DIR, HIGH); }
    else               { ledcWrite(BACK_RIGHT_CHANNEL, spd); digitalWrite(BACK_RIGHT_MOTOR_DIR, LOW); }
  }
}

void setMotorSpeeds(int front_leftSpeed, int front_rightSpeed, int back_leftSpeed, int back_rightSpeed) {
  setMotorSpeed(FRONT_LEFT, front_leftSpeed);
  setMotorSpeed(FRONT_RIGHT, front_rightSpeed);
  setMotorSpeed(BACK_LEFT, back_leftSpeed);
  setMotorSpeed(BACK_RIGHT, back_rightSpeed);
}

#else
  #error A motor driver must be selected!
#endif

#endif
