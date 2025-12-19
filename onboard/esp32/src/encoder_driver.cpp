/* *************************************************************
   Encoder definitions - ESP32 Hardware Interrupt Version
************************************************************* */

#include <Arduino.h>
#include <commands.h>

#define USE_BASE
#define ARDUINO_ENC_COUNTER
#include "encoder_driver.h"

#ifdef USE_BASE

#ifdef ARDUINO_ENC_COUNTER

volatile long front_left_enc_pos = 0L;
volatile long front_right_enc_pos = 0L;
volatile long back_left_enc_pos = 0L;
volatile long back_right_enc_pos = 0L;

// Encoder lookup table for quadrature decoding
static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

// Previous A/B state for each motor
volatile uint8_t front_left_prev = 0;
volatile uint8_t front_right_prev = 0;
volatile uint8_t back_left_prev = 0;
volatile uint8_t back_right_prev = 0;

// ========================
// ISR functions
// ========================
void IRAM_ATTR frontLeftISR() {
    uint8_t a = digitalRead(FRONT_LEFT_ENC_PIN_A);
    uint8_t b = digitalRead(FRONT_LEFT_ENC_PIN_B);
    uint8_t state = (a << 1) | b;
    front_left_enc_pos += -ENC_STATES[(front_left_prev << 2) | state];
    front_left_prev = state;
}

void IRAM_ATTR frontRightISR() {
    uint8_t a = digitalRead(FRONT_RIGHT_ENC_PIN_A);
    uint8_t b = digitalRead(FRONT_RIGHT_ENC_PIN_B);
    uint8_t state = (a << 1) | b;
    front_right_enc_pos += ENC_STATES[(front_right_prev << 2) | state];
    front_right_prev = state;
}

void IRAM_ATTR backLeftISR() {
    uint8_t a = digitalRead(BACK_LEFT_ENC_PIN_A);
    uint8_t b = digitalRead(BACK_LEFT_ENC_PIN_B);
    uint8_t state = (a << 1) | b;
    back_left_enc_pos += -ENC_STATES[(back_left_prev << 2) | state];
    back_left_prev = state;
}

void IRAM_ATTR backRightISR() {
    uint8_t a = digitalRead(BACK_RIGHT_ENC_PIN_A);
    uint8_t b = digitalRead(BACK_RIGHT_ENC_PIN_B);
    uint8_t state = (a << 1) | b;
    back_right_enc_pos += -ENC_STATES[(back_right_prev << 2) | state];
    back_right_prev = state;
}

// ========================
// Read & Reset functions
// ========================
long readEncoder(int i) {
    if (i == FRONT_LEFT) return front_left_enc_pos;
    else if (i == FRONT_RIGHT) return front_right_enc_pos;
    else if (i == BACK_LEFT) return back_left_enc_pos;
    else return back_right_enc_pos;
}

void resetEncoder(int i) {
    if (i == FRONT_LEFT) front_left_enc_pos = 0L;
    else if (i == FRONT_RIGHT) front_right_enc_pos = 0L;
    else if (i == BACK_LEFT) back_left_enc_pos = 0L;
    else back_right_enc_pos = 0L;
}

void resetEncoders() {
    resetEncoder(FRONT_LEFT);
    resetEncoder(FRONT_RIGHT);
    resetEncoder(BACK_LEFT);
    resetEncoder(BACK_RIGHT);
}

#else
  #error An encoder driver must be selected!
#endif

#endif
