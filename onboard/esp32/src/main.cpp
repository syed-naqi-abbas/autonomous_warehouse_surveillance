#define USE_BASE      // Enable the base controller code
#undef USE_SERVOS     // Disable use of PWM servos

#define BAUDRATE     115200
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "commands.h"

#define ARDUINO_ENC_COUNTER

   /* L298 Motor driver*/
#define L298_MOTOR_DRIVER

#ifdef USE_BASE
  #include "motor_driver.h"
  #include "encoder_driver.h"
  #include "diff_controller.h"
  #include "stepper_driver.h"

  #define PID_RATE           30     // Hz
  const int PID_INTERVAL = 1000 / PID_RATE;
  unsigned long nextPID = PID_INTERVAL;
  #define AUTO_STOP_INTERVAL 100
  long lastMotorCommand = AUTO_STOP_INTERVAL;

#endif

int arg = 0;
int my_index = 0;
char chr;
char cmd;
char argv1[16], argv2[16], argv3[16], argv4[16];
long arg1, arg2, arg3, arg4;

void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = arg2 = arg3 = arg4 = 0;
  arg = 0;
  my_index = 0;
}

void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);

  switch(cmd) {
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(FRONT_LEFT));
    Serial.print(" ");
    Serial.print(readEncoder(FRONT_RIGHT));
    Serial.print(" ");
    Serial.print(readEncoder(BACK_LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(BACK_RIGHT));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0 && arg3 == 0 && arg4 == 0) {
      setMotorSpeeds(0, 0, 0, 0);
      resetPID();
      moving = 0;
    } else moving = 1;
    front_leftPID.TargetTicksPerFrame = arg1;
    front_rightPID.TargetTicksPerFrame = arg2;
    back_leftPID.TargetTicksPerFrame = arg3;
    back_rightPID.TargetTicksPerFrame = arg4;
    Serial.println("OK");
    break;
  case MOTOR_RAW_PWM:
    lastMotorCommand = millis();
    resetPID();
    moving = 0;
    setMotorSpeeds(arg1, arg2, arg3, arg4);
    Serial.println("OK");
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != NULL) {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
  case STEPPER_STEP_POSITIVE:
    stepMotor(arg1, true);
    setMotorSpeeds(0, 0, 0, 0);
    Serial.println("OK");
    break;
  case STEPPER_STEP_NEGATIVE:
    stepMotor(arg1, false);
    setMotorSpeeds(0, 0, 0, 0);
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }

  return;
}

/* ===================== Hardware Interrupts for ESP32 ===================== */

void setup() {
  Serial.begin(BAUDRATE);

#ifdef USE_BASE
#ifdef ARDUINO_ENC_COUNTER
  // Set encoder pins
  pinMode(FRONT_LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(FRONT_LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(FRONT_RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(FRONT_RIGHT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(BACK_LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(BACK_LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(BACK_RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(BACK_RIGHT_ENC_PIN_B, INPUT_PULLUP);
    // Stepper pins
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);

  // Attach hardware interrupts
  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENC_PIN_A), frontLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENC_PIN_A), frontRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_LEFT_ENC_PIN_A), backLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_RIGHT_ENC_PIN_A), backRightISR, CHANGE);
#endif

  initMotorController();
  resetPID();
#endif

  pinMode(FRONT_LEFT_MOTOR_DIR, OUTPUT);
  pinMode(FRONT_RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(BACK_LEFT_MOTOR_DIR, OUTPUT);
  pinMode(BACK_RIGHT_MOTOR_DIR, OUTPUT);
}

void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();

    if (chr == 13) {
        if (arg == 1) argv1[my_index] = NULL;
        else if (arg == 2) argv2[my_index] = NULL;
        else if (arg == 3) argv3[my_index] = NULL;
        else if (arg == 4) argv4[my_index] = NULL;

        runCommand();
        resetCommand();
    }
    else if (chr == ' ') {
        if (arg == 0) arg = 1;
        else if (arg == 1) { argv1[my_index] = NULL; arg = 2; my_index = 0; }
        else if (arg == 2) { argv2[my_index] = NULL; arg = 3; my_index = 0; }
        else if (arg == 3) { argv3[my_index] = NULL; arg = 4; my_index = 0; }
        continue;
    }
    else {
        if (arg == 0) cmd = chr;
        else if (arg == 1) argv1[my_index++] = chr;
        else if (arg == 2) argv2[my_index++] = chr;
        else if (arg == 3) argv3[my_index++] = chr;
        else if (arg == 4) argv4[my_index++] = chr;
    }
  }

#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0, 0, 0);
    moving = 0;
  }
#endif
}
