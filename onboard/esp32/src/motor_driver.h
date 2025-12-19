#ifdef L298_MOTOR_DRIVER

  // Front Left Motor
  #define FRONT_LEFT_MOTOR_PWM  14  // ESP32 safe PWM pin
  #define FRONT_LEFT_MOTOR_DIR  27

  // Front Right Motor
  #define FRONT_RIGHT_MOTOR_PWM  12
  #define FRONT_RIGHT_MOTOR_DIR  26

  // Back Left Motor
  #define BACK_LEFT_MOTOR_PWM   25
  #define BACK_LEFT_MOTOR_DIR   33

  // Back Right Motor
  #define BACK_RIGHT_MOTOR_PWM  32
  #define BACK_RIGHT_MOTOR_DIR  15

#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int front_leftSpeed, int front_rightSpeed, int back_leftSpeed, int back_rightSpeed);
