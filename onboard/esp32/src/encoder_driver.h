/* *************************************************************
   Encoder driver function definitions - ESP32 version
************************************************************* */

#ifdef ARDUINO_ENC_COUNTER

// Motor 1 (Front Left)
#define FRONT_LEFT_ENC_PIN_A   34
#define FRONT_LEFT_ENC_PIN_B   35

// Motor 2 (Front Right)
#define FRONT_RIGHT_ENC_PIN_A  36
#define FRONT_RIGHT_ENC_PIN_B  39

// Motor 3 (Back Left)
#define BACK_LEFT_ENC_PIN_A    4
#define BACK_LEFT_ENC_PIN_B    16

// Motor 4 (Back Right)
#define BACK_RIGHT_ENC_PIN_A   17
#define BACK_RIGHT_ENC_PIN_B   18

#endif

// Function prototypes
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

// ISR declarations (defined in encoder_driver.cpp)
#ifdef ARDUINO_ENC_COUNTER
extern void IRAM_ATTR frontLeftISR();
extern void IRAM_ATTR frontRightISR();
extern void IRAM_ATTR backLeftISR();
extern void IRAM_ATTR backRightISR();
#endif
