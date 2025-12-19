#ifndef STEPPER_MOTOR_CONTROLLER_H
#define STEPPER_MOTOR_CONTROLLER_H

#include <string>

class StepperMotorController {
public:
    StepperMotorController();
    ~StepperMotorController();
    
    bool init(const std::string& port, int baudrate);
    bool moveSteps(int steps, char direction);
    bool stop();
    
    // Getters
    int getPosition() const;
    bool isMoving() const;
    
    // Serial communication
    bool sendToMicrocontroller(const char* command);
    bool openSerialPort(const std::string& port, int baudrate);
    void closeSerialPort();

private:
    int motor_position_;
    int serial_fd_;
    std::string port_name_;
    int baudrate_;
    bool is_moving_;
};

#endif // STEPPER_MOTOR_CONTROLLER_H