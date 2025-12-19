#include "stepper_motor_controller.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

StepperMotorController::StepperMotorController()
    : motor_position_(0), serial_fd_(-1), 
      baudrate_(9600), is_moving_(false) {
}

StepperMotorController::~StepperMotorController() {
    closeSerialPort();
}

bool StepperMotorController::init(const std::string& port, int baudrate) {
    port_name_ = port;
    baudrate_ = baudrate;
    
    if (!openSerialPort(port, baudrate)) {
        std::cerr << "Failed to open serial port: " << port << std::endl;
        return false;
    }
    
    if (!sendToMicrocontroller("INIT")) {
        std::cerr << "Failed to initialize microcontroller" << std::endl;
        return false;
    }
    
    std::cout << "Stepper motor controller initialized on " << port << std::endl;
    return true;
}

bool StepperMotorController::openSerialPort(const std::string& port, int baudrate) {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        std::cerr << "Cannot open serial port: " << port << std::endl;
        return false;
    }
    
    struct termios options;
    tcgetattr(serial_fd_, &options);
    
    speed_t baud;
    switch (baudrate) {
        case 9600:
            baud = B9600;
            break;
        case 19200:
            baud = B19200;
            break;
        case 38400:
            baud = B38400;
            break;
        case 115200:
            baud = B115200;
            break;
        default:
            baud = B9600;
    }
    
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD;
    options.c_cflag |= CLOCAL;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    
    tcsetattr(serial_fd_, TCSANOW, &options);
    tcflush(serial_fd_, TCIOFLUSH);
    
    std::cout << "Serial port opened: " << port << std::endl;
    return true;
}

void StepperMotorController::closeSerialPort() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool StepperMotorController::sendToMicrocontroller(const char* command) {
    if (serial_fd_ < 0) {
        std::cerr << "Serial port not open" << std::endl;
        return false;
    }
    
    int command_length = strlen(command);
    ssize_t bytes_written = write(serial_fd_, command, command_length);
    
    if (bytes_written != command_length) {
        std::cerr << "Failed to send command: " << command << std::endl;
        return false;
    }
    
    std::cout << "Command sent: " << command << std::endl;
    return true;
}

bool StepperMotorController::moveSteps(int steps, char direction) {
    if (steps <= 0) {
        std::cerr << "Steps must be positive" << std::endl;
        return false;
    }
    
    if (direction != 'p' && direction != 'n') {
        std::cerr << "Direction must be 'p' or 'n'" << std::endl;
        return false;
    }
    
    is_moving_ = true;
    
    char command[50];
    snprintf(command, sizeof(command), "%c %d\r", direction, steps);
    
    bool result = sendToMicrocontroller(command);
    if (!result) {
        is_moving_ = false;
    }
    
    return result;
}

bool StepperMotorController::stop() {
    is_moving_ = false;
    return sendToMicrocontroller("STOP");
}

int StepperMotorController::getPosition() const {
    return motor_position_;
}

bool StepperMotorController::isMoving() const {
    return is_moving_;
}