#include "hardware/esp_comms.h"
#include <sstream>
#include <cstdlib>
#include <string.h>

void EspComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    
    auto timeout = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(timeout);

    try {
        serial_conn_.open();
    } catch (std::exception &e) {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
    }
}

void EspComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void EspComms::readEncoderValues(int &val_1, int &val_2, int &val_3, int &val_4)
{
    
    std::string response = sendMsg("e\r");
    
    std::stringstream ss(response);
    
    int v1 = 0;
    int v2 = 0;
    int v3 = 0;
    int v4 = 0;

    if (ss >> v1 >> v2 >> v3 >> v4) {
        val_1 = v1;
        val_2 = v2;
        val_3 = v3;
        val_4 = v4;
    } else {
    }
}

void EspComms::setMotorValues(int val_1, int val_2, int val_3, int val_4)
{
    std::stringstream ss;
    
    ss << "o " << val_1 << " " << val_2 << " " << val_3 << " " << val_4 << "\r";
    
    sendMsg(ss.str());
}

void EspComms::setPidValues(int k_p, int k_d, int k_i, int k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string EspComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.flushInput();
    
    serial_conn_.write(msg_to_send);

    std::string response = "";
    try
    {
        response = serial_conn_.readline();
        
    }
    catch (const std::exception &e)
    {
    
    }
    return response;
}