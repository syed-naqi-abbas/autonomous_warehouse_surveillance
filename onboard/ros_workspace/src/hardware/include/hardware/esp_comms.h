#ifndef MECANUMDRIVE_ESP_ESP_COMMS_H
#define MECANUMDRIVE_ESP_ESP_COMMS_H

// Use the standard serial library (wjwwood/serial)
// This is much easier to use than LibSerial
#include <serial/serial.h>
#include <iostream>
#include <sstream>

class EspComms
{
public:
  EspComms() = default;

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  
  // 4-wheel interface for Mecanum
  void readEncoderValues(int &val_1, int &val_2, int &val_3, int &val_4);
  void setMotorValues(int val_1, int val_2, int val_3, int val_4);
  
  void setPidValues(int k_p, int k_d, int k_i, int k_o);

  // Check connection status
  bool connected() const { return serial_conn_.isOpen(); }
  
  // Helper to send message and get response
  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);

private:
  serial::Serial serial_conn_; 

};

#endif // MECANUMDRIVE_ESP_ESP_COMMS_H