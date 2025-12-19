#ifndef MECANUMDRIVE_ESP_WHEEL_H
#define MECANUMDRIVE_ESP_WHEEL_H

#include <string>
#include <cmath>

class Wheel
{
public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Wheel() = default;

    // Declarations ONLY (No code logic here)
    Wheel(const std::string &wheel_name, int counts_per_rev);
    void setup(const std::string &wheel_name, int counts_per_rev);
    double calcEncAngle();
};

#endif // MECANUMDRIVE_ESP_WHEEL_H