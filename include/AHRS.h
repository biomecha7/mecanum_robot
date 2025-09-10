#pragma once
#include "IMUDriver.h"

struct AHRSData {
    float q[4]; // Quaternion
    float roll, pitch, yaw; // Euler angles
    bool valid;
};

class AHRS {
public:
    AHRS();
    void update(const IMUData& imu);
    const AHRSData& getData() const;
private:
    AHRSData _data;
    // Add algorithm state variables here
};
