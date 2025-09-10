#pragma once
#include "IMUDriver.h"
#include "MadgwickAHRS.h"

struct AHRSData {
    float q[4]; // Quaternion
    float roll, pitch, yaw; // Euler angles
    bool valid;
};

class AHRS {
public:
    AHRS(float sampleFreq = 100.0f, float beta = 0.1f);
    void update(const IMUData& imu);
    const AHRSData& getData() const;
private:
    AHRSData _data;
    MadgwickAHRS _madgwick;
};
