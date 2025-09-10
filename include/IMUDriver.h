#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>

struct IMUData {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float temperature;
    bool data_ready;
    uint32_t last_read;
};

class IMUDriver {
public:
    IMUDriver(uint8_t sda, uint8_t scl);
    bool begin();
    void update();
    const IMUData& getData() const;
private:
    ICM_20948_I2C _icm;
    IMUData _data;
    uint8_t _sda, _scl;
};
