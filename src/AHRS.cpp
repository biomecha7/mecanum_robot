#include "AHRS.h"

AHRS::AHRS(float sampleFreq, float beta) : _madgwick(sampleFreq, beta) {
    _data = {};
    _data.valid = false;
}

void AHRS::update(const IMUData& imu) {
    if (!imu.data_ready) {
        _data.valid = false;
        return;
    }
    // Feed IMU data to Madgwick filter
    _madgwick.update(
        imu.gyro_x, imu.gyro_y, imu.gyro_z,
        imu.accel_x, imu.accel_y, imu.accel_z,
        imu.mag_x, imu.mag_y, imu.mag_z
    );
    _madgwick.getQuaternion(_data.q[0], _data.q[1], _data.q[2], _data.q[3]);
    _madgwick.getEuler(_data.roll, _data.pitch, _data.yaw);
    _data.valid = true;
}

const AHRSData& AHRS::getData() const {
    return _data;
}
