#include "AHRS.h"

AHRS::AHRS() {
    _data = {};
    _data.valid = false;
}

void AHRS::update(const IMUData& imu) {
    // Stub: implement sensor fusion here
    // For now, just mark as invalid
    _data.valid = false;
}

const AHRSData& AHRS::getData() const {
    return _data;
}
