#include "IMUDriver.h"

IMUDriver::IMUDriver(uint8_t sda, uint8_t scl) : _sda(sda), _scl(scl) {
    _data = {};
}

bool IMUDriver::begin() {
    Wire.begin(_sda, _scl);
    Wire.setClock(400000);
    if (_icm.begin(Wire, 0) == ICM_20948_Stat_Ok || _icm.begin(Wire, 1) == ICM_20948_Stat_Ok) {
        _data.data_ready = false;
        _data.last_read = 0;
        return true;
    }
    return false;
}

void IMUDriver::update() {
    if (_icm.dataReady()) {
        _icm.getAGMT();
        _data.accel_x = _icm.accX() / 16384.0f;
        _data.accel_y = _icm.accY() / 16384.0f;
        _data.accel_z = _icm.accZ() / 16384.0f;
        _data.gyro_x = _icm.gyrX() / 131.0f;
        _data.gyro_y = _icm.gyrY() / 131.0f;
        _data.gyro_z = _icm.gyrZ() / 131.0f;
        _data.mag_x = _icm.magX();
        _data.mag_y = _icm.magY();
        _data.mag_z = _icm.magZ();
        _data.temperature = _icm.temp() / 333.87f + 21.0f;
        _data.data_ready = true;
        _data.last_read = millis();
    }
}

const IMUData& IMUDriver::getData() const {
    return _data;
}
