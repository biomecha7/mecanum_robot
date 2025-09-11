#include "IMUDriver.h"

IMUDriver::IMUDriver(uint8_t sda, uint8_t scl) : _sda(sda), _scl(scl) {
    // Initialize data structure to zero values
    _data = {};
}

bool IMUDriver::begin() {
    // Initialize I2C communication
    Wire.begin(_sda, _scl);
    Wire.setClock(IMU_I2C_CLOCK_SPEED);
    
    // Try both possible I2C addresses for device detection
    if (_icm.begin(Wire, IMU_DEVICE_ADDRESS_0) == ICM_20948_Stat_Ok || 
        _icm.begin(Wire, IMU_DEVICE_ADDRESS_1) == ICM_20948_Stat_Ok) {
        
        // Initialize data structure
        _data.data_ready = false;
        _data.last_read = 0;
        return true;
    }
    
    return false;
}

void IMUDriver::update() {
    // Only read data if new data is available from the sensor
    if (_icm.dataReady()) {
        // Read all sensor data from ICM-20948
        _icm.getAGMT();
        
        // Convert accelerometer data to g (gravity units)
        _data.accel_x = _icm.accX() / IMU_ACCEL_SCALE_FACTOR;
        _data.accel_y = _icm.accY() / IMU_ACCEL_SCALE_FACTOR;
        _data.accel_z = _icm.accZ() / IMU_ACCEL_SCALE_FACTOR;
        
        // Convert gyroscope data to degrees per second (dps)
        _data.gyro_x = _icm.gyrX() / IMU_GYRO_SCALE_FACTOR;
        _data.gyro_y = _icm.gyrY() / IMU_GYRO_SCALE_FACTOR;
        _data.gyro_z = _icm.gyrZ() / IMU_GYRO_SCALE_FACTOR;
        
        // Magnetometer data (raw units - may require calibration)
        _data.mag_x = _icm.magX();
        _data.mag_y = _icm.magY();
        _data.mag_z = _icm.magZ();
        
        // Convert temperature to Celsius
        _data.temperature = _icm.temp() / IMU_TEMP_SCALE_FACTOR + IMU_TEMP_OFFSET;
        
        // Mark data as ready and update timestamp
        _data.data_ready = true;
        _data.last_read = millis();
    }
}

const IMUData& IMUDriver::getData() const {
    return _data;
}
