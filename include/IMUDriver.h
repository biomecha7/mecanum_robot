#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>

// ---- IMU Constants ----
// ICM-20948 Sensor Scaling Factors
#define IMU_ACCEL_SCALE_FACTOR 16384.0f    // Accelerometer scale factor (g)
#define IMU_GYRO_SCALE_FACTOR 131.0f       // Gyroscope scale factor (dps)
#define IMU_TEMP_SCALE_FACTOR 333.87f      // Temperature scale factor
#define IMU_TEMP_OFFSET 21.0f              // Temperature offset (°C)

// I2C Configuration
#define IMU_I2C_CLOCK_SPEED 400000         // I2C clock speed (Hz)
#define IMU_DEVICE_ADDRESS_0 0             // Primary I2C address
#define IMU_DEVICE_ADDRESS_1 1             // Secondary I2C address

// Data Structure
struct IMUData {
    float accel_x, accel_y, accel_z;       // Accelerometer data (g)
    float gyro_x, gyro_y, gyro_z;          // Gyroscope data (dps)
    float mag_x, mag_y, mag_z;             // Magnetometer data (raw)
    float temperature;                     // Temperature (°C)
    bool data_ready;                       // Data validity flag
    uint32_t last_read;                    // Last read timestamp (ms)
};

/**
 * @brief IMU Driver for ICM-20948 9-axis motion sensor
 * 
 * This class provides a high-level interface for reading accelerometer,
 * gyroscope, and magnetometer data from the ICM-20948 sensor. It handles
 * I2C communication, data scaling, and provides a clean API for sensor data.
 * 
 * Features:
 * - Automatic I2C address detection
 * - Proper data scaling to engineering units
 * - Temperature compensation
 * - Data ready flag for polling
 * 
 * @note The magnetometer data is returned in raw units and may require
 *       additional calibration for accurate heading calculations.
 */
class IMUDriver {
public:
    /**
     * @brief Construct a new IMUDriver object
     * @param sda I2C SDA pin number
     * @param scl I2C SCL pin number
     */
    IMUDriver(uint8_t sda, uint8_t scl);
    
    /**
     * @brief Initialize the IMU sensor
     * @return true if initialization successful, false otherwise
     * @note Attempts both I2C addresses (0 and 1) for device detection
     */
    bool begin();
    
    /**
     * @brief Update sensor data
     * 
     * Reads new data from the sensor if available and updates the internal
     * data structure. This method should be called regularly in the main loop.
     * 
     * @note Only updates data if new data is available from the sensor
     */
    void update();
    
    /**
     * @brief Get the current sensor data
     * @return const reference to IMUData structure
     * @note Data is only valid if data_ready flag is true
     */
    const IMUData& getData() const;

private:
    ICM_20948_I2C _icm;                    // ICM-20948 sensor object
    IMUData _data;                         // Current sensor data
    uint8_t _sda, _scl;                    // I2C pin assignments
};
