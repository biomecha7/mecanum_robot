# IMU Task Architecture

## Overview

The IMU sensor has been refactored from being owned by the `MotionController` to being managed by a dedicated `IMUTask`. This change provides better separation of concerns and allows for more efficient RTOS-based sensor management.

## Architecture Changes

### Before (MotionController owned IMU)
- `MotionController` created and managed `IMUDriver` instance
- IMU updates were called from `MotionController::updateSensors()`
- IMU data was directly accessed within motion control context

### After (Dedicated IMU Task)
- `IMUTask` owns and manages `IMUDriver` instance
- IMU runs in its own FreeRTOS task at 200Hz
- IMU data is shared via thread-safe queue mechanism
- Other tasks can access IMU data without blocking

## New Components

### IMUTask Class
- **File**: `include/IMUTask.h`, `src/IMUTask.cpp`
- **Purpose**: Manages IMU sensor in dedicated FreeRTOS task
- **Features**:
  - Thread-safe data sharing via queue
  - Configurable update frequency (200Hz default)
  - Non-blocking data access
  - Task statistics and monitoring

### Key Methods
```cpp
// Initialize IMU task and sensor
bool initialize();

// Start the IMU task
bool start();

// Get latest IMU data (non-blocking)
bool getLatestData(IMUData& data, uint32_t timeout_ms = 0);

// Check if task is running
bool isRunning() const;

// Get task statistics
void getStats(uint32_t& update_count, uint32_t& last_update_ms) const;
```

## Usage Example

```cpp
// In main.cpp
IMUTask imuTask(IMU_SDA, IMU_SCL);

void setup() {
    // Initialize and start IMU task
    if (imuTask.initialize() && imuTask.start()) {
        Serial.println("✅ IMU task started");
    }
}

void loop() {
    // Access IMU data from any task
    IMUData imu_data;
    if (imuTask.getLatestData(imu_data, 0)) {  // Non-blocking
        // Use IMU data for navigation, orientation, etc.
        float heading_rate = imu_data.gyro_z * DEG_TO_RAD;
        // ... process data
    }
}
```

## Benefits

1. **Separation of Concerns**: Motion control and IMU sensing are now independent
2. **Real-time Performance**: IMU runs at consistent 200Hz regardless of motion control timing
3. **Thread Safety**: Queue-based communication prevents data corruption
4. **Non-blocking Access**: Other tasks can access IMU data without waiting
5. **Scalability**: Easy to add more sensor tasks following the same pattern
6. **Debugging**: IMU task can be monitored independently

## Task Priorities

- **IMU Task**: Priority 3 (high priority for sensor data)
- **Control Task**: Priority 4 (highest priority for motion control)
- **Network Task**: Priority 2 (medium priority)
- **Heartbeat Task**: Priority 1 (lowest priority)

## Future Enhancements

1. **Sensor Fusion**: Add complementary filter or Kalman filter in IMUTask
2. **Data Logging**: Add IMU data logging capability
3. **Calibration**: Add IMU calibration routines
4. **Multiple Sensors**: Extend pattern to other sensors (GPS, etc.)
5. **ROS Integration**: Publish IMU data via micro-ROS

## Migration Notes

- `MotionController` no longer has IMU dependencies
- IMU data access now requires using `IMUTask::getLatestData()`
- Heading calculation from IMU data should be moved to appropriate task
- All IMU-related initialization moved to `IMUTask::initialize()`