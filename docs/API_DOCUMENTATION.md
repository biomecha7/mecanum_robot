# API Documentation

This document provides comprehensive API documentation for the Mecanum Robot Controller components.

## 📋 Table of Contents

- [MotionController](#motioncontroller)
- [EncoderTask](#encodertask)
- [IMUTask](#imutask)
- [Supervisor](#supervisor)
- [ControlTask](#controltask)
- [CommsTask](#commstask)
- [PS2Controller](#ps2controller)
- [PIDController](#pidcontroller)
- [MotorDriver](#motordriver)

---

## MotionController

The core control system responsible for mecanum wheel kinematics, motor control, and robot state management.

### Constructor
```cpp
MotionController(EncoderTask& encoder_task)
```
- **Parameters**: `encoder_task` - Reference to encoder task for sensor data
- **Description**: Initializes the motion controller with encoder dependency injection

### Public Methods

#### Control Management
```cpp
void initialize()
```
Initializes PWM channels, motor drivers, and PID controllers.

```cpp
void setControlMode(ControlMode mode)
```
Sets the control mode for the robot.
- **Parameters**: `mode` - Control mode (OPEN_LOOP, VELOCITY_PID, ORIENTATION_PID, POSITION_PID)

```cpp
ControlMode getControlMode() const
```
Returns the current control mode.

#### Movement Control
```cpp
void drive(float forward, float strafe, float rotate)
```
Controls robot movement in body frame.
- **Parameters**: 
  - `forward` - Forward velocity (m/s)
  - `strafe` - Sideways velocity (m/s) 
  - `rotate` - Angular velocity (rad/s)

```cpp
void driveWithHeading(float forward, float strafe, float target_heading)
```
Controls movement with orientation control.
- **Parameters**:
  - `forward` - Forward velocity (m/s)
  - `strafe` - Sideways velocity (m/s)
  - `target_heading` - Target heading angle (rad)

```cpp
void stop()
```
Immediately stops all motors.

#### Sensor Integration
```cpp
void updateSensors()
```
Updates encoder data and wheel velocities.

```cpp
void updateOdometry()
```
Updates robot pose using wheel odometry.

#### PID Control
```cpp
void enablePIDControl(bool enable)
```
Enables or disables PID control.

```cpp
void setVelocityPIDGains(float kp, float ki, float kd)
```
Sets PID gains for velocity control.
- **Parameters**: `kp` - Proportional gain, `ki` - Integral gain, `kd` - Derivative gain

```cpp
void setOrientationPIDGains(float kp, float ki, float kd)
```
Sets PID gains for orientation control.

```cpp
void resetPIDControllers()
```
Resets all PID controllers to initial state.

#### State Access
```cpp
const RobotState& getState() const
```
Returns current robot state including position, velocity, and wheel data.

```cpp
float getWheelbaseInches() const
```
Returns wheelbase distance in inches.

```cpp
float getWheelDiameterMm() const
```
Returns wheel diameter in millimeters.

```cpp
float getDeadband() const
```
Returns joystick deadband value.

#### Data Access
```cpp
void getEncoderCounts(int32_t counts[4]) const
```
Gets raw encoder counts for all wheels.
- **Parameters**: `counts` - Array to store encoder counts [FL, FR, RL, RR]

```cpp
void getWheelPositions(float positions[4]) const
```
Gets wheel positions in meters.

```cpp
void getWheelVelocities(float velocities[4]) const
```
Gets wheel velocities in m/s.

#### Debug
```cpp
void printDebugInfo()
```
Prints current robot state and sensor data.

```cpp
void printPIDStatus()
```
Prints PID controller status and gains.

---

## EncoderTask

FreeRTOS task responsible for reading and processing encoder data from all four wheels.

### Constructor
```cpp
EncoderTask()
```
Initializes the encoder task with default parameters.

### Public Methods

#### Task Management
```cpp
bool initialize()
```
Initializes encoder pins and interrupt handlers.
- **Returns**: `true` if successful, `false` otherwise

```cpp
bool start()
```
Starts the encoder task.
- **Returns**: `true` if successful, `false` otherwise

```cpp
void stop()
```
Stops the encoder task.

#### Data Access
```cpp
void getLatestData(EncoderQueueData& data)
```
Gets the most recent encoder data.
- **Parameters**: `data` - Reference to store encoder data

```cpp
QueueHandle_t getQueue() const
```
Returns the encoder data queue handle.

#### Configuration
```cpp
void setPublishRate(float rate_hz)
```
Sets the data publishing rate.
- **Parameters**: `rate_hz` - Publishing rate in Hz

---

## IMUTask

FreeRTOS task for IMU data acquisition and processing using the ICM-20948 sensor.

### Constructor
```cpp
IMUTask(gpio_num_t sda_pin, gpio_num_t scl_pin)
```
Initializes IMU task with I2C pins.
- **Parameters**: `sda_pin`, `scl_pin` - I2C communication pins

### Public Methods

#### Task Management
```cpp
bool initialize()
```
Initializes I2C communication and IMU sensor.
- **Returns**: `true` if successful, `false` otherwise

```cpp
bool start()
```
Starts the IMU task.
- **Returns**: `true` if successful, `false` otherwise

```cpp
void stop()
```
Stops the IMU task.

#### Data Access
```cpp
void getLatestData(IMUData& data)
```
Gets the most recent IMU data.
- **Parameters**: `data` - Reference to store IMU data

```cpp
QueueHandle_t getQueue() const
```
Returns the IMU data queue handle.

#### Calibration
```cpp
void calibrate()
```
Performs IMU calibration for bias compensation.

---

## Supervisor

State machine that manages robot operation modes and command arbitration.

### Constructor
```cpp
Supervisor(PS2Controller& ps2)
```
Initializes supervisor with PS2 controller reference.
- **Parameters**: `ps2` - Reference to PS2 controller

### Public Methods

#### Task Management
```cpp
bool initialize()
```
Initializes the supervisor state machine.
- **Returns**: `true` if successful, `false` otherwise

```cpp
bool start()
```
Starts the supervisor task.
- **Returns**: `true` if successful, `false` otherwise

```cpp
void stop()
```
Stops the supervisor task.

#### ISetpointProvider Interface
```cpp
BodyCmd latest() const
```
Returns the current velocity command.
- **Returns**: `BodyCmd` containing vx, vy, wz, and timestamp

```cpp
const char* stateName() const
```
Returns the current state name as string.
- **Returns**: State name ("IDLE", "MANUAL_PS2", "TELEOP_PI", "ESTOP")

#### Command Interface
```cpp
void feedPiCmd(float vx, float vy, float wz, uint32_t t_ms)
```
Feeds command from Raspberry Pi.
- **Parameters**:
  - `vx` - Forward velocity (m/s)
  - `vy` - Sideways velocity (m/s)
  - `wz` - Angular velocity (rad/s)
  - `t_ms` - Timestamp in milliseconds

```cpp
void requestMode(const char* mode)
```
Requests mode change.
- **Parameters**: `mode` - Mode name ("TELEOP_PI", "MANUAL_PS2", etc.)

```cpp
uint32_t getCmdAgeMs() const
```
Returns age of last Pi command in milliseconds.

---

## ControlTask

Main control loop task that executes motion commands.

### Constructor
```cpp
ControlTask(MotionController& mc, PS2Controller& ps2, IMUTask& imu, ISetpointProvider& provider)
```
Initializes control task with dependencies.
- **Parameters**:
  - `mc` - Motion controller reference
  - `ps2` - PS2 controller reference
  - `imu` - IMU task reference
  - `provider` - Setpoint provider reference

### Public Methods

#### Task Management
```cpp
bool initialize()
```
Initializes the control task.
- **Returns**: `true` if successful, `false` otherwise

```cpp
bool start()
```
Starts the control task at 100Hz.
- **Returns**: `true` if successful, `false` otherwise

```cpp
void stop()
```
Stops the control task.

---

## CommsTask

Communication task responsible for serial data publishing and command reception.

### Constructor
```cpp
CommsTask()
```
Initializes the communication task.

### Public Methods

#### Task Management
```cpp
bool initialize()
```
Initializes serial communication.
- **Returns**: `true` if successful, `false` otherwise

```cpp
bool start()
```
Starts the communication task.
- **Returns**: `true` if successful, `false` otherwise

```cpp
void stop()
```
Stops the communication task.

#### Subscription Management
```cpp
void subscribeToEncoderTask(EncoderTask& encoder)
```
Subscribes to encoder task data.

```cpp
void subscribeToIMUTask(IMUTask& imu)
```
Subscribes to IMU task data.

```cpp
void subscribeToSupervisor(Supervisor& supervisor)
```
Subscribes to supervisor status data.

#### Configuration
```cpp
void setPublishRate(float rate_hz)
```
Sets telemetry publishing rate.
- **Parameters**: `rate_hz` - Publishing rate in Hz

---

## PS2Controller

Handles PS2 controller communication and button/joystick processing.

### Constructor
```cpp
PS2Controller()
```
Initializes PS2 controller with default pins.

### Public Methods

#### Initialization
```cpp
void initialize()
```
Initializes PS2 controller pins and communication.

#### Data Access
```cpp
bool update()
```
Updates controller state and processes new data.
- **Returns**: `true` if new data available, `false` otherwise

```cpp
float getVx() const
```
Returns forward/backward velocity from left stick.

```cpp
float getVy() const
```
Returns sideways velocity from right stick.

```cpp
float getWz() const
```
Returns angular velocity from stick combination.

```cpp
float getSpeedScale() const
```
Returns current speed scaling factor based on shoulder buttons.

#### Button Access
```cpp
bool isButtonPressed(uint8_t button) const
```
Checks if specific button is pressed.

```cpp
bool isEmergencyStop() const
```
Returns emergency stop state.

#### Configuration
```cpp
void setDeadband(float deadband)
```
Sets joystick deadband value.
- **Parameters**: `deadband` - Deadband value (0.0-1.0)

---

## PIDController

Proportional-Integral-Derivative controller implementation.

### Constructor
```cpp
PIDController(float kp, float ki, float kd, float output_min, float output_max)
```
Initializes PID controller with gains and limits.
- **Parameters**:
  - `kp` - Proportional gain
  - `ki` - Integral gain
  - `kd` - Derivative gain
  - `output_min` - Minimum output value
  - `output_max` - Maximum output value

### Public Methods

#### Control
```cpp
float compute(float setpoint, float input, float dt)
```
Computes PID output.
- **Parameters**:
  - `setpoint` - Target value
  - `input` - Current measured value
  - `dt` - Time delta in seconds
- **Returns**: PID output value

#### Configuration
```cpp
void setGains(float kp, float ki, float kd)
```
Sets PID gains.

```cpp
void setOutputLimits(float min, float max)
```
Sets output limits.

```cpp
void reset()
```
Resets PID controller state.

---

## MotorDriver

Controls individual motor using PWM signals.

### Constructor
```cpp
MotorDriver(gpio_num_t pwm_pin_a, gpio_num_t pwm_pin_b, uint8_t channel_a, uint8_t channel_b)
```
Initializes motor driver with PWM pins and channels.
- **Parameters**:
  - `pwm_pin_a` - PWM pin A
  - `pwm_pin_b` - PWM pin B
  - `channel_a` - LEDC channel A
  - `channel_b` - LEDC channel B

### Public Methods

#### Control
```cpp
void setSpeed(float speed)
```
Sets motor speed.
- **Parameters**: `speed` - Speed value (-1.0 to 1.0)

```cpp
void stop()
```
Stops the motor immediately.

#### Configuration
```cpp
void setPWMResolution(uint8_t resolution)
```
Sets PWM resolution in bits.

```cpp
void setPWMFrequency(uint32_t frequency)
```
Sets PWM frequency in Hz.

---

## Data Structures

### RobotState
```cpp
struct RobotState {
    float x, y;                    // Position in meters
    float heading;                 // Heading in radians
    float vx, vy, vz;             // Linear velocities in m/s
    float wx, wy, wz;             // Angular velocities in rad/s
    float wheel_positions[4];      // Wheel positions in meters
    float wheel_velocities[4];     // Wheel velocities in m/s
    uint32_t last_update_ms;       // Last update timestamp
    uint32_t last_encoder_read_ms; // Last encoder read timestamp
};
```

### BodyCmd
```cpp
struct BodyCmd {
    float vx, vy, wz;    // Velocities in m/s and rad/s
    uint32_t t_ms;       // Timestamp in milliseconds
};
```

### EncoderQueueData
```cpp
struct EncoderQueueData {
    uint32_t timestamp_ms;
    int32_t encoder_counts[4];
    int32_t encoder_deltas[4];
    float wheel_positions[4];
    float wheel_velocities[4];
    float filtered_velocities[4];
    float dt_ms;
    float frequency_hz;
    uint32_t update_count;
    bool data_valid;
    uint8_t encoder_errors[4];
};
```

### IMUData
```cpp
struct IMUData {
    uint32_t timestamp_ms;
    float accelerometer[3];    // X, Y, Z in m/s²
    float gyroscope[3];        // X, Y, Z in rad/s
    float magnetometer[3];     // X, Y, Z in μT
    float temperature;         // Temperature in °C
    bool data_valid;
};
```

---

## Error Handling

All components implement consistent error handling:

- **Initialization errors**: Return `false` from `initialize()` methods
- **Runtime errors**: Log errors and continue operation when possible
- **Critical errors**: Trigger emergency stop or safe shutdown
- **Communication errors**: Retry with exponential backoff

## Performance Characteristics

- **Control Loop**: 100Hz (10ms period)
- **Encoder Updates**: 100Hz
- **IMU Updates**: 200Hz
- **Communication**: 50Hz telemetry publishing
- **Latency**: <5ms from command to motor response
- **Memory Usage**: <80% of available RAM
- **CPU Usage**: <70% of available processing time

## Thread Safety

- All public methods are thread-safe
- Internal state protected by mutexes where necessary
- Queue-based communication between tasks
- Atomic operations for critical data
