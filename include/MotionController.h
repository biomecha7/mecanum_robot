#pragma once
#include "MotorDriver.h"
#include "EncoderDriver.h"
#include "IMUDriver.h"
#include "PIDController.h"
#include "RobotPins.h"

// ---- Robot Physical Parameters ----
#define WHEELBASE_INCHES 10.75f    // Distance between wheels (inches)
#define WHEEL_DIAMETER_MM 80.0f    // Wheel diameter (mm)
#define WHEELBASE_METERS (WHEELBASE_INCHES * 0.0254f)  // Convert to meters
#define WHEELBASE_HALF (WHEELBASE_METERS / 2.0f)  // Corrected geometry

// ---- Control Parameters ----
#define ROTATION_MULTIPLIER 4.5f  // Increased rotation sensitivity
#define DEADBAND 0.10f        // Slightly smaller deadband for more responsive control
#define SPEED_SMOOTH 0.80f    // Less smoothing for more responsive feel

// ---- Encoder Parameters ----
#define ENCODER_PPR 360        // Pulses per revolution
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * PI)  // Wheel circumference in mm
#define MM_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR)  // mm per encoder pulse

// ---- Control Modes ----
enum class ControlMode {
    OPEN_LOOP,      // Direct motor control (original behavior)
    VELOCITY_PID,   // Closed-loop velocity control
    ORIENTATION_PID, // Closed-loop orientation control
    POSITION_PID    // Closed-loop position control
};

// ---- Robot State Structure ----
struct RobotState {
    // Position and orientation
    float x, y;           // Position in meters
    float heading;        // Heading in radians
    
    // Velocities
    float vx, vy, vz;     // Linear velocities in m/s
    float wx, wy, wz;     // Angular velocities in rad/s
    
    // Wheel states
    float wheel_positions[4];  // Wheel positions in meters
    float wheel_velocities[4]; // Wheel velocities in m/s
    
    // Timestamps
    uint32_t last_update_ms;
    uint32_t last_encoder_read_ms;
};

class MotionController {
public:
    MotionController();
    ~MotionController();
    
    // Initialize the motion controller with PWM setup
    void initialize();
    
    // Control mode management
    void setControlMode(ControlMode mode);
    ControlMode getControlMode() const { return _control_mode; }
    
    // Main control methods
    void drive(float forward, float strafe, float rotate);
    void driveWithHeading(float forward, float strafe, float target_heading);
    void stop();
    
    // Sensor integration
    void updateSensors();
    void updateOdometry();
    
    // PID control methods
    void enablePIDControl(bool enable);
    void setVelocityPIDGains(float kp, float ki, float kd);
    void setOrientationPIDGains(float kp, float ki, float kd);
    void resetPIDControllers();
    
    // State access
    const RobotState& getState() const { return _state; }
    float getWheelbaseInches() const { return WHEELBASE_INCHES; }
    float getWheelDiameterMm() const { return WHEEL_DIAMETER_MM; }
    float getDeadband() const { return DEADBAND; }
    
    // Debug and tuning
    void printDebugInfo();
    void printPIDStatus();

private:
    // Motor driver instances
    MotorDriver* m_frontLeft;
    MotorDriver* m_frontRight;
    MotorDriver* m_rearLeft;
    MotorDriver* m_rearRight;
    
    // Sensor instances
    EncoderDriver* m_encoderFL;
    EncoderDriver* m_encoderFR;
    EncoderDriver* m_encoderRL;
    EncoderDriver* m_encoderRR;
    IMUDriver* m_imu;
    
    // PID controllers
    PIDController* m_velocityPID[4];  // One for each wheel
    PIDController* m_orientationPID;
    
    // Control state
    ControlMode _control_mode;
    bool _pid_enabled;
    
    // Robot state
    RobotState _state;
    
    // Encoder state
    volatile int32_t m_encoderCounts[4];
    int32_t m_lastEncoderCounts[4];
    
    // Target values for closed-loop control
    float m_targetVelocities[4];  // Target wheel velocities
    float m_targetHeading;        // Target heading for orientation control
    
    // PWM configuration
    static const int PWM_FREQ = 16000;
    static const int PWM_RES = 10;
    
    // LEDC channels
    enum {
        CH_M1_R, CH_M1_L,  // Front Left
        CH_M2_R, CH_M2_L,  // Front Right
        CH_M3_R, CH_M3_L,  // Rear Left
        CH_M4_R, CH_M4_L   // Rear Right
    };
    
    // Private methods
    void _updateWheelVelocities();
    void _updateRobotPose();
    void _applyVelocityControl();
    void _applyOrientationControl();
    void _clampWheelVelocities(float& fl, float& fr, float& rl, float& rr);
    float _calculateWheelVelocity(int32_t encoder_delta, uint32_t time_delta_ms);
    void _mecanumKinematics(float vx, float vy, float wz, float* wheel_vels);
    void _mecanumInverseKinematics(float fl, float fr, float rl, float rr, float* vx, float* vy, float* wz);
};
