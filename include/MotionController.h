#pragma once
#include "MotorDriver.h"
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

class MotionController {
public:
    MotionController();
    ~MotionController();
    
    // Initialize the motion controller with PWM setup
    void initialize();
    
    // Main control methods
    void drive(float forward, float strafe, float rotate);
    void stop();
    
    // Getters for parameters (useful for status reporting)
    float getWheelbaseInches() const { return WHEELBASE_INCHES; }
    float getWheelDiameterMm() const { return WHEEL_DIAMETER_MM; }
    float getDeadband() const { return DEADBAND; }
    
private:
    // Motor driver instances
    MotorDriver* m_frontLeft;
    MotorDriver* m_frontRight;
    MotorDriver* m_rearLeft;
    MotorDriver* m_rearRight;
    
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
};
