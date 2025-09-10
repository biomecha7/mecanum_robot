#pragma once
#include "MotorDriver.h"
#include "RobotPins.h"

class MotionController {
public:
    MotionController();
    ~MotionController();
    
    // Initialize the motion controller with PWM setup
    void initialize();
    
    void drive(float forward, float strafe, float rotate);
    void stop();
    
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
