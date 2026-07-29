#pragma once
#include "MotorDriver.h"
#include "RobotPins.h"

#define DEADBAND 0.10f

class MotionController {
public:
    MotionController();
    ~MotionController();

    void initialize();
    void drive(float forward, float strafe, float rotate);
    void stop();
    float getDeadband() const { return DEADBAND; }

private:
    MotorDriver* m_frontLeft{nullptr};
    MotorDriver* m_frontRight{nullptr};
    MotorDriver* m_rearLeft{nullptr};
    MotorDriver* m_rearRight{nullptr};

    static const int PWM_FREQ = 16000;
    static const int PWM_RES = 10;

    enum {
        CH_M1_R, CH_M1_L,
        CH_M2_R, CH_M2_L,
        CH_M3_R, CH_M3_L,
        CH_M4_R, CH_M4_L
    };

    static void clamp(float& v) {
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
    }
};
