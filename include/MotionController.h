#pragma once
#include "MotorDriver.h"

class MotionController {
public:
    MotionController(MotorDriver* frontLeft, MotorDriver* frontRight, MotorDriver* rearLeft, MotorDriver* rearRight);
    void drive(float forward, float strafe, float rotate);
private:
    MotorDriver* m_frontLeft;
    MotorDriver* m_frontRight;
    MotorDriver* m_rearLeft;
    MotorDriver* m_rearRight;
};
