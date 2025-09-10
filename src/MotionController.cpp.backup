#include "MotionController.h"
#include <algorithm>

MotionController::MotionController(MotorDriver* frontLeft, MotorDriver* frontRight, MotorDriver* rearLeft, MotorDriver* rearRight)
    : m_frontLeft(frontLeft), m_frontRight(frontRight), m_rearLeft(rearLeft), m_rearRight(rearRight) {}

void MotionController::drive(float forward, float strafe, float rotate) {
    // Mecanum wheel kinematics
    float fl = forward + strafe + rotate;
    float fr = forward - strafe - rotate;
    float rl = forward - strafe + rotate;
    float rr = forward + strafe - rotate;

    // Clamp values to [-1, 1]
    fl = std::max(-1.0f, std::min(1.0f, fl));
    fr = std::max(-1.0f, std::min(1.0f, fr));
    rl = std::max(-1.0f, std::min(1.0f, rl));
    rr = std::max(-1.0f, std::min(1.0f, rr));

    m_frontLeft->setSpeed(fl);
    m_frontRight->setSpeed(fr);
    m_rearLeft->setSpeed(rl);
    m_rearRight->setSpeed(rr);
}

void MotionController::stop() {
    m_frontLeft->stop();
    m_frontRight->stop();
    m_rearLeft->stop();
    m_rearRight->stop();
}
