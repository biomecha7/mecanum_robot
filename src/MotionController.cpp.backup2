#include "MotionController.h"
#include <algorithm>

MotionController::MotionController() 
    : m_frontLeft(nullptr), m_frontRight(nullptr), m_rearLeft(nullptr), m_rearRight(nullptr) {}

MotionController::~MotionController() {
    if (m_frontLeft) delete m_frontLeft;
    if (m_frontRight) delete m_frontRight;
    if (m_rearLeft) delete m_rearLeft;
    if (m_rearRight) delete m_rearRight;
}

void MotionController::initialize() {
    // Create motor driver instances
    m_frontLeft = new MotorDriver(CH_M1_R, CH_M1_L, M1_RPWM, M1_LPWM, PWM_FREQ, PWM_RES);
    m_frontRight = new MotorDriver(CH_M2_R, CH_M2_L, M2_RPWM, M2_LPWM, PWM_FREQ, PWM_RES);
    m_rearLeft = new MotorDriver(CH_M3_R, CH_M3_L, M3_RPWM, M3_LPWM, PWM_FREQ, PWM_RES);
    m_rearRight = new MotorDriver(CH_M4_R, CH_M4_L, M4_RPWM, M4_LPWM, PWM_FREQ, PWM_RES);
}

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
