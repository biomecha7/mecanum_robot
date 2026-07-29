#include "MotionController.h"
#include <Arduino.h>

MotionController::MotionController() = default;

MotionController::~MotionController() {
    delete m_frontLeft;
    delete m_frontRight;
    delete m_rearLeft;
    delete m_rearRight;
}

void MotionController::initialize() {
    m_frontLeft  = new MotorDriver(CH_M1_R, CH_M1_L, M1_RPWM, M1_LPWM, PWM_FREQ, PWM_RES);
    m_frontRight = new MotorDriver(CH_M2_R, CH_M2_L, M2_RPWM, M2_LPWM, PWM_FREQ, PWM_RES);
    m_rearLeft   = new MotorDriver(CH_M3_R, CH_M3_L, M3_RPWM, M3_LPWM, PWM_FREQ, PWM_RES);
    m_rearRight  = new MotorDriver(CH_M4_R, CH_M4_L, M4_RPWM, M4_LPWM, PWM_FREQ, PWM_RES);
    stop();
    Serial.println("✅ MotionController ready (open-loop)");
}

void MotionController::drive(float forward, float strafe, float rotate) {
    float fl = forward + strafe + rotate;
    float fr = forward - strafe - rotate;
    float rl = forward - strafe + rotate;
    float rr = forward + strafe - rotate;

    clamp(fl); clamp(fr); clamp(rl); clamp(rr);

    m_frontLeft->setSpeed(fl);
    m_frontRight->setSpeed(fr);
    m_rearLeft->setSpeed(rl);
    m_rearRight->setSpeed(rr);
}

void MotionController::stop() {
    if (m_frontLeft)  m_frontLeft->stop();
    if (m_frontRight) m_frontRight->stop();
    if (m_rearLeft)   m_rearLeft->stop();
    if (m_rearRight)  m_rearRight->stop();
}
