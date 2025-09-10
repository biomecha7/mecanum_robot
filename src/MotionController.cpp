#include "MotionController.h"
#include <algorithm>
#include <math.h>

// Constants for calculations
// --- Robot physical params (meters) ---
static constexpr float WHEEL_DIAMETER_M = 0.080f;                 // 80 mm
static constexpr float WHEEL_RADIUS_M   = WHEEL_DIAMETER_M * 0.5f;

// From MotionController.h: 10.75" square → wheelbase = track
static constexpr float WHEELBASE_M = WHEELBASE_METERS;            // from header
static constexpr float TRACK_M     = WHEELBASE_METERS;            // assume square
static constexpr float L_EFFECT    = 0.5f*WHEELBASE_M + 0.5f*TRACK_M;

// Map stick [-1,1] to body-frame speeds (start conservative)
static constexpr float MAX_VX_MPS  = 0.40f;
static constexpr float MAX_VY_MPS  = 0.40f;
static constexpr float MAX_WZ_RAD  = 1.50f;

MotionController::MotionController() 
    : m_frontLeft(nullptr), m_frontRight(nullptr), m_rearLeft(nullptr), m_rearRight(nullptr),
      m_encoderFL(nullptr), m_encoderFR(nullptr), m_encoderRL(nullptr), m_encoderRR(nullptr),
      m_imu(nullptr), _control_mode(ControlMode::OPEN_LOOP), _pid_enabled(false),
      m_targetHeading(0.0f) {
    
    // Initialize encoder counts
    for (int i = 0; i < 4; i++) {
        m_encoderCounts[i] = 0;
        m_lastEncoderCounts[i] = 0;
        m_targetVelocities[i] = 0.0f;
    }
    
    // Initialize robot state
    _state = {};
    _state.last_update_ms = 0;
    _state.last_encoder_read_ms = 0;

    for (int i = 0; i < 4; ++i) {
        m_wheelVelFilt[i] = 0.0f;
    }

}

MotionController::~MotionController() {
    // Clean up motor drivers
    if (m_frontLeft) delete m_frontLeft;
    if (m_frontRight) delete m_frontRight;
    if (m_rearLeft) delete m_rearLeft;
    if (m_rearRight) delete m_rearRight;
    
    // Clean up encoders
    if (m_encoderFL) delete m_encoderFL;
    if (m_encoderFR) delete m_encoderFR;
    if (m_encoderRL) delete m_encoderRL;
    if (m_encoderRR) delete m_encoderRR;
    
    // Clean up IMU
    if (m_imu) delete m_imu;
    
    // Clean up PID controllers
    for (int i = 0; i < 4; i++) {
        if (m_velocityPID[i]) delete m_velocityPID[i];
    }
    if (m_orientationPID) delete m_orientationPID;
}

void MotionController::initialize() {
    // Create motor driver instances
    m_frontLeft = new MotorDriver(CH_M1_R, CH_M1_L, M1_RPWM, M1_LPWM, PWM_FREQ, PWM_RES);
    m_frontRight = new MotorDriver(CH_M2_R, CH_M2_L, M2_RPWM, M2_LPWM, PWM_FREQ, PWM_RES);
    m_rearLeft = new MotorDriver(CH_M3_R, CH_M3_L, M3_RPWM, M3_LPWM, PWM_FREQ, PWM_RES);
    m_rearRight = new MotorDriver(CH_M4_R, CH_M4_L, M4_RPWM, M4_LPWM, PWM_FREQ, PWM_RES);
    
    // Create encoder instances
    m_encoderFL = new EncoderDriver(ENC_M1_A, ENC_M1_B, &m_encoderCounts[0]);
    m_encoderFR = new EncoderDriver(ENC_M2_A, ENC_M2_B, &m_encoderCounts[1]);
    m_encoderRL = new EncoderDriver(ENC_M3_A, ENC_M3_B, &m_encoderCounts[2]);
    m_encoderRR = new EncoderDriver(ENC_M4_A, ENC_M4_B, &m_encoderCounts[3]);
    
    // Initialize encoders
    m_encoderFL->begin();
    m_encoderFR->begin();
    m_encoderRL->begin();
    m_encoderRR->begin();
    
    // Create IMU instance
    m_imu = new IMUDriver(IMU_SDA, IMU_SCL);
    m_imu->begin();
    
    // Create PID controllers
    PIDConfig velocityConfig = PIDController::getWheelVelocityConfig();
    for (int i = 0; i < 4; i++) {
        m_velocityPID[i] = new PIDController(PIDMode::VELOCITY, velocityConfig);
    }
    
    PIDConfig orientationConfig = PIDController::getOrientationConfig();
    m_orientationPID = new PIDController(PIDMode::ORIENTATION, orientationConfig);
    
    // Initialize state
    _state.last_update_ms = millis();
    _state.last_encoder_read_ms = millis();
}

void MotionController::setControlMode(ControlMode mode) {
    _control_mode = mode;
    
    // Reset PID controllers when changing modes
    if (mode != ControlMode::OPEN_LOOP) {
        resetPIDControllers();
    }
}

void MotionController::drive(float forward, float strafe, float rotate) {
    if (_control_mode == ControlMode::OPEN_LOOP) {
        // Original open-loop control
        float fl = forward + strafe + rotate;
        float fr = forward - strafe - rotate;
        float rl = forward - strafe + rotate;
        float rr = forward + strafe - rotate;
        
        _clampWheelVelocities(fl, fr, rl, rr);
        
        m_frontLeft->setSpeed(fl);
        m_frontRight->setSpeed(fr);
        m_rearLeft->setSpeed(rl);
        m_rearRight->setSpeed(rr);
    } else {
        // Closed-loop control - set target velocities
        _mecanumKinematics(forward, strafe, rotate, m_targetVelocities);
        
        if (_control_mode == ControlMode::VELOCITY_PID) {
            _applyVelocityControl();
        } else if (_control_mode == ControlMode::ORIENTATION_PID) {
            _applyOrientationControl();
        }
    }
}

void MotionController::driveWithHeading(float forward, float strafe, float target_heading) {
    m_targetHeading = target_heading;

    if (_control_mode == ControlMode::ORIENTATION_PID) {
        // Compute heading error to [-pi, pi]
        float heading_error = target_heading - _state.heading;
        while (heading_error > PI)  heading_error -= 2*PI;
        while (heading_error < -PI) heading_error += 2*PI;

        // Let orientation PID produce a wz_correction
        float wz_correction = m_orientationPID->compute(0.0f, heading_error, millis());

        // Feed orientation as wz through kinematics so wheel signs are correct
        _mecanumKinematics(forward, strafe, wz_correction, m_targetVelocities);
        _applyVelocityControl();   // per-wheel velocity PIDs in m/s → duty
    } else {
        drive(forward, strafe, 0.0f);
    }
}

void MotionController::stop() {
    for (int i = 0; i < 4; i++) {
        m_targetVelocities[i] = 0.0f;
    }
    
    m_frontLeft->stop();
    m_frontRight->stop();
    m_rearLeft->stop();
    m_rearRight->stop();
}

void MotionController::updateSensors() {
    // Update IMU
    if (m_imu) {
        m_imu->update();
    }
    
    // Update wheel velocities from encoders
    _updateWheelVelocities();
    
    // Update robot pose
    _updateRobotPose();
}

void MotionController::updateOdometry() {
    updateSensors();
}

void MotionController::enablePIDControl(bool enable) {
    _pid_enabled = enable;
    
    if (!enable) {
        // Reset all PID controllers
        resetPIDControllers();
    }
}

void MotionController::setVelocityPIDGains(float kp, float ki, float kd) {
    for (int i = 0; i < 4; i++) {
        if (m_velocityPID[i]) {
            m_velocityPID[i]->setGains(kp, ki, kd);
        }
    }
}

void MotionController::setOrientationPIDGains(float kp, float ki, float kd) {
    if (m_orientationPID) {
        m_orientationPID->setGains(kp, ki, kd);
    }
}

void MotionController::resetPIDControllers() {
    for (int i = 0; i < 4; i++) {
        if (m_velocityPID[i]) m_velocityPID[i]->reset();
        m_wheelVelFilt[i] = 0.0f;
    }
    if (m_orientationPID) m_orientationPID->reset();
}

void MotionController::_updateWheelVelocities() {
    uint32_t current_time = millis();
    uint32_t time_delta = current_time - _state.last_encoder_read_ms;
    
    if (time_delta > 0) {
        for (int i = 0; i < 4; i++) {
            int32_t encoder_delta = m_encoderCounts[i] - m_lastEncoderCounts[i];
            _state.wheel_velocities[i] = _calculateWheelVelocity(encoder_delta, time_delta);
            const float alpha = 0.25f; // 0..1  (0.2-0.4 works well)
            m_wheelVelFilt[i] = alpha * _state.wheel_velocities[i] + (1.0f - alpha) * m_wheelVelFilt[i];
            m_lastEncoderCounts[i] = m_encoderCounts[i];
        }
        _state.last_encoder_read_ms = current_time;
    }
}

void MotionController::_updateRobotPose() {
    uint32_t current_time = millis();
    float dt = (current_time - _state.last_update_ms) / 1000.0f;
    
    if (dt > 0) {
        // Update position based on wheel velocities
        float avg_vx = (_state.wheel_velocities[0] + _state.wheel_velocities[1] + 
                       _state.wheel_velocities[2] + _state.wheel_velocities[3]) / 4.0f;
        float avg_vy = (-_state.wheel_velocities[0] + _state.wheel_velocities[1] + 
                       _state.wheel_velocities[2] - _state.wheel_velocities[3]) / 4.0f;
        
        // Rotate velocities to global frame
        float cos_h = cos(_state.heading);
        float sin_h = sin(_state.heading);
        
        _state.x += (avg_vx * cos_h - avg_vy * sin_h) * dt;
        _state.y += (avg_vx * sin_h + avg_vy * cos_h) * dt;
        
        // Update heading from IMU
        if (m_imu) {
            const IMUData& imu_data = m_imu->getData();
            if (imu_data.data_ready) {
                _state.heading += imu_data.gyro_z * DEG_TO_RAD * dt;
                // Normalize heading to [-PI, PI]
                while (_state.heading > PI) _state.heading -= 2*PI;
                while (_state.heading < -PI) _state.heading += 2*PI;
            }
        }
        
        _state.last_update_ms = current_time;
    }
}

void MotionController::_applyVelocityControl() {
    if (!_pid_enabled) return;
    
    uint32_t current_time = millis();
    
    for (int i = 0; i < 4; i++) {
        if (m_velocityPID[i]) {
            const float kS = 0.03f;   // static bump (duty fraction)
            const float kV = 0.50f;   // duty per (m/s) — tune later

            float v_set = m_targetVelocities[i];   // m/s
            float v_meas = m_wheelVelFilt[i];      // m/s

            float u_pid = m_velocityPID[i]->compute(v_set, v_meas, current_time);
            float u_ff  = (v_set != 0.0f ? kS * (v_set > 0 ? 1.0f : -1.0f) : 0.0f) + kV * v_set;
            float u = u_ff + u_pid;

            // clamp and send
            if (u > 1.0f) u = 1.0f;
            if (u < -1.0f) u = -1.0f;
            
            // Apply PID output to motor
            switch (i) {
                case 0: m_frontLeft->setSpeed(u); break;
                case 1: m_frontRight->setSpeed(u); break;
                case 2: m_rearLeft->setSpeed(u); break;
                case 3: m_rearRight->setSpeed(u); break;
            }
        }
    }
}

void MotionController::_applyOrientationControl() {
    if (!_pid_enabled) return;
    
    // This is handled in driveWithHeading method
    _applyVelocityControl();
}

void MotionController::_clampWheelVelocities(float& fl, float& fr, float& rl, float& rr) {
    fl = std::max(-1.0f, std::min(1.0f, fl));
    fr = std::max(-1.0f, std::min(1.0f, fr));
    rl = std::max(-1.0f, std::min(1.0f, rl));
    rr = std::max(-1.0f, std::min(1.0f, rr));
}

float MotionController::_calculateWheelVelocity(int32_t encoder_delta, uint32_t time_delta_ms) {
    if (time_delta_ms == 0) return 0.0f;
    
    float distance_mm = encoder_delta * MM_PER_PULSE;
    float velocity_mm_per_s = (distance_mm * 1000.0f) / time_delta_ms;
    return velocity_mm_per_s / 1000.0f; // Convert to m/s
}

void MotionController::_mecanumKinematics(float forward_cmd, float strafe_cmd, float rotate_cmd, float* wheel_vels) {
    // Map joystick [-1,1] to physical body velocities
    const float vx = forward_cmd * MAX_VX_MPS;   // m/s
    const float vy = strafe_cmd  * MAX_VY_MPS;   // m/s
    const float wz = rotate_cmd  * MAX_WZ_RAD;   // rad/s

    // Per-wheel **linear** speeds [m/s], using standard mecanum signs
    wheel_vels[0] = vx + vy + L_EFFECT * wz;  // FL
    wheel_vels[1] = vx - vy - L_EFFECT * wz;  // FR
    wheel_vels[2] = vx - vy + L_EFFECT * wz;  // RL
    wheel_vels[3] = vx + vy - L_EFFECT * wz;  // RR
    // NOTE: no clamping here; clamp at final duty (MotorDriver::set)
}

void MotionController::_mecanumInverseKinematics(float fl, float fr, float rl, float rr, float* vx, float* vy, float* wz) {
    *vx = (fl + fr + rl + rr) / 4.0f;
    *vy = (fl - fr - rl + rr) / 4.0f;
    *wz = (fl - fr + rl - rr) / (4.0f * L_EFFECT);
}

void MotionController::printDebugInfo() {
    Serial.println("=== Robot State ===");
    Serial.printf("Position: (%.3f, %.3f) m\n", _state.x, _state.y);
    Serial.printf("Heading: %.1f deg\n", _state.heading * RAD_TO_DEG);
    Serial.printf("Wheel Velocities: [%.3f, %.3f, %.3f, %.3f] m/s\n", 
                  _state.wheel_velocities[0], _state.wheel_velocities[1],
                  _state.wheel_velocities[2], _state.wheel_velocities[3]);
    Serial.printf("Encoder Counts: [%ld, %ld, %ld, %ld]\n",
                  m_encoderCounts[0], m_encoderCounts[1], m_encoderCounts[2], m_encoderCounts[3]);
}

void MotionController::printPIDStatus() {
    Serial.println("=== PID Status ===");
    for (int i = 0; i < 4; i++) {
        if (m_velocityPID[i]) {
            Serial.printf("Wheel %d PID: Error=%.3f, Integral=%.3f, Derivative=%.3f\n",
                         i, m_velocityPID[i]->getLastError(), 
                         m_velocityPID[i]->getIntegral(), m_velocityPID[i]->getDerivative());
        }
    }
    if (m_orientationPID) {
        Serial.printf("Orientation PID: Error=%.3f, Integral=%.3f, Derivative=%.3f\n",
                     m_orientationPID->getLastError(), m_orientationPID->getIntegral(), 
                     m_orientationPID->getDerivative());
    }
}
