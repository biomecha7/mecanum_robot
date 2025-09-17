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
// conservative → sporty
static constexpr float MAX_VX_MPS  = 0.80f;   // was 0.40
static constexpr float MAX_VY_MPS  = 0.80f;   // was 0.40
static constexpr float MAX_WZ_RAD  = 2.50f;   // was 1.50


MotionController::MotionController(EncoderTask& encoder_task) 
    : _encoder_task(encoder_task),
      m_frontLeft(nullptr), m_frontRight(nullptr), m_rearLeft(nullptr), m_rearRight(nullptr),
      _control_mode(ControlMode::OPEN_LOOP), _pid_enabled(false),
      m_targetHeading(0.0f) {
    
    // Initialize target velocities
    for (int i = 0; i < 4; i++) {
        m_targetVelocities[i] = 0.0f;
        m_wheelVelFilt[i] = 0.0f;
    }
    
    // Initialize robot state
    _state = {};
    _state.last_update_ms = 0;
    _state.last_encoder_read_ms = 0;
}

MotionController::~MotionController() {
    // Clean up motor drivers
    if (m_frontLeft) delete m_frontLeft;
    if (m_frontRight) delete m_frontRight;
    if (m_rearLeft) delete m_rearLeft;
    if (m_rearRight) delete m_rearRight;
    
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
    
    Serial.println("✅ MotionController initialized (using EncoderTask for sensor data)");
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
        // Get encoder data from EncoderTask (atomic read)
        EncoderAtomicData encoder_data;
        if (_encoder_task.getAtomicData(encoder_data)) {
            // Copy velocities and apply additional filtering for PID
            const float alpha = 0.25f; // 0..1  (0.2-0.4 works well)
            for (int i = 0; i < 4; i++) {
                _state.wheel_velocities[i] = encoder_data.velocities_ms[i];
                m_wheelVelFilt[i] = alpha * _state.wheel_velocities[i] + (1.0f - alpha) * m_wheelVelFilt[i];
            }
        } else {
            // EncoderTask data not available, set velocities to zero
            for (int i = 0; i < 4; i++) {
                _state.wheel_velocities[i] = 0.0f;
                m_wheelVelFilt[i] = 0.0f;
            }
        }
        
        _state.last_encoder_read_ms = current_time;
    }
}

void MotionController::_updateRobotPose() {
    uint32_t current_time = millis();
    float dt = (current_time - _state.last_update_ms) / 1000.0f;
    
    if (dt > 0) {
        // Get encoder data from EncoderTask (atomic read)
        EncoderAtomicData encoder_data;
        if (_encoder_task.getAtomicData(encoder_data)) {
            // Update wheel positions from encoder data
            for (int i = 0; i < 4; i++) {
                _state.wheel_positions[i] = encoder_data.positions_rad[i];
            }
        }
        
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
        
        // Note: Heading update from IMU will be handled by dedicated IMU task
        // For now, heading is not updated from IMU data
        
        _state.last_update_ms = current_time;
    }
}

void MotionController::_applyVelocityControl() {
    if (!_pid_enabled) return;
    
    uint32_t current_time = millis();
    
    for (int i = 0; i < 4; i++) {
        if (m_velocityPID[i]) {
            const float kS = 0.03f;   // static bump (duty fraction)
            const float kV = 0.90f;   // duty per (m/s) — tune later

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
    
    // Get encoder counts from EncoderTask
    int32_t encoder_counts[4];
    getEncoderCounts(encoder_counts);
    Serial.printf("Encoder Counts: [%ld, %ld, %ld, %ld]\n",
                  encoder_counts[0], encoder_counts[1], encoder_counts[2], encoder_counts[3]);
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

void MotionController::getEncoderCounts(int32_t counts[4]) const {
    EncoderAtomicData encoder_data;
    if (_encoder_task.getAtomicData(encoder_data)) {
        for (int i = 0; i < 4; i++) {
            counts[i] = encoder_data.counts[i];
        }
    } else {
        for (int i = 0; i < 4; i++) {
            counts[i] = 0;
        }
    }
}

void MotionController::getWheelPositions(float positions[4]) const {
    EncoderAtomicData encoder_data;
    if (_encoder_task.getAtomicData(encoder_data)) {
        for (int i = 0; i < 4; i++) {
            positions[i] = encoder_data.positions_rad[i];
        }
    } else {
        for (int i = 0; i < 4; i++) {
            positions[i] = 0.0f;
        }
    }
}

void MotionController::getWheelVelocities(float velocities[4]) const {
    for (int i = 0; i < 4; i++) {
        velocities[i] = _state.wheel_velocities[i];
    }
}
