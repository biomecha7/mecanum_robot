#include "PIDController.h"

// Constants for different control modes
const float WHEEL_VELOCITY_KP = 0.8f;
const float WHEEL_VELOCITY_KI = 0.10f;
const float WHEEL_VELOCITY_KD = 0.0f;
const float WHEEL_VELOCITY_MAX_OUTPUT = 1.0f;
const float WHEEL_VELOCITY_MIN_OUTPUT = -1.0f;
const float WHEEL_VELOCITY_INTEGRAL_LIMIT = 2.0f;
const uint32_t WHEEL_VELOCITY_SAMPLE_TIME_MS = 20;  // from 10 → 20 ms
const float WHEEL_VELOCITY_DEADBAND = 0.02f;        // ignore tiny errors in m/s

const float ORIENTATION_KP = 1.5f;
const float ORIENTATION_KI = 0.2f;
const float ORIENTATION_KD = 0.3f;
const float ORIENTATION_MAX_OUTPUT = 0.8f;
const float ORIENTATION_MIN_OUTPUT = -0.8f;
const float ORIENTATION_INTEGRAL_LIMIT = 1.0f;
const float ORIENTATION_DEADBAND = 0.02f;
const uint32_t ORIENTATION_SAMPLE_TIME_MS = 20;

const float POSITION_KP = 1.0f;
const float POSITION_KI = 0.1f;
const float POSITION_KD = 0.2f;
const float POSITION_MAX_OUTPUT = 0.6f;
const float POSITION_MIN_OUTPUT = -0.6f;
const float POSITION_INTEGRAL_LIMIT = 0.5f;
const float POSITION_DEADBAND = 0.05f;
const uint32_t POSITION_SAMPLE_TIME_MS = 50;

PIDController::PIDController(PIDMode mode, const PIDConfig& config)
    : _mode(mode), _config(config), _enabled(true), _last_error(0.0f), 
      _integral(0.0f), _derivative(0.0f), _last_output(0.0f), _last_time_ms(0) {
}

float PIDController::compute(float setpoint, float current_value, uint32_t current_time_ms) {
    if (!_enabled) {
        return 0.0f;
    }
    
    // Check if enough time has passed for this sample
    if (!_shouldCompute(current_time_ms)) {
        return _last_output;
    }
    
    // Calculate error
    float error = setpoint - current_value;

    // Apply deadband
    if (abs(error) < _config.deadband) error = 0.0f;

    float dt = (_last_time_ms > 0) ? (current_time_ms - _last_time_ms) / 1000.0f : 0.0f;

    // Proportional
    float proportional = _config.kp * error;

    // Derivative (on error is fine here)
    if (dt > 0) _derivative = (error - _last_error) / dt;
    else        _derivative = 0.0f;
    float derivative_term = _config.kd * _derivative;

    // ---- Anti-windup guarded integral (update ONCE) ----
    if (dt > 0) {
    // predict output without the new integral to test saturation direction
    float tentative_output_noI = proportional + _config.kd * _derivative;
    // if we are saturating and error pushes further into saturation, skip integrating
    bool pushing_into_sat_hi = (tentative_output_noI >= _config.max_output && error > 0);
    bool pushing_into_sat_lo = (tentative_output_noI <= _config.min_output && error < 0);
    if (!(pushing_into_sat_hi || pushing_into_sat_lo)) {
        _integral += error * dt;
        _clampIntegral();
    }
    }
    float integral_term = _config.ki * _integral;

    // Output + clamp
    float output = proportional + integral_term + derivative_term;
    _applyOutputLimits(output);

    // Bookkeeping
    _last_error = error;
    _last_output = output;
    _last_time_ms = current_time_ms;

    return output;
}

void PIDController::setGains(float kp, float ki, float kd) {
    _config.kp = kp;
    _config.ki = ki;
    _config.kd = kd;
}

void PIDController::setOutputLimits(float min_output, float max_output) {
    _config.min_output = min_output;
    _config.max_output = max_output;
}

void PIDController::setIntegralLimit(float limit) {
    _config.integral_limit = limit;
}

void PIDController::setDeadband(float deadband) {
    _config.deadband = deadband;
}

void PIDController::setSampleTime(uint32_t sample_time_ms) {
    _config.sample_time_ms = sample_time_ms;
}

void PIDController::reset() {
    _last_error = 0.0f;
    _integral = 0.0f;
    _derivative = 0.0f;
    _last_output = 0.0f;
    _last_time_ms = 0;
}

void PIDController::enable() {
    _enabled = true;
}

void PIDController::disable() {
    _enabled = false;
}

bool PIDController::_shouldCompute(uint32_t current_time_ms) {
    if (_last_time_ms == 0) {
        return true;
    }
    return (current_time_ms - _last_time_ms) >= _config.sample_time_ms;
}

void PIDController::_clampIntegral() {
    if (_integral > _config.integral_limit) {
        _integral = _config.integral_limit;
    } else if (_integral < -_config.integral_limit) {
        _integral = -_config.integral_limit;
    }
}

void PIDController::_applyOutputLimits(float& output) {
    if (output > _config.max_output) {
        output = _config.max_output;
    } else if (output < _config.min_output) {
        output = _config.min_output;
    }
}

// Static factory methods for preset configurations
PIDConfig PIDController::getWheelVelocityConfig() {
    PIDConfig config;
    config.kp = WHEEL_VELOCITY_KP;
    config.ki = WHEEL_VELOCITY_KI;
    config.kd = WHEEL_VELOCITY_KD;
    config.max_output = WHEEL_VELOCITY_MAX_OUTPUT;
    config.min_output = WHEEL_VELOCITY_MIN_OUTPUT;
    config.integral_limit = WHEEL_VELOCITY_INTEGRAL_LIMIT;
    config.deadband = WHEEL_VELOCITY_DEADBAND;
    config.sample_time_ms = WHEEL_VELOCITY_SAMPLE_TIME_MS;
    return config;
}

PIDConfig PIDController::getOrientationConfig() {
    PIDConfig config;
    config.kp = ORIENTATION_KP;
    config.ki = ORIENTATION_KI;
    config.kd = ORIENTATION_KD;
    config.max_output = ORIENTATION_MAX_OUTPUT;
    config.min_output = ORIENTATION_MIN_OUTPUT;
    config.integral_limit = ORIENTATION_INTEGRAL_LIMIT;
    config.deadband = ORIENTATION_DEADBAND;
    config.sample_time_ms = ORIENTATION_SAMPLE_TIME_MS;
    return config;
}

PIDConfig PIDController::getPositionConfig() {
    PIDConfig config;
    config.kp = POSITION_KP;
    config.ki = POSITION_KI;
    config.kd = POSITION_KD;
    config.max_output = POSITION_MAX_OUTPUT;
    config.min_output = POSITION_MIN_OUTPUT;
    config.integral_limit = POSITION_INTEGRAL_LIMIT;
    config.deadband = POSITION_DEADBAND;
    config.sample_time_ms = POSITION_SAMPLE_TIME_MS;
    return config;
}
