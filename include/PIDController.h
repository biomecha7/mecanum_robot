#pragma once
#include <Arduino.h>

// PID Controller Modes
enum class PIDMode {
    VELOCITY,    // Control wheel velocity
    ORIENTATION, // Control robot heading
    POSITION     // Control robot position
};

// PID Configuration Structure
struct PIDConfig {
    float kp;           // Proportional gain
    float ki;           // Integral gain  
    float kd;           // Derivative gain
    float max_output;   // Maximum output value
    float min_output;   // Minimum output value
    float integral_limit; // Anti-windup limit
    float deadband;     // Deadband for small errors
    uint32_t sample_time_ms; // Sample time in milliseconds
};

class PIDController {
public:
    PIDController(PIDMode mode, const PIDConfig& config);
    
    // Main control function
    float compute(float setpoint, float current_value, uint32_t current_time_ms);
    
    // Configuration methods
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min_output, float max_output);
    void setIntegralLimit(float limit);
    void setDeadband(float deadband);
    void setSampleTime(uint32_t sample_time_ms);
    
    // Control methods
    void reset();
    void enable();
    void disable();
    bool isEnabled() const { return _enabled; }
    
    // Status methods
    float getLastError() const { return _last_error; }
    float getIntegral() const { return _integral; }
    float getDerivative() const { return _derivative; }
    PIDMode getMode() const { return _mode; }
    
    // Preset configurations for common use cases
    static PIDConfig getWheelVelocityConfig();
    static PIDConfig getOrientationConfig();
    static PIDConfig getPositionConfig();

private:
    PIDMode _mode;
    PIDConfig _config;
    bool _enabled;
    
    // PID state variables
    float _last_error;
    float _integral;
    float _derivative;
    float _last_output;
    uint32_t _last_time_ms;
    
    // Helper methods
    bool _shouldCompute(uint32_t current_time_ms);
    void _clampIntegral();
    void _applyOutputLimits(float& output);
};
