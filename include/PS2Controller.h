#pragma once
#include <PSX.h>
#include "RobotPins.h"

// ---- Speed Scale Constants ----
#define SPEED_SLOW 0.35f      // L1: Slow mode for precision
#define SPEED_FAST 1.00f      // R1: Full speed mode
#define SPEED_MEDIUM 0.50f    // L2: Medium-slow mode
#define SPEED_DEFAULT 0.70f   // Default mode

class PS2Controller {
public:
    PS2Controller();
    
    // Initialize the PS2 controller
    void initialize();
    
    // Main update method - returns true if controller is connected and data is valid
    bool update();
    
    // Get current control values
    float getVx() const { return m_vx; }
    float getVy() const { return m_vy; }
    float getWz() const { return m_wz; }
    float getSpeedScale() const { return m_speed_scale; }
    
    // Check for emergency stop
    bool isEmergencyStop() const { return m_emergency_stop; }
    
    // Check connection status
    bool isConnected() const { return m_controller_connected; }
    
    // Set deadband (needed for joystick mapping)
    void setDeadband(float deadband) { m_deadband = deadband; }
    
private:
    // PS2 controller instance
    PSX m_psx;
    
    // Controller state
    bool m_controller_connected;
    uint32_t m_last_controller_read;
    
    // Control values
    float m_vx, m_vy, m_wz;
    float m_speed_scale;
    bool m_emergency_stop;
    
    // Joystick mapping parameters
    float m_deadband;
    
    // Enhanced Stick Mapping with Better Feel
    float mapStick(uint8_t rawValue, bool invert = false);
    
    // Process button inputs
    void processButtons(const PSX::PSXDATA& js);
    
    // Process joystick inputs
    void processJoysticks(const PSX::PSXDATA& js);
};
