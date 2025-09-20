#include "PS2Controller.h"
#include <Arduino.h>

PS2Controller::PS2Controller() 
    : m_controller_connected(false), m_last_controller_read(0),
      m_vx(0.0f), m_vy(0.0f), m_wz(0.0f), m_speed_scale(SPEED_DEFAULT),
      m_emergency_stop(false), m_last_buttons(0), m_previous_buttons(0), 
      m_estop_clear_start(0), m_deadband(0.10f) {}

void PS2Controller::initialize() {
    Serial.println("Initializing PS2 controller...");
    m_psx.setupPins(PIN_PS2_DAT, PIN_PS2_CMD, PIN_PS2_ATT, PIN_PS2_CLK, 10);
    m_psx.config(PSXMODE_ANALOG);
    delay(500);
    Serial.println("PS2 controller ready!");
}

bool PS2Controller::update() {
    PSX::PSXDATA js;
    uint32_t now = millis();
    
    // Try to read controller
    int result = m_psx.read(js);
    
    // Debug PS2 read result every 2 seconds
    static uint32_t last_debug = 0;
    if (now - last_debug >= 2000) {
        Serial.printf("🔧 PS2 Debug: Read_result=%d, Connected=%s, Estop=%s, Last_read=%ums ago\n",
                      result, m_controller_connected ? "YES" : "NO", 
                      m_emergency_stop ? "YES" : "NO",
                      now - m_last_controller_read);
        last_debug = now;
    }
    
    if (result == PSXERROR_SUCCESS) {
        m_controller_connected = true;
        m_last_controller_read = now;
        
        // Store previous button state
        m_previous_buttons = m_last_buttons;
        m_last_buttons = js.buttons;
        
        // Process controller inputs
        processButtons(js);
        processJoysticks(js);
        
        return true;
    } else {
        // Controller timeout handling
        if (now - m_last_controller_read > 100) {  // 100ms timeout
            if (m_controller_connected) {
                Serial.println("Controller disconnected!");
                m_controller_connected = false;
                // Only set emergency stop if controller was previously connected
                // This prevents auto-ESTOP when controller is physically disconnected
                m_emergency_stop = true;
            }
            return false;
        }
    }
    
    return false;
}

bool PS2Controller::getButtonPressed(uint16_t button) const {
    return (m_last_buttons & button) && !(m_previous_buttons & button);
}

bool PS2Controller::getButtonReleased(uint16_t button) const {
    return !(m_last_buttons & button) && (m_previous_buttons & button);
}

void PS2Controller::processButtons(const PSX::PSXDATA& js) {
    // ESTOP Clear: SELECT + START held for 1 second (reduced from 2)
    if (m_emergency_stop && (js.buttons & PSXBTN_SELECT) && (js.buttons & PSXBTN_START)) {
        if (m_estop_clear_start == 0) {
            m_estop_clear_start = millis();
            Serial.println("🔄 Clearing ESTOP... Hold SELECT+START for 1 second");
        } else if (millis() - m_estop_clear_start > 1000) {
            m_emergency_stop = false;
            m_estop_clear_start = 0;
            Serial.println("✅ ESTOP CLEARED! Robot ready.");
        }
        return; // Don't process other buttons during ESTOP clear
    } else {
        m_estop_clear_start = 0; // Reset if buttons released
    }
    
    // Emergency Stop: Changed to require START button instead of SELECT to avoid accidental triggers
    if (!m_emergency_stop && (js.buttons & PSXBTN_START) && (js.buttons & PSXBTN_TRIANGLE)) {
        m_emergency_stop = true;
        Serial.println("🛑 EMERGENCY STOP! Press SELECT+START for 1 second to clear.");
        return;
    }
    
    // Speed Mode Control (only if not in ESTOP)
    if (!m_emergency_stop) {
        if (js.buttons & PSXBTN_L1) {
            m_speed_scale = SPEED_SLOW;  // Slow mode for precision
        } else if (js.buttons & PSXBTN_R1) {
            m_speed_scale = SPEED_FAST;  // Full speed mode
        } else if (js.buttons & PSXBTN_L2) {
            m_speed_scale = SPEED_MEDIUM;  // Medium-slow mode
        } else {
            m_speed_scale = SPEED_DEFAULT;  // Default mode
        }
    }
}

void PS2Controller::processJoysticks(const PSX::PSXDATA& js) {
    // Get joystick commands
    m_vx = mapStick(js.JoyLeftY, true);   // Forward/backward (inverted)
    m_vy = mapStick(js.JoyRightX, false); // Left/right strafe (swapped)
    m_wz = mapStick(js.JoyLeftX, false);  // Rotation (swapped)
}

float PS2Controller::mapStick(uint8_t rawValue, bool invert) {
    // Convert 0-255 to -128 to +127
    int centered = int(rawValue) - 128;
    
    // Normalize to -1.0 to +1.0
    float normalized = (centered >= 0) ? centered / 127.0f : centered / 128.0f;
    
    // Apply inversion if requested
    if (invert) normalized = -normalized;
    
    // Apply deadband
    if (fabsf(normalized) < m_deadband) return 0.0f;
    
    // Scale remaining range to maintain full -1 to +1 output
    float sign = (normalized >= 0) ? 1.0f : -1.0f;
    float scaled = (fabsf(normalized) - m_deadband) / (1.0f - m_deadband);
    
    // Apply slight exponential curve for finer control near center
    scaled = scaled * scaled * sign;
    
    return constrain(scaled, -1.0f, 1.0f);
}
