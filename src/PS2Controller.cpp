#include "PS2Controller.h"
#include <Arduino.h>
#include <math.h>

PS2Controller::PS2Controller()
    : m_controller_connected(false), m_last_controller_read(0),
      m_vx(0.0f), m_vy(0.0f), m_wz(0.0f), m_speed_scale(SPEED_DEFAULT),
      m_emergency_stop(false), m_last_buttons(0), m_previous_buttons(0),
      m_estop_clear_start(0), m_deadband(0.10f) {}

void PS2Controller::initialize() {
    Serial.println("Initializing PS2 controller...");
    m_psx.setupPins(PIN_PS2_DAT, PIN_PS2_CMD, PIN_PS2_ATT, PIN_PS2_CLK, 10);
    m_psx.config(PSXMODE_ANALOG);
    m_psx.setRumble(0, 0);
    delay(500);
    Serial.println("PS2 controller ready!");
}

void PS2Controller::setRumble(bool on) {
    // Small motor on/off + large motor medium buzz
    m_psx.setRumble(on ? 0xFF : 0x00, on ? 0x80 : 0x00);
}

bool PS2Controller::hasMotorInput() const {
    const float eps = 0.02f;
    return fabsf(m_vx) > eps || fabsf(m_vy) > eps || fabsf(m_wz) > eps;
}

bool PS2Controller::update() {
    PSX::PSXDATA js;
    uint32_t now = millis();

    if (m_psx.read(js) == PSXERROR_SUCCESS) {
        m_controller_connected = true;
        m_last_controller_read = now;

        m_previous_buttons = m_last_buttons;
        m_last_buttons = js.buttons;

        processButtons(js);
        processJoysticks(js);
        return true;
    }

    if (now - m_last_controller_read > 100) {
        if (m_controller_connected) {
            Serial.println("Controller disconnected!");
            m_controller_connected = false;
            m_emergency_stop = true;
            m_psx.setRumble(0, 0);
        }
        return false;
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
    if (m_emergency_stop && (js.buttons & PSXBTN_SELECT) && (js.buttons & PSXBTN_START)) {
        if (m_estop_clear_start == 0) {
            m_estop_clear_start = millis();
            Serial.println("🔄 Clearing ESTOP... Hold SELECT+START for 1 second");
        } else if (millis() - m_estop_clear_start > 1000) {
            m_emergency_stop = false;
            m_estop_clear_start = 0;
            Serial.println("✅ ESTOP CLEARED! Press START to arm.");
        }
        return;
    } else {
        m_estop_clear_start = 0;
    }

    if (!m_emergency_stop && (js.buttons & PSXBTN_START) && (js.buttons & PSXBTN_TRIANGLE)) {
        m_emergency_stop = true;
        m_psx.setRumble(0, 0);
        Serial.println("🛑 EMERGENCY STOP! Press SELECT+START for 1 second to clear.");
        return;
    }

    if (!m_emergency_stop) {
        if (js.buttons & PSXBTN_L1) {
            m_speed_scale = SPEED_SLOW;
        } else if (js.buttons & PSXBTN_R1) {
            m_speed_scale = SPEED_FAST;
        } else if (js.buttons & PSXBTN_L2) {
            m_speed_scale = SPEED_MEDIUM;
        } else {
            m_speed_scale = SPEED_DEFAULT;
        }
    }
}

void PS2Controller::processJoysticks(const PSX::PSXDATA& js) {
    m_vx = mapStick(js.JoyLeftY, true);
    m_vy = mapStick(js.JoyRightX, false);
    m_wz = mapStick(js.JoyLeftX, false);

    if (js.buttons & PSXBTN_UP) {
        m_vx = 1.0f;
    } else if (js.buttons & PSXBTN_DOWN) {
        m_vx = -1.0f;
    }
    if (js.buttons & PSXBTN_LEFT) {
        m_wz = -1.0f;
    } else if (js.buttons & PSXBTN_RIGHT) {
        m_wz = 1.0f;
    }
}

float PS2Controller::mapStick(uint8_t rawValue, bool invert) {
    int centered = int(rawValue) - 128;
    float normalized = (centered >= 0) ? centered / 127.0f : centered / 128.0f;
    if (invert) normalized = -normalized;
    if (fabsf(normalized) < m_deadband) return 0.0f;

    float sign = (normalized >= 0) ? 1.0f : -1.0f;
    float scaled = (fabsf(normalized) - m_deadband) / (1.0f - m_deadband);
    scaled = scaled * scaled * sign;
    return constrain(scaled, -1.0f, 1.0f);
}
