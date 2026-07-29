#pragma once
#include <PSX.h>
#include "RobotPins.h"

#define SPEED_SLOW 0.35f
#define SPEED_FAST 1.00f
#define SPEED_MEDIUM 0.50f
#define SPEED_DEFAULT 0.70f

class PS2Controller {
public:
    PS2Controller();

    void initialize();
    bool update();

    float getVx() const { return m_vx; }
    float getVy() const { return m_vy; }
    float getWz() const { return m_wz; }
    float getSpeedScale() const { return m_speed_scale; }

    bool isEmergencyStop() const { return m_emergency_stop; }
    void clearEmergencyStop() { m_emergency_stop = false; }
    bool isConnected() const { return m_controller_connected; }

    void setDeadband(float deadband) { m_deadband = deadband; }
    void setRumble(bool on);

    /** True when sticks/D-pad would command motion (ignores arm state). */
    bool hasMotorInput() const;

    bool getButton(uint16_t button) const { return (m_last_buttons & button) != 0; }
    bool getButtonPressed(uint16_t button) const;
    bool getButtonReleased(uint16_t button) const;

private:
    PSX m_psx;

    bool m_controller_connected;
    uint32_t m_last_controller_read;

    float m_vx, m_vy, m_wz;
    float m_speed_scale;
    bool m_emergency_stop;

    uint16_t m_last_buttons;
    uint16_t m_previous_buttons;
    uint32_t m_estop_clear_start;
    float m_deadband;

    float mapStick(uint8_t rawValue, bool invert = false);
    void processButtons(const PSX::PSXDATA& js);
    void processJoysticks(const PSX::PSXDATA& js);
};
