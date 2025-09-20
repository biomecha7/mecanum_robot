#pragma once
#include "ISetpointProvider.h"
#include "PS2Controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Supervisor state enumeration
 * 
 * Defines the possible states of the supervisor's state machine
 */
enum class SupervisorState { 
  IDLE,              // Robot is idle, no commands
  PS2_OPEN_LOOP,      // PS2 control with open-loop motor control
  PS2_CLOSED_LOOP,   // PS2 control with PID closed-loop control
  PI_TELEOP,         // Pi/ROS2 teleop control
  ESTOP              // Emergency stop (latched until cleared)
};

/**
 * @brief Supervisor task for robot state management
 * 
 * This task implements a state machine that determines who owns the setpoint
 * and provides a single authoritative body-frame command for ControlTask to consume.
 * 
 * Features:
 * - State machine with IDLE, MANUAL_PS2, ESTOP states
 * - PS2 controller integration
 * - Emergency stop handling
 * - Status publishing for telemetry
 */
class Supervisor : public ISetpointProvider {
public:
  /**
   * @brief Construct a new Supervisor object
   * @param ps2 Reference to PS2Controller
   */
  Supervisor(PS2Controller& ps2);
  
  /**
   * @brief Initialize the supervisor
   * @return true if initialization successful, false otherwise
   */
  bool initialize();
  
  /**
   * @brief Start the supervisor task
   * @return true if task started successfully, false otherwise
   */
  bool start();
  
  /**
   * @brief Stop the supervisor task
   */
  void stop();
  
  /**
   * @brief Get the latest velocity command (ISetpointProvider interface)
   * @return BodyCmd containing the latest setpoint
   */
  BodyCmd latest() const override;
  
  /**
   * @brief Get the current state name for telemetry (ISetpointProvider interface)
   * @return String describing the current state
   */
  const char* stateName() const override;
  
  /**
   * @brief Feed Pi command to supervisor
   * @param vx Forward velocity (m/s)
   * @param vy Sideways velocity (m/s)
   * @param wz Angular velocity (rad/s)
   * @param t_ms Timestamp in milliseconds
   */
  void feedPiCmd(float vx, float vy, float wz, uint32_t t_ms);
  
  /**
   * @brief Request mode change
   * @param mode Mode name ("TELEOP_PI", "MANUAL_PS2", etc.)
   */
  void requestMode(const char* mode);
  
  /**
   * @brief Get command age in milliseconds
   * @return Age of last Pi command in ms
   */
  uint32_t getCmdAgeMs() const;

private:
  static void taskTrampoline(void* arg);
  void taskLoop();
  static const char* stateNameFromEnum(SupervisorState state);
  
  PS2Controller& _ps2;
  TaskHandle_t _task{nullptr};
  
  // State machine state
  volatile SupervisorState _state{SupervisorState::IDLE};
  
  // Command data (atomic access)
  mutable BodyCmd _cmd{}; // Current command
  
  // Pi command data
  struct PiCmd {
    float vx, vy, wz;
    uint32_t t_ms;
    uint32_t last_received_ms;
  } _pi_cmd{};
  
  // Command freshness timeout (ms)
  static const uint32_t PI_CMD_TIMEOUT_MS = 200;
  
  // Mode request tracking
  enum class ModeRequest {
    NONE,
    PI_TELEOP,
    PS2_OPEN_LOOP,
    PS2_CLOSED_LOOP
  };
  volatile ModeRequest _mode_request{ModeRequest::NONE};
  volatile uint32_t _mode_request_time_ms{0};
  
  // Task configuration
  static const int SUPERVISOR_HZ = 50;
};