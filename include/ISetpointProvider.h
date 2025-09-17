#pragma once
#include <stdint.h>

/**
 * @brief Body frame command structure
 * 
 * Represents a velocity command in the robot's body frame
 */
struct BodyCmd { 
  float vx;      // Forward/backward velocity (m/s)
  float vy;      // Left/right velocity (m/s) 
  float wz;      // Angular velocity (rad/s)
  uint32_t t_ms; // Timestamp (ms)
};

/**
 * @brief Interface for setpoint providers
 * 
 * This interface allows different sources to provide velocity commands
 * to the ControlTask. Examples: PS2 controller, Pi commands, autonomous planner
 */
struct ISetpointProvider {
  /**
   * @brief Get the latest velocity command
   * @return BodyCmd containing the latest setpoint
   */
  virtual BodyCmd latest() const = 0;
  
  /**
   * @brief Get the current state name for telemetry
   * @return String describing the current state
   */
  virtual const char* stateName() const = 0;
  
  /**
   * @brief Virtual destructor for proper cleanup
   */
  virtual ~ISetpointProvider() = default;
};