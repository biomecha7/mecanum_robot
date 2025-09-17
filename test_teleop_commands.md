# Teleop Testing Guide

## Overview
The MCU side of the teleop feature has been successfully implemented. The robot can now receive JSON commands from a Pi/ROS2 system and respond accordingly.

## What's Been Implemented

### 1. CommsTask RX Functionality
- Added JSON command parsing in `CommsTask::taskLoop()`
- Supports two command types:
  - `cmd_vel`: Velocity commands from Pi
  - `set_mode`: Mode change requests

### 2. Supervisor State Machine
- Added `TELEOP_PI` state
- Implements command freshness tracking (200ms timeout)
- Priority: PS2 > Pi commands > IDLE
- Emergency stop always takes precedence

### 3. ControlTask Provider-Only
- Removed direct PS2 reads for motion commands
- Now consumes authoritative setpoint from Supervisor
- PS2 still used for button handling (START/R2 toggles)

### 4. Enhanced Status Publishing
- Status messages include command age (`cmd_age_ms`)
- State transitions visible in telemetry

## JSON Command Format

### Velocity Command
```json
{"type":"cmd_vel","t_ms":169999999,"vx":0.25,"vy":0.00,"wz":0.10}
```

### Mode Request
```json
{"type":"set_mode","mode":"TELEOP_PI"}
```

## Status Message Format
```json
{"type":"status","t_ms":12345,"state":"TELEOP_PI","cmd_age_ms":17,"overruns":0}
```

## State Machine Behavior

1. **IDLE**: No commands, robot stopped
2. **MANUAL_PS2**: PS2 controller active (highest priority)
3. **TELEOP_PI**: Pi commands active (if fresh < 200ms)
4. **ESTOP**: Emergency stop (PS2 SELECT button)

## Testing Instructions

### Manual Testing via Serial Monitor
1. Flash the firmware to ESP32
2. Open serial monitor at 115200 baud
3. Send JSON commands manually:

```
{"type":"cmd_vel","t_ms":123,"vx":0.1,"vy":0.0,"wz":0.1}
```

### Expected Behavior
- State should change to `TELEOP_PI`
- Robot should move according to command
- Status messages should show `cmd_age_ms` counting up
- After 200ms without commands, state returns to `IDLE`

### PS2 Override
- PS2 controller always takes priority when connected
- Emergency stop (SELECT) always works regardless of mode

## Next Steps for ROS2 Integration

The MCU side is ready for ROS2 integration. The Pi side needs:
1. Serial bridge node to convert `/cmd_vel` to JSON
2. JSON parser for status/encoder/IMU data
3. Foxglove bridge for visualization

See `TELEOP_FEATURE_PLAN.md` for complete Pi-side implementation details.
