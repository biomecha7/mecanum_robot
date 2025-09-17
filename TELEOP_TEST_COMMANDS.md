# Mecanum Robot Teleop Test Commands

## Test the teleop functionality by copying and pasting these JSON commands into the serial monitor:

### Test 1: Forward movement with rotation
```json
{"type":"cmd_vel","t_ms":123456789,"vx":0.3,"vy":0.0,"wz":0.2}
```

### Test 2: Sideways strafe movement  
```json
{"type":"cmd_vel","t_ms":123456790,"vx":0.0,"vy":0.4,"wz":0.0}
```

### Test 3: Diagonal movement
```json
{"type":"cmd_vel","t_ms":123456791,"vx":0.2,"vy":0.2,"wz":0.1}
```

### Test 4: Stop command
```json
{"type":"cmd_vel","t_ms":123456792,"vx":0.0,"vy":0.0,"wz":0.0}
```

### Test 5: Mode request
```json
{"type":"set_mode","mode":"TELEOP_PI"}
```

## Expected Behavior:

1. **When sending cmd_vel**: State should change from IDLE → TELEOP_PI
2. **Movement commands**: Robot should move (wheels spin, encoder counts change)  
3. **After stopping commands**: State should return to IDLE after 200ms timeout
4. **Status messages**: Should show cmd_age_ms increasing until timeout

## How to Test:

1. Open serial monitor: `pio device monitor --baud 115200`
2. Wait for robot to be in IDLE state (no PS2 connected)
3. Copy/paste one command at a time and press Enter
4. Watch the status messages for state changes
5. Observe encoder data for wheel movement

## Success Criteria:

- ✅ Status shows "TELEOP_PI" when commands are active
- ✅ Status returns to "IDLE" after 200ms without commands  
- ✅ Encoder counts change when movement commands are sent
- ✅ cmd_age_ms increases over time, resets with new commands
