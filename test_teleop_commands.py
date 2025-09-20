#!/usr/bin/env python3
"""
Test script for mecanum robot teleop functionality
Sends JSON commands via serial to test Pi → MCU communication
"""

import serial
import json
import time
import sys

def main():
    # Serial port configuration
    port = "/dev/ttyUSB0"  # Linux ESP32 port (adjust if needed)
    baud = 115200
    
    try:
        # Connect to serial port
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud")
        print("Waiting 2 seconds for connection to stabilize...")
        time.sleep(2)
        
        # Test 1: Send a cmd_vel command
        print("\n🧪 Test 1: Sending cmd_vel command (forward + rotation)")
        cmd = {
            "type": "cmd_vel",
            "t_ms": int(time.time() * 1000),
            "vx": 0.3,   # forward 0.3 m/s
            "vy": 0.0,   # no sideways
            "wz": 0.2    # rotate 0.2 rad/s
        }
        json_str = json.dumps(cmd) + "\n"
        print(f"Sending: {json_str.strip()}")
        ser.write(json_str.encode())
        
        # Read responses for 3 seconds
        print("Listening for responses...")
        start_time = time.time()
        while time.time() - start_time < 3:
            if ser.in_waiting:
                line = ser.readline().decode().strip()
                if line:
                    try:
                        data = json.loads(line)
                        if data.get("type") == "status":
                            print(f"📊 Status: {data['state']}, cmd_age: {data['cmd_age_ms']}ms")
                    except json.JSONDecodeError:
                        pass
        
        # Test 2: Stop command (zeros)
        print("\n🧪 Test 2: Sending stop command")
        cmd = {
            "type": "cmd_vel", 
            "t_ms": int(time.time() * 1000),
            "vx": 0.0, "vy": 0.0, "wz": 0.0
        }
        json_str = json.dumps(cmd) + "\n"
        print(f"Sending: {json_str.strip()}")
        ser.write(json_str.encode())
        time.sleep(1)
        
        # Test 3: Wait for timeout (should return to IDLE after 200ms)
        print("\n🧪 Test 3: Testing timeout behavior (waiting 1 second)")
        print("Expecting state to change from TELEOP_PI to IDLE after 200ms timeout")
        start_time = time.time()
        while time.time() - start_time < 1:
            if ser.in_waiting:
                line = ser.readline().decode().strip()
                if line:
                    try:
                        data = json.loads(line)
                        if data.get("type") == "status":
                            print(f"📊 Status: {data['state']}, cmd_age: {data['cmd_age_ms']}ms")
                    except json.JSONDecodeError:
                        pass
        
        # Test 4: Mode request
        print("\n🧪 Test 4: Sending mode request")
        cmd = {"type": "set_mode", "mode": "TELEOP_PI"}
        json_str = json.dumps(cmd) + "\n"
        print(f"Sending: {json_str.strip()}")
        ser.write(json_str.encode())
        time.sleep(0.5)
        
        # Test 5: Strafe command
        print("\n🧪 Test 5: Sending strafe command (sideways movement)")
        cmd = {
            "type": "cmd_vel",
            "t_ms": int(time.time() * 1000),
            "vx": 0.0,   # no forward
            "vy": 0.4,   # sideways 0.4 m/s
            "wz": 0.0    # no rotation
        }
        json_str = json.dumps(cmd) + "\n"
        print(f"Sending: {json_str.strip()}")
        ser.write(json_str.encode())
        
        # Listen for final responses
        start_time = time.time()
        while time.time() - start_time < 2:
            if ser.in_waiting:
                line = ser.readline().decode().strip()
                if line:
                    try:
                        data = json.loads(line)
                        if data.get("type") == "status":
                            print(f"📊 Status: {data['state']}, cmd_age: {data['cmd_age_ms']}ms")
                    except json.JSONDecodeError:
                        pass
        
        print("\n✅ Test complete! Check that:")
        print("  - State changed to TELEOP_PI when commands were sent")
        print("  - State returned to IDLE after 200ms timeout")
        print("  - cmd_age_ms values increased over time")
        
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        print("Make sure the correct port is specified and device is connected")
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    finally:
        if 'ser' in locals():
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()
