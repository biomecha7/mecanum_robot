#!/usr/bin/env python3
"""
Simple script to monitor PS2 controller status and help debug ESTOP issues.
"""

import serial
import json
import time
import sys

def monitor_ps2_status(port="/dev/cu.usbserial-0001", baud=115200):
    """Monitor the robot's status messages to check PS2 controller state."""
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"📱 Connected to {port} at {baud} baud")
        print("🔍 Monitoring PS2 controller status...")
        print("💡 Look for 'Controller disconnected!' or ESTOP messages")
        print("⏹️  Press Ctrl+C to stop\n")
        
        line_count = 0
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        line_count += 1
                        
                        # Show line numbers to see if output is too verbose
                        if line_count <= 50:
                            
                            # Filter for important messages
                            if any(keyword in line.lower() for keyword in 
                                  ['controller', 'estop', 'emergency', 'ps2', 'status', 'state']):
                                print(f"🔍 {line}")
                            
                            # Try to parse JSON status messages
                            elif line.startswith('{"type":"status"'):
                                try:
                                    data = json.loads(line)
                                    state = data.get("state", "UNKNOWN")
                                    cmd_age = data.get("cmd_age_ms", 0)
                                    print(f"📊 State: {state}, Cmd Age: {cmd_age}ms")
                                except json.JSONDecodeError:
                                    pass
                        
                        elif line_count == 51:
                            print("📢 Too much output - filtering to important messages only...")
                            
                except UnicodeDecodeError:
                    pass
            
            time.sleep(0.01)  # Small delay to prevent CPU spinning
            
    except KeyboardInterrupt:
        print("\n✅ Monitoring stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
        monitor_ps2_status(port)
    else:
        monitor_ps2_status()
