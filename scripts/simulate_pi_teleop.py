#!/usr/bin/env python3
"""
Simulated Raspberry Pi Teleop Commands
This script sends teleop commands to the ESP32S3 robot via serial
to test the TELEOP_PI mode functionality.
"""

import serial
import time
import json
import argparse
import math
from typing import Dict, Any

class PiTeleopSimulator:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        """Initialize the Pi teleop simulator."""
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
    def connect(self) -> bool:
        """Connect to the robot's serial port."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1.0)
            print(f"✅ Connected to robot at {self.port}")
            time.sleep(2)  # Wait for connection to stabilize
            return True
        except Exception as e:
            print(f"❌ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the robot."""
        if self.serial_conn:
            self.serial_conn.close()
            print("📡 Disconnected from robot")
    
    def send_command(self, vx: float, vy: float, wz: float) -> bool:
        """Send a teleop command to the robot."""
        if not self.serial_conn:
            return False
        
        # Create command JSON similar to what the Pi would send
        command = {
            "type": "cmd_vel",
            "t_ms": int(time.time() * 1000),  # milliseconds
            "vx": round(vx, 3),  # m/s forward/backward
            "vy": round(vy, 3),  # m/s left/right  
            "wz": round(wz, 3)   # rad/s rotation
        }
        
        try:
            # Send as JSON line
            json_str = json.dumps(command) + "\n"
            self.serial_conn.write(json_str.encode('utf-8'))
            print(f"📤 Sent: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
            return True
        except Exception as e:
            print(f"❌ Send error: {e}")
            return False
    
    def send_mode_request(self, mode: str) -> bool:
        """Send a mode change request to the robot."""
        if not self.serial_conn:
            return False
        
        command = {
            "type": "set_mode",
            "mode": mode
        }
        
        try:
            json_str = json.dumps(command) + "\n"
            self.serial_conn.write(json_str.encode('utf-8'))
            print(f"📤 Mode request: {mode}")
            return True
        except Exception as e:
            print(f"❌ Failed to send mode request: {e}")
            return False
    
    def read_responses(self):
        """Read and display responses from the robot."""
        if not self.serial_conn or not self.serial_conn.in_waiting:
            return
        
        try:
            while self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line:
                    try:
                        # Try to parse as JSON
                        data = json.loads(line)
                        if data.get("type") == "status":
                            state = data.get("state", "UNKNOWN")
                            print(f"📊 Robot state: {state}")
                    except json.JSONDecodeError:
                        # Plain text message
                        if "supervisor" in line.lower() or "teleop" in line.lower():
                            print(f"🤖 {line}")
        except Exception as e:
            print(f"❌ Read error: {e}")
    
    def test_sequence_basic(self):
        """Run a basic movement test sequence."""
        print("\n🚀 Starting Basic Test Sequence...")
        
        # Request TELEOP_PI mode
        print("📋 Requesting TELEOP_PI mode...")
        if not self.send_mode_request("TELEOP_PI"):
            print("❌ Failed to request TELEOP_PI mode")
            return False
        
        # Wait a moment for mode change
        time.sleep(0.5)
        self.read_responses()
        
        sequences = [
            # (vx, vy, wz, duration, description)
            (0.0, 0.0, 0.0, 2.0, "Stop/Initialize"),
            (0.3, 0.0, 0.0, 3.0, "Forward"),
            (0.0, 0.0, 0.0, 1.0, "Stop"),
            (-0.3, 0.0, 0.0, 3.0, "Backward"),
            (0.0, 0.0, 0.0, 1.0, "Stop"),
            (0.0, 0.3, 0.0, 3.0, "Strafe Right"),
            (0.0, 0.0, 0.0, 1.0, "Stop"),
            (0.0, -0.3, 0.0, 3.0, "Strafe Left"),
            (0.0, 0.0, 0.0, 1.0, "Stop"),
            (0.0, 0.0, 1.0, 3.0, "Rotate Left"),
            (0.0, 0.0, 0.0, 1.0, "Stop"),
            (0.0, 0.0, -1.0, 3.0, "Rotate Right"),
            (0.0, 0.0, 0.0, 2.0, "Final Stop"),
        ]
        
        for vx, vy, wz, duration, description in sequences:
            print(f"\n🎯 {description}")
            start_time = time.time()
            
            while time.time() - start_time < duration:
                if not self.send_command(vx, vy, wz):
                    print("❌ Failed to send command")
                    return False
                
                self.read_responses()
                time.sleep(0.05)  # 50ms between commands (well under 200ms timeout)
        
        print("\n✅ Basic test sequence completed!")
        return True
    
    def test_sequence_advanced(self):
        """Run an advanced movement test with complex patterns."""
        print("\n🚀 Starting Advanced Test Sequence...")
        
        # Request TELEOP_PI mode
        print("📋 Requesting TELEOP_PI mode...")
        if not self.send_mode_request("TELEOP_PI"):
            print("❌ Failed to request TELEOP_PI mode")
            return False
        
        # Wait a moment for mode change
        time.sleep(0.5)
        self.read_responses()
        
        # Figure-8 pattern
        print("\n🎯 Figure-8 Pattern")
        for t in range(0, 200):  # 20 seconds at 10Hz
            angle = t * 0.1  # time in seconds * rate
            
            # Figure-8 motion
            vx = 0.2 * math.sin(angle)
            vy = 0.2 * math.sin(2 * angle)
            wz = 0.5 * math.cos(angle)
            
            if not self.send_command(vx, vy, wz):
                return False
            
            self.read_responses()
            time.sleep(0.1)
        
        # Stop
        print("\n🎯 Stop")
        for _ in range(20):
            self.send_command(0.0, 0.0, 0.0)
            self.read_responses()
            time.sleep(0.1)
        
        print("\n✅ Advanced test sequence completed!")
        return True
    
    def interactive_mode(self):
        """Interactive mode for manual command input."""
        print("\n🎮 Interactive Mode")
        print("Commands:")
        print("  w/s - forward/backward")
        print("  a/d - strafe left/right") 
        print("  q/e - rotate left/right")
        print("  space - stop")
        print("  x - exit")
        print("\nPress Enter after each command...")
        
        command_map = {
            'w': (0.3, 0.0, 0.0),   # forward
            's': (-0.3, 0.0, 0.0),  # backward
            'a': (0.0, -0.3, 0.0),  # strafe left
            'd': (0.0, 0.3, 0.0),   # strafe right
            'q': (0.0, 0.0, 1.0),   # rotate left
            'e': (0.0, 0.0, -1.0),  # rotate right
            ' ': (0.0, 0.0, 0.0),   # stop
        }
        
        while True:
            try:
                cmd = input("Command: ").strip().lower()
                
                if cmd == 'x':
                    break
                elif cmd in command_map:
                    vx, vy, wz = command_map[cmd]
                    # Send command for 1 second
                    for _ in range(10):
                        self.send_command(vx, vy, wz)
                        self.read_responses()
                        time.sleep(0.1)
                else:
                    print("Invalid command!")
                    
            except KeyboardInterrupt:
                break
        
        # Final stop
        self.send_command(0.0, 0.0, 0.0)
    
    def run(self, mode: str = "basic"):
        """Run the simulator in specified mode."""
        if not self.connect():
            return False
        
        self.running = True
        
        try:
            print(f"\n🤖 Robot Teleop Simulator Started")
            print(f"📡 Port: {self.port}")
            print(f"⚡ Baudrate: {self.baudrate}")
            print(f"🎯 Mode: {mode}")
            
            # Wait a moment for robot to be ready
            time.sleep(1.0)
            
            if mode == "basic":
                success = self.test_sequence_basic()
            elif mode == "advanced":
                success = self.test_sequence_advanced()
            elif mode == "interactive":
                self.interactive_mode()
                success = True
            else:
                print(f"❌ Unknown mode: {mode}")
                success = False
            
            if success:
                print("\n🎉 Simulation completed successfully!")
            
        except KeyboardInterrupt:
            print("\n⏹️  Simulation interrupted by user")
        except Exception as e:
            print(f"\n❌ Simulation error: {e}")
        finally:
            # Send final stop command
            for _ in range(5):
                self.send_command(0.0, 0.0, 0.0)
                time.sleep(0.1)
            
            self.disconnect()
            self.running = False

def main():
    """Main function with command line argument parsing."""
    parser = argparse.ArgumentParser(description="Simulate Pi teleop commands for robot testing")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", 
                      help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baudrate", "-b", type=int, default=115200,
                      help="Baudrate (default: 115200)")
    parser.add_argument("--mode", "-m", choices=["basic", "advanced", "interactive"], 
                      default="basic", help="Test mode (default: basic)")
    
    args = parser.parse_args()
    
    print("🤖 Pi Teleop Simulator")
    print("=" * 50)
    
    simulator = PiTeleopSimulator(args.port, args.baudrate)
    simulator.run(args.mode)

if __name__ == "__main__":
    main()
