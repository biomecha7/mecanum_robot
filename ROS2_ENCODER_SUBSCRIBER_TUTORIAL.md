# ROS2 Encoder Data Subscriber Package Tutorial

This tutorial shows how to create a ROS2 package to subscribe to encoder data from the ESP32 mecanum robot via USB serial JSON bridge.

## Overview

The ESP32 publishes encoder data as JSON messages over USB serial at ~10Hz with the following format:

```json
{
  "type": "encoder",
  "t_ms": 12345,
  "counts": [1234, 5678, 9012, 3456],
  "deltas": [5, -3, 2, 4],
  "pos_rad": [0.123, 0.456, 0.789, 0.012],
  "vel_ms": [0.05, -0.02, 0.03, 0.01],
  "avel_rads": [1.25, -0.5, 0.75, 0.25],
  "vel_filt_ms": [0.048, -0.019, 0.029, 0.011],
  "dt_ms": 10.5,
  "freq_hz": 95.2,
  "update_count": 12345,
  "valid": true,
  "wheel_names": ["front_left", "front_right", "rear_left", "rear_right"],
  "errors": [0, 0, 0, 0]
}
```

## Prerequisites

- ROS2 (tested with Humble/Iron)
- Python 3.8+
- pyserial package
- ESP32 connected via USB

## Step 1: Create ROS2 Workspace and Package

```bash
# Create workspace
mkdir -p ~/mecanum_ws/src
cd ~/mecanum_ws/src

# Create the package
ros2 pkg create --build-type ament_python mecanum_encoder_subscriber \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs

cd mecanum_encoder_subscriber
```

## Step 2: Define Custom Message Types

Create custom message definitions for the encoder data:

```bash
mkdir msg
```

Create `msg/EncoderData.msg`:
```
# Encoder data from ESP32 mecanum robot
Header header
string type                    # "encoder"
uint64 timestamp_ms           # ESP32 timestamp in milliseconds
int32[] counts                # Raw encoder counts [FL, FR, RL, RR]
int32[] deltas                # Count changes since last reading
float64[] positions_rad       # Wheel positions in radians
float64[] velocities_ms       # Linear wheel velocities in m/s
float64[] angular_velocities_rads  # Angular velocities in rad/s
float64[] filtered_velocities_ms   # Filtered velocities
float64 dt_ms                 # Time delta in milliseconds
float64 frequency_hz          # Update frequency
uint32 update_count           # Update counter
bool data_valid               # Data validity flag
string[] wheel_names          # Wheel identifiers
uint8[] errors                # Error flags per wheel
```

Create `msg/WheelOdometry.msg`:
```
# Computed wheel odometry
Header header
geometry_msgs/Twist robot_velocity     # Robot velocity (linear + angular)
geometry_msgs/Pose2D robot_pose        # Robot pose (x, y, theta)
float64[] wheel_velocities             # Individual wheel velocities
float64[] wheel_positions              # Individual wheel positions
float64 left_velocity                  # Left side average velocity
float64 right_velocity                 # Right side average velocity
bool data_valid                        # Validity flag
```

## Step 3: Update package.xml

Edit `package.xml` to include message dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mecanum_encoder_subscriber</name>
  <version>0.0.0</version>
  <description>ROS2 package for subscribing to ESP32 mecanum robot encoder data</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>rosidl_default_generators</depend>
  
  <exec_depend>rosidl_default_runtime</exec_depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Step 4: Update CMakeLists.txt

Edit `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(mecanum_encoder_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderData.msg"
  "msg/WheelOdometry.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Step 5: Create Serial Reader Node

Create `mecanum_encoder_subscriber/serial_encoder_reader.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import json
import threading
from std_msgs.msg import Header
from mecanum_encoder_subscriber.msg import EncoderData


class SerialEncoderReader(Node):
    def __init__(self):
        super().__init__('serial_encoder_reader')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # Publisher
        self.encoder_pub = self.create_publisher(
            EncoderData, 
            'encoder_data_raw', 
            10
        )
        
        # Serial connection
        self.serial_conn = None
        self.running = False
        
        # Statistics
        self.message_count = 0
        self.error_count = 0
        
        # Start serial reading
        self.connect_serial()
        
        # Status timer
        self.create_timer(5.0, self.print_status)
        
        self.get_logger().info(f"Serial encoder reader started on {self.serial_port}")

    def connect_serial(self):
        """Connect to serial port and start reading thread"""
        try:
            self.serial_conn = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=1.0
            )
            self.running = True
            
            # Start reading thread
            self.read_thread = threading.Thread(target=self.serial_read_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            self.get_logger().info("Serial connection established")
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.running = False

    def serial_read_loop(self):
        """Main serial reading loop"""
        buffer = ""
        
        while self.running and rclpy.ok():
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    # Read available data
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line and line.startswith('{') and line.endswith('}'):
                            self.process_json_line(line)
                            
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                self.error_count += 1

    def process_json_line(self, line):
        """Process a JSON line from serial"""
        try:
            data = json.loads(line)
            
            # Check if this is encoder data
            if data.get('type') == 'encoder':
                self.publish_encoder_data(data)
                self.message_count += 1
                
        except json.JSONDecodeError as e:
            self.get_logger().debug(f"JSON decode error: {e}")
            self.error_count += 1
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")
            self.error_count += 1

    def publish_encoder_data(self, data):
        """Convert JSON data to ROS message and publish"""
        try:
            msg = EncoderData()
            
            # Header
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            
            # Basic info
            msg.type = data.get('type', 'encoder')
            msg.timestamp_ms = int(data.get('t_ms', 0))
            
            # Arrays
            msg.counts = data.get('counts', [0, 0, 0, 0])
            msg.deltas = data.get('deltas', [0, 0, 0, 0])
            msg.positions_rad = data.get('pos_rad', [0.0, 0.0, 0.0, 0.0])
            msg.velocities_ms = data.get('vel_ms', [0.0, 0.0, 0.0, 0.0])
            msg.angular_velocities_rads = data.get('avel_rads', [0.0, 0.0, 0.0, 0.0])
            msg.filtered_velocities_ms = data.get('vel_filt_ms', [0.0, 0.0, 0.0, 0.0])
            
            # Timing
            msg.dt_ms = float(data.get('dt_ms', 0.0))
            msg.frequency_hz = float(data.get('freq_hz', 0.0))
            msg.update_count = int(data.get('update_count', 0))
            
            # Status
            msg.data_valid = bool(data.get('valid', False))
            msg.wheel_names = data.get('wheel_names', ['fl', 'fr', 'rl', 'rr'])
            msg.errors = data.get('errors', [0, 0, 0, 0])
            
            # Publish
            self.encoder_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Publishing error: {e}")

    def print_status(self):
        """Print periodic status"""
        self.get_logger().info(
            f"Messages: {self.message_count}, Errors: {self.error_count}, "
            f"Connected: {self.running and self.serial_conn is not None}"
        )

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = SerialEncoderReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 6: Create Odometry Calculator Node

Create `mecanum_encoder_subscriber/mecanum_odometry.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from mecanum_encoder_subscriber.msg import EncoderData, WheelOdometry


class MecanumOdometry(Node):
    def __init__(self):
        super().__init__('mecanum_odometry')
        
        # Robot parameters (adjust to match your robot)
        self.declare_parameter('wheel_radius', 0.04)  # 40mm radius
        self.declare_parameter('wheelbase_length', 0.2)  # 200mm between front/rear
        self.declare_parameter('wheelbase_width', 0.2)   # 200mm between left/right
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase_length = self.get_parameter('wheelbase_length').value
        self.wheelbase_width = self.get_parameter('wheelbase_width').value
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.last_time = None
        
        # Subscribers
        self.encoder_sub = self.create_subscription(
            EncoderData,
            'encoder_data_raw',
            self.encoder_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(WheelOdometry, 'wheel_odometry', 10)
        self.nav_odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        self.get_logger().info("Mecanum odometry calculator started")

    def encoder_callback(self, msg):
        """Process encoder data and compute odometry"""
        try:
            if not msg.data_valid or len(msg.velocities_ms) < 4:
                return
                
            # Extract wheel velocities (m/s)
            vfl = msg.velocities_ms[0]  # Front Left
            vfr = msg.velocities_ms[1]  # Front Right
            vrl = msg.velocities_ms[2]  # Rear Left
            vrr = msg.velocities_ms[3]  # Rear Right
            
            # Mecanum wheel kinematics
            # Robot velocity in robot frame
            vx = (vfl + vfr + vrl + vrr) / 4.0
            vy = (-vfl + vfr + vrl - vrr) / 4.0
            wz = (-vfl + vfr - vrl + vrr) / (4.0 * (self.wheelbase_length + self.wheelbase_width) / 2.0)
            
            # Time handling
            current_time = self.get_clock().now()
            if self.last_time is not None:
                dt = (current_time - self.last_time).nanoseconds / 1e9
                
                # Update robot pose (simple integration)
                cos_theta = math.cos(self.robot_theta)
                sin_theta = math.sin(self.robot_theta)
                
                # Transform to global frame
                delta_x = (vx * cos_theta - vy * sin_theta) * dt
                delta_y = (vx * sin_theta + vy * cos_theta) * dt
                delta_theta = wz * dt
                
                self.robot_x += delta_x
                self.robot_y += delta_y
                self.robot_theta += delta_theta
                
                # Normalize angle
                while self.robot_theta > math.pi:
                    self.robot_theta -= 2.0 * math.pi
                while self.robot_theta < -math.pi:
                    self.robot_theta += 2.0 * math.pi
            
            self.last_time = current_time
            
            # Publish custom odometry message
            self.publish_wheel_odometry(msg, vx, vy, wz)
            
            # Publish nav_msgs/Odometry
            self.publish_nav_odometry(vx, vy, wz, current_time)
            
        except Exception as e:
            self.get_logger().error(f"Odometry calculation error: {e}")

    def publish_wheel_odometry(self, encoder_msg, vx, vy, wz):
        """Publish custom wheel odometry message"""
        msg = WheelOdometry()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        
        # Robot velocity
        msg.robot_velocity = Twist()
        msg.robot_velocity.linear.x = vx
        msg.robot_velocity.linear.y = vy
        msg.robot_velocity.angular.z = wz
        
        # Robot pose
        msg.robot_pose = Pose2D()
        msg.robot_pose.x = self.robot_x
        msg.robot_pose.y = self.robot_y
        msg.robot_pose.theta = self.robot_theta
        
        # Wheel data
        msg.wheel_velocities = encoder_msg.velocities_ms
        msg.wheel_positions = encoder_msg.positions_rad
        
        # Side velocities
        msg.left_velocity = (encoder_msg.velocities_ms[0] + encoder_msg.velocities_ms[2]) / 2.0  # FL + RL
        msg.right_velocity = (encoder_msg.velocities_ms[1] + encoder_msg.velocities_ms[3]) / 2.0  # FR + RR
        
        msg.data_valid = encoder_msg.data_valid
        
        self.odom_pub.publish(msg)

    def publish_nav_odometry(self, vx, vy, wz, current_time):
        """Publish standard nav_msgs/Odometry"""
        odom = Odometry()
        
        # Header
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        cy = math.cos(self.robot_theta * 0.5)
        sy = math.sin(self.robot_theta * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        
        # Covariances (simple diagonal)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y  
        odom.pose.covariance[35] = 0.01  # yaw
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[7] = 0.01  # vy
        odom.twist.covariance[35] = 0.01 # vyaw
        
        self.nav_odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    
    node = MecanumOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 7: Update setup.py

Edit `setup.py`:

```python
from setuptools import setup

package_name = 'mecanum_encoder_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/encoder_subscriber.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for ESP32 mecanum robot encoder data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_encoder_reader = mecanum_encoder_subscriber.serial_encoder_reader:main',
            'mecanum_odometry = mecanum_encoder_subscriber.mecanum_odometry:main',
        ],
    },
)
```

## Step 8: Create Launch File

Create `launch/encoder_subscriber.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for ESP32'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate'
        ),
        
        # Serial encoder reader node
        Node(
            package='mecanum_encoder_subscriber',
            executable='serial_encoder_reader',
            name='serial_encoder_reader',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }],
            output='screen'
        ),
        
        # Mecanum odometry calculator
        Node(
            package='mecanum_encoder_subscriber',
            executable='mecanum_odometry',
            name='mecanum_odometry',
            parameters=[{
                'wheel_radius': 0.04,      # 40mm
                'wheelbase_length': 0.2,   # 200mm
                'wheelbase_width': 0.2,    # 200mm
            }],
            output='screen'
        ),
    ])
```

## Step 9: Install Python Dependencies

```bash
# Install pyserial
pip3 install pyserial

# Or in your ROS2 environment
sudo apt install python3-serial
```

## Step 10: Build and Run

```bash
# Build the workspace
cd ~/mecanum_ws
colcon build --packages-select mecanum_encoder_subscriber

# Source the workspace
source install/setup.bash

# Run the launch file
ros2 launch mecanum_encoder_subscriber encoder_subscriber.launch.py serial_port:=/dev/ttyUSB0

# Or run nodes individually
ros2 run mecanum_encoder_subscriber serial_encoder_reader --ros-args -p serial_port:=/dev/ttyUSB0
ros2 run mecanum_encoder_subscriber mecanum_odometry
```

## Step 11: Verify Data

```bash
# Check topics
ros2 topic list

# Monitor raw encoder data
ros2 topic echo /encoder_data_raw

# Monitor computed odometry
ros2 topic echo /wheel_odometry
ros2 topic echo /odom

# Check data rate
ros2 topic hz /encoder_data_raw
```

## Expected Topics

- `/encoder_data_raw` - Raw encoder data from ESP32
- `/wheel_odometry` - Custom wheel odometry message
- `/odom` - Standard nav_msgs/Odometry for navigation stack

## Troubleshooting

1. **Serial permission**: `sudo usermod -a -G dialout $USER` (logout/login)
2. **Wrong port**: Check `dmesg | grep tty` after plugging in ESP32
3. **No data**: Verify ESP32 is publishing JSON, check baud rate
4. **Build errors**: Ensure all dependencies are installed

## Next Steps

- Integrate with IMU data for sensor fusion
- Add TF transforms for robot frames
- Connect to navigation stack
- Add data recording/playback
- Implement Kalman filtering

This package provides a complete foundation for consuming encoder data from your ESP32 mecanum robot in ROS2!
