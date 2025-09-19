# Hardware Setup Guide

This guide provides detailed instructions for assembling and wiring the mecanum robot hardware.

## 📋 Table of Contents

- [Component List](#component-list)
- [Wiring Diagram](#wiring-diagram)
- [Assembly Instructions](#assembly-instructions)
- [Pin Configuration](#pin-configuration)
- [Power Requirements](#power-requirements)
- [Testing Procedures](#testing-procedures)
- [Troubleshooting](#troubleshooting)

---

## Component List

### Main Controller
- **ESP32 DevKit** (Heltec WiFi LoRa 32 V3 recommended)
- **MicroSD Card** (optional, for logging)

### Motor System
- **4x DC Motors** (12V, 100-200 RPM recommended)
- **4x Mecanum Wheels** (80mm diameter)
- **4x BTS7960 Motor Drivers** (or similar H-bridge drivers)
- **4x Quadrature Encoders** (360 PPR recommended)

### Sensors
- **ICM-20948 IMU** (9-DOF sensor breakout board)
- **PS2 Controller** with USB adapter
- **Emergency Stop Button** (normally closed)

### Power System
- **12V Battery Pack** (LiPo recommended, 2000mAh+)
- **5V/3.3V Voltage Regulator** (LM2596 or similar)
- **Power Distribution Board**
- **Fuses** (5A for motors, 1A for electronics)

### Mechanical
- **Robot Chassis** (custom or commercial)
- **Motor Mounts**
- **Battery Mount**
- **Cable Management**

---

## Wiring Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 Controller                         │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   GPIO 21   │  │   GPIO 20   │  │   GPIO 26   │        │
│  │    ENC1A    │  │    ENC1B    │  │    ENC2A    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   GPIO 48   │  │   GPIO 47   │  │   GPIO 33   │        │
│  │    ENC2B    │  │    ENC3A    │  │    ENC3B    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   GPIO 34   │  │   GPIO 35   │  │   GPIO 38   │        │
│  │    ENC4A    │  │    ENC4B    │  │    IMU_SDA  │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   GPIO 39   │  │   GPIO 46   │  │   GPIO 1    │        │
│  │    IMU_SCL  │  │   M1_RPWM   │  │   M1_LPWM   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   GPIO 2    │  │   GPIO 3    │  │   GPIO 36   │        │
│  │   M2_RPWM   │  │   M2_LPWM   │  │   M3_RPWM   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   GPIO 37   │  │   GPIO 45   │  │   GPIO 19   │        │
│  │   M3_LPWM   │  │   M4_RPWM   │  │   M4_LPWM   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   GPIO 5    │  │   GPIO 7    │  │   GPIO 6    │        │
│  │   PS2_ATT   │  │   PS2_CLK   │  │   PS2_CMD   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   GPIO 4    │  │   GPIO 42   │  │   GPIO 41   │        │
│  │   PS2_DAT   │  │   LED_RED   │  │   LED_GRN   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   GPIO 40   │  │   3.3V      │  │     GND     │        │
│  │   LED_YLW   │  │   Power     │  │   Ground    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    Motor Drivers (BTS7960)                  │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │    M1       │  │    M2       │  │    M3       │        │
│  │  Front Left │  │ Front Right │  │  Rear Left  │        │
│  │             │  │             │  │             │        │
│  │ RPWM←GPIO46 │  │ RPWM←GPIO2  │  │ RPWM←GPIO36 │        │
│  │ LPWM←GPIO1  │  │ LPWM←GPIO3  │  │ LPWM←GPIO37 │        │
│  │    R_IS     │  │    R_IS     │  │    R_IS     │        │
│  │    L_IS     │  │    L_IS     │  │    L_IS     │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
│  ┌─────────────┐                                          │
│  │    M4       │                                          │
│  │ Rear Right  │                                          │
│  │             │                                          │
│  │ RPWM←GPIO45 │                                          │
│  │ LPWM←GPIO19 │                                          │
│  │    R_IS     │                                          │
│  │    L_IS     │                                          │
│  └─────────────┘                                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                       Power System                          │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │  12V LiPo   │  │ LM2596 Reg  │  │   5V Rail   │        │
│  │   Battery   │  │   12V→5V    │  │  (Motors)   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                              │                             │
│                              ▼                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ AMS1117     │  │   3.3V      │  │   GND       │        │
│  │   5V→3.3V   │  │   Rail      │  │   Rail      │        │
│  │  (ESP32)    │  │ (Sensors)   │  │ (Common)    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

---

## Assembly Instructions

### Step 1: Prepare the Chassis
1. **Mount Motors**: Attach 4 DC motors to the chassis corners
2. **Install Wheels**: Attach mecanum wheels to motor shafts
3. **Mount Encoders**: Install encoders on motor shafts
4. **Secure Battery**: Mount battery pack in center of chassis

### Step 2: Install Electronics
1. **ESP32 Mounting**: Secure ESP32 in accessible location
2. **Motor Drivers**: Mount BTS7960 drivers with heat sinks
3. **IMU Placement**: Mount IMU in center, away from motors
4. **Power Distribution**: Install power distribution board

### Step 3: Wiring
1. **Power Connections**:
   - Connect battery positive to power distribution
   - Connect battery negative to common ground
   - Wire voltage regulators for 5V and 3.3V rails

2. **Motor Connections**:
   - Connect motor drivers to 12V rail
   - Wire PWM signals from ESP32 to driver inputs
   - Connect motor outputs to wheels

3. **Sensor Connections**:
   - Wire encoders to ESP32 GPIO pins
   - Connect IMU via I2C (SDA/SCL)
   - Wire PS2 controller to SPI pins

4. **Safety Systems**:
   - Install emergency stop button in series with motor power
   - Connect status LEDs to GPIO pins

### Step 4: Cable Management
1. **Route Cables**: Keep power and signal cables separate
2. **Secure Connections**: Use cable ties and strain reliefs
3. **Label Wires**: Mark connections for easy troubleshooting

---

## Pin Configuration

The pin configuration is defined in `include/RobotPins.h`:

### Motor Driver Pins
```cpp
#define M1_RPWM 46   // Front Left motor - Right PWM
#define M1_LPWM 1    // Front Left motor - Left PWM
#define M2_RPWM 2    // Front Right motor - Right PWM
#define M2_LPWM 3    // Front Right motor - Left PWM
#define M3_RPWM 36   // Rear Left motor - Right PWM
#define M3_LPWM 37   // Rear Left motor - Left PWM
#define M4_RPWM 45   // Rear Right motor - Right PWM
#define M4_LPWM 19   // Rear Right motor - Left PWM
```

### Encoder Pins
```cpp
#define ENC_M1_A  21  // Front Left encoder A
#define ENC_M1_B  20  // Front Left encoder B
#define ENC_M2_A  26  // Front Right encoder A  
#define ENC_M2_B  48  // Front Right encoder B
#define ENC_M3_A  47  // Rear Left encoder A
#define ENC_M3_B  33  // Rear Left encoder B
#define ENC_M4_A  34  // Rear Right encoder A
#define ENC_M4_B  35  // Rear Right encoder B
```

### Sensor Pins
```cpp
#define IMU_SDA  38   // IMU I2C Data
#define IMU_SCL  39   // IMU I2C Clock
```

### PS2 Controller Pins
```cpp
#define PIN_PS2_ATT  5   // PS2 Attention
#define PIN_PS2_CLK  7   // PS2 Clock
#define PIN_PS2_CMD  6   // PS2 Command
#define PIN_PS2_DAT  4   // PS2 Data
```

### Status LEDs
```cpp
#define LED_PIN_RED GPIO_NUM_42  // Error/ESTOP LED
#define LED_PIN_GRN GPIO_NUM_41  // Normal operation LED
#define LED_PIN_YLW GPIO_NUM_40  // Warning LED
```

---

## Power Requirements

### Motor Power
- **Voltage**: 12V DC
- **Current**: 2-4A per motor (8-16A total)
- **Peak Current**: 6A per motor (24A total)
- **Battery Capacity**: 2000mAh minimum (4000mAh recommended)

### Electronics Power
- **ESP32**: 3.3V, 500mA
- **IMU**: 3.3V, 20mA
- **Motor Drivers**: 5V, 100mA each
- **PS2 Controller**: 3.3V, 50mA
- **Total Electronics**: 3.3V, 1A

### Power Distribution
```
12V Battery
├── Direct to Motor Drivers (via fuses)
├── 12V→5V Regulator (LM2596)
│   └── 5V Rail
│       ├── Motor Driver Logic
│       └── 5V→3.3V Regulator (AMS1117)
│           └── 3.3V Rail
│               ├── ESP32
│               ├── IMU
│               └── PS2 Controller
└── Common Ground
```

---

## Testing Procedures

### Pre-Power Testing
1. **Continuity Check**: Verify all connections with multimeter
2. **Resistance Check**: Ensure no short circuits
3. **Voltage Check**: Verify power rails before connecting ESP32

### Initial Power-On
1. **Power Sequence**:
   - Connect battery
   - Check 12V rail voltage
   - Check 5V rail voltage
   - Check 3.3V rail voltage
   - Connect ESP32

2. **Boot Verification**:
   - Monitor serial output
   - Verify all tasks initialize
   - Check sensor readings

### Motor Testing
1. **Individual Motor Test**:
   - Test each motor separately
   - Verify forward/reverse operation
   - Check PWM response

2. **Encoder Testing**:
   - Verify encoder pulse counting
   - Check quadrature signal quality
   - Test velocity calculations

### Sensor Testing
1. **IMU Calibration**:
   - Perform static calibration
   - Verify accelerometer readings
   - Check gyroscope response

2. **PS2 Controller**:
   - Test button responses
   - Verify joystick ranges
   - Check emergency stop function

### System Integration
1. **Control Loop Test**:
   - Verify 100Hz control loop
   - Test open-loop control
   - Test closed-loop control

2. **Communication Test**:
   - Verify serial telemetry
   - Test ROS2 bridge connection
   - Check command reception

---

## Troubleshooting

### Common Issues

#### Motors Not Responding
- **Check**: Power connections to motor drivers
- **Check**: PWM signal connections
- **Check**: Motor driver enable pins
- **Check**: Fuse continuity

#### Encoder Issues
- **Check**: Encoder power supply (5V)
- **Check**: Signal connections (A/B channels)
- **Check**: Pull-up resistors (10kΩ)
- **Check**: Ground connections

#### IMU Communication Errors
- **Check**: I2C connections (SDA/SCL)
- **Check**: Pull-up resistors (4.7kΩ)
- **Check**: Power supply (3.3V)
- **Check**: I2C address configuration

#### PS2 Controller Not Working
- **Check**: SPI connections (ATT/CLK/CMD/DAT)
- **Check**: Power supply (3.3V)
- **Check**: Controller initialization
- **Check**: USB adapter functionality

#### ESP32 Boot Issues
- **Check**: Power supply voltage (3.3V)
- **Check**: Ground connections
- **Check**: Boot mode pins
- **Check**: Serial connection

### Diagnostic Commands

#### Serial Monitor Commands
```
# Check system status
ros2 topic echo /mcu/status

# Monitor encoder data
ros2 topic echo /encoder_data_raw

# Check IMU data
ros2 topic echo /imu

# Test motor response
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

#### Hardware Diagnostics
```bash
# Check power rails
multimeter - voltage check on 12V, 5V, 3.3V rails

# Check signal integrity
oscilloscope - PWM signals, encoder signals, I2C signals

# Check continuity
multimeter - continuity check on all connections
```

### Performance Issues

#### Slow Response
- **Check**: Control loop frequency (should be 100Hz)
- **Check**: Task priorities
- **Check**: CPU usage
- **Check**: Memory usage

#### Inaccurate Movement
- **Check**: Encoder calibration
- **Check**: Wheel diameter settings
- **Check**: Wheelbase measurements
- **Check**: PID tuning

#### Communication Issues
- **Check**: Serial baud rate (115200)
- **Check**: Cable connections
- **Check**: USB port functionality
- **Check**: ROS2 bridge status

---

## Safety Considerations

### Electrical Safety
- **Fuse Protection**: Install appropriate fuses for all power rails
- **Ground Isolation**: Ensure proper grounding
- **Voltage Limits**: Never exceed component voltage ratings
- **Current Limits**: Monitor current consumption

### Mechanical Safety
- **Emergency Stop**: Always accessible and functional
- **Secure Mounting**: Ensure all components are securely mounted
- **Cable Protection**: Protect cables from damage
- **Wheel Guards**: Consider adding wheel guards for safety

### Operational Safety
- **Startup Sequence**: Follow proper startup procedures
- **Testing Area**: Use safe testing environment
- **Supervision**: Never leave robot unattended during testing
- **Documentation**: Keep detailed records of all modifications

---

## Maintenance

### Regular Maintenance
- **Battery Check**: Monitor battery voltage and capacity
- **Connection Check**: Inspect all connections for wear
- **Software Updates**: Keep firmware up to date
- **Calibration**: Periodic sensor recalibration

### Preventive Maintenance
- **Cleaning**: Regular cleaning of mechanical components
- **Lubrication**: Lubricate moving parts as needed
- **Inspection**: Regular inspection of cables and connectors
- **Backup**: Keep backup copies of configuration and code

This hardware setup guide provides comprehensive instructions for building and maintaining the mecanum robot. Follow all safety procedures and test thoroughly before operation.
