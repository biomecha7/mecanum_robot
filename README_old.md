Awesome—now that the plumbing works, here’s a clean, scalable architecture you can build in stages without painting yourself into a corner.

# 0) High-level split (who does what)

* **Raspberry Pi 4 (ROS 2 Humble, “brain”)**

  * Runs all ROS graph, state estimation, planning, and gateways.
  * micro-ROS **Agent** (one process) to talk to multiple ESP32 “peripherals”.
  * Nav2 (later), `robot_localization`, TF, logging/rosbag, diagnostics.
* **ESP32s (real-time “limbs”)**

  * Do time-critical I/O: motor PWM + per-wheel PID, encoder counting, IMU sampling, PS2 reading.
  * Publish sensor data and accept velocity/actuator setpoints via **micro-ROS** (serial or Wi-Fi).
* **Android app (operator/UI, later autonomy UI)**

  * Sends teleop or **GPS goals**.
  * Talks only to a **Gateway** on the Pi (HTTP/WebSocket/MQTT), not directly to DDS.

---

# 1) Transport choices (pick per board)

**Preferred:** ESP32 <—(UART/USB or Wi-Fi)—> **micro-ROS Agent** on the Pi

* Pros: native ROS topics/services, time sync, QoS, less glue code.
* For UART: `/dev/ttyUSB*` @ 921600 baud works well.
* For Wi-Fi: give each ESP32 a static IP or mDNS name.

**Fallback:** custom binary over UART + a **bridge node** on Pi

* Useful if you already have parsing code; just translate to ROS messages in C++.

You can mix: motor/encoders over UART for reliability, PS2/IMU over Wi-Fi.

---

# 2) Nodes, topics, and message types

## ESP32: Motor/Encoder board (“mcu\_drive”)

* **Subscribes**

  * `/cmd_vel` (geometry\_msgs/Twist) **or** `/wheel_speeds_cmd` (std\_msgs/Float64MultiArray \[FL,FR,RL,RR])
* **Publishes** (100–200 Hz)

  * `/wheel_ticks` (custom msg or std\_msgs/Int32MultiArray)
  * `/wheel_rads` (Float64MultiArray) – wheel rates (rad/s)
  * `/battery` (sensor\_msgs/BatteryState) – optional
  * `/driver_status` (diagnostic\_msgs/DiagnosticArray) – faults, temps
* **Local control**

  * Per-wheel PID on **velocity** (target from Pi; feedback from encoders)
  * **Watchdog:** if no cmd within 200 ms, ramp to zero

## ESP32: IMU board (“mcu\_imu”)

* **Publishes** (200 Hz)

  * `/imu/data_raw` (sensor\_msgs/Imu)
  * `/mag` (sensor\_msgs/MagneticField) if available
  * `/imu/temp` (std\_msgs/Float32) optional

## ESP32: PS2 board (“mcu\_joy”)

* **Publishes** (50 Hz)

  * `/joy` (sensor\_msgs/Joy) – buttons map includes **E-Stop** and “enable”
* **Optional**: subscribe `/cmd_enable` (std\_msgs/Bool) to gate outputs

## Pi core nodes

* `mecanum_kinematics_node`

  * **Sub**: `/wheel_rads`
  * **Pub**: `/wheel_odom` (nav\_msgs/Odometry), `tf` (`odom→base_link`)
  * Provides IK for `/cmd_vel`→`/wheel_speeds_cmd` (if not using ros2\_control)
* `robot_state_publisher` (+ URDF)
* `robot_localization` (EKF/UKF)

  * **Inputs now**: `/wheel_odom`, `/imu/data`
  * **Later**: GPS via `navsat_transform_node`
  * **Outputs**: `/odometry/filtered` + TF (`map→odom` or `odom→base_link`)
* **Gateway** (see §5)
* **micro-ROS Agent**
* **Diagnostics + logging**

  * `ros2 run rqt_robot_monitor` (when you want a GUI) or Foxglove

---

# 3) Frames & TF (keep it simple)

```
map (global)  ──>  odom (drift-minimized)  ──>  base_link  ──>  base_imu, base_gps, wheel_* …
```

* Early phase (no GPS): publish `odom→base_link` from wheel odom or EKF.
* With GPS: use `robot_localization` `ekf_localization_node` + `navsat_transform_node` to generate `map→odom`.

  * GPS + IMU + wheel odom → **EKF** for `base_link`.
  * `navsat_transform_node` converts lat/lon to your **map** frame.

---

# 4) QoS and rates (pragmatic defaults)

* High-rate sensors (IMU, encoders): **Best Effort**, `depth=5`
* Commands (`/cmd_vel`, wheel setpoints): **Reliable**, `depth=1`
* Odometry/TF: Reliable, `10–50 Hz`
* PS2 `/joy`: Reliable, `20–50 Hz`
* Keep micro-ROS publishes roughly: IMU 200 Hz, encoders 100–200 Hz, joy 50 Hz.

---

# 5) Android <—> Pi “Gateway” (don’t expose DDS)

Pick one; all live **on the Pi** and translate to ROS:

### Option A: **REST** (FastAPI/Flask) – simplest with Kotlin (Ktor/OkHttp)

* `POST /goal/gps` → `{ "lat": ..., "lon": ..., "yaw_deg": ... }`

  * Gateway converts to UTM / `geometry_msgs/PoseStamped` in `map`, publishes `/goal_pose`.
* `POST /teleop` → `{ "vx": 0.2, "vy": 0.0, "wz": 0.0 }` (for short bursts)
* `POST /estop` → `{ "on": true }` → toggles `/cmd_enable` false, opens relay.

### Option B: **WebSocket** (rosbridge or custom)

* Use `rosbridge_server` and a lightweight Kotlin WS client to publish/subscribe topics.
* Faster iteration, but you’ll manage schemas client-side.

### Option C: **MQTT**

* Run Mosquitto on Pi; Android publishes (e.g., `robot/cmd/goal_gps`).
* Bridge node translates to ROS. Great if you later add cloud.

**Security**: whichever you pick, add a shared secret/token + optional TLS if off-LAN.

---

# 6) Safety & failsafes (do this early)

* **Hardware E-Stop**: latching relay that cuts motor driver power; wired to a big red mushroom or PS2 button combo.
* **Software E-Stop**: `/cmd_enable` (std\_msgs/Bool). The drive MCU will **ignore commands** unless enabled.
* **Command watchdog**: zero outputs if stale `/cmd_vel`.
* **Overcurrent/temp**: MCU publishes faults on `/driver_status`; Pi gateway refuses motion if faulted.

---

# 7) Nav stack path (outdoor GPS → autonomy)

**Stage 1: Teleop**

* `joy` → `/cmd_vel` → IK → wheel PIDs
* Odometry visible in Foxglove

**Stage 2: State Estimation**

* `robot_localization` EKF fusing `/wheel_odom` + `/imu/data`
* Publish `/odometry/filtered` + TF

**Stage 3: Add GPS**

* Add GPS (NMEA or u-blox); publish `/fix` (sensor\_msgs/NavSatFix) + `/imu`.
* `navsat_transform_node` + EKF → get **`map→odom`**.
* Verify pose in Foxglove (map frame).

**Stage 4: Navigate to GPS goals**

* Use **Nav2** with a “global planner” compatible with sparse maps or just **Regulated Pure Pursuit** controller.
* Gateway converts Android GPS → `PoseStamped` in `map`.
* Use `nav2_bt_navigator` to accept `/goal_pose`.

**Stage 5: Perception & obstacles (later)**

* Add 2D LiDAR or depth camera → Nav2 local/global costmaps.
* Tune controller limits for your mecanum (max vx, vy, wz; acceleration; footprint).

---

# 8) ros2\_control (optional but nice)

You can keep your current IK + PIDs on MCU, or move to `ros2_control`:

* Write a `SystemInterface` exposing 4 wheel **velocity** joints.
* Use a **mecanum\_drive\_controller** (community) or keep your proven kinematics node.
* Benefit: parameters & controllers standardized; easier to swap hardware later.

---

# 9) Package layout (Pi repo)

```
mecanum_bot/
  firmware/
    esp32_drive/            # PWM, encoders, PID, micro-ROS (node: mcu_drive)
    esp32_imu/              # IMU publisher (node: mcu_imu)
    esp32_ps2/              # PS2 -> Joy (node: mcu_joy)
  ros2_ws/src/
    mecanum_description/    # URDF + meshes, frames, wheel radii, Lx/Ly
    mecanum_bringup/        # launch: core + foxglove_bridge + ekf + gateway
    mecanum_kin/            # IK/FK + odom publisher (if not ros2_control)
    gateway/                # REST/WebSocket/MQTT <-> ROS translator
    mecanum_msgs/           # (optional) custom message definitions
```

**Bringup launch** (what it starts):

* micro-ROS agent
* `robot_state_publisher` (URDF)
* kinematics node
* EKF + (later) navsat\_transform
* foxglove\_bridge (for your Mac)
* gateway

---

# 10) Starter EKF params (drop-in)

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /wheel_odom
    odom0_config: [true, true, false,  false, false, true,  false, false, false,  false, false, false,  false, false, false]
    odom0_differential: false

    imu0: /imu/data
    imu0_config:  [false, false, false,  false, false, true,  true,  true,  false, false, false, false, false, false, false]
    imu0_remove_gravitational_acceleration: true
```

When GPS arrives, add `navsat_transform_node` and feed its output as `odomN` or switch to a second EKF for `map` fusion.

---

## What I’d build first (1–2 days)

1. **Drive MCU** over UART micro-ROS: subscribe `/wheel_speeds_cmd`, publish `/wheel_rads`, watchdog.
2. **Pi** kinematics + odom + EKF; teleop via PS2 → `/cmd_vel`.
3. **Gateway** with two endpoints:

   * `POST /estop` → toggles `/cmd_enable`
   * `POST /teleop` → publishes `/cmd_vel` (for now)
4. **Foxglove** running through your SSH tunnel to validate TF/odom.

From there, adding GPS and swapping the Android client from teleop to “send goals” becomes straightforward.