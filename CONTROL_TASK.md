Awesome—let’s lock this in as three small, self-contained steps. I’ve written a mini-README for each so you can code-along and land one PR at a time.

---

# Step 1 — Lift `ControlTask` out of `main.cpp`

**Goal:** encapsulate the control loop in a class (no globals), mirroring your `EncoderTask` / `IMUTask` pattern. The robot should behave exactly the same after this step.

&#x20; &#x20;

## What you’ll create

```
include/ControlTask.h
src/ControlTask.cpp
```

## ControlTask.h (interface you implement)

```cpp
#pragma once
#include "MotionController.h"
#include "PS2Controller.h"

class ControlTask {
public:
  ControlTask(MotionController& mc, PS2Controller& ps2);
  bool initialize();   // any LEDs, preflight checks
  bool start();        // xTaskCreatePinnedToCore(...) + trampoline
  void stop();

private:
  static void taskTrampoline(void* arg);
  void taskLoop();     // moved from main.cpp (same timing & behavior)

  MotionController& _mc;
  PS2Controller&    _ps2;
  TaskHandle_t      _task{nullptr};
};
```

## ControlTask.cpp (what to move)

* Copy the body of your current free `ControlTask(void*)` into `taskLoop()` basically verbatim (keep 100 Hz, PS2 update, START toggling closed-loop vs open-loop, R2 debug toggle, `updateSensors()`, `drive(vx,vy,wz)`), just route through `_ps2` and `_mc`.&#x20;
* Use the same core/priority style you already use for `EncoderTask`/`IMUTask` (trampoline → `taskLoop()`), pin to core 1. &#x20;

## main.cpp changes

* Remove the free `ControlTask(void*)` symbol and its globals.
* Construct everything locally and inject:

  ```cpp
  EncoderTask encoder;
  MotionController mc(encoder);        // stays DI-based  :contentReference[oaicite:7]{index=7}
  PS2Controller ps2;
  IMUTask imu(IMU_SDA, IMU_SCL);

  ControlTask ctrl(mc, ps2);
  // init & start in the same places you already do for encoder/imu
  ```
* Keep behavior identical (manual PS2 control, START toggles VELOCITY\_PID vs OPEN\_LOOP).&#x20;

## Acceptance checklist

* Build passes; robot drives the same with PS2.
* START still toggles PID mode; R2 still toggles debug; IMU debug print still works.&#x20;
* No global `motionController`/`ps2Controller`/`ControlTask(void*)` left.

---

# Step 2 — Standardize Pi messaging & move all prints into a `CommsTask`

**Goal:** stop emitting JSON from sensor/control tasks; centralize TX (and later RX) in one publisher task so lines never interleave and the schema is consistent with your ROS2 tutorial.

&#x20;&#x20;

## What you’ll create

```
include/CommsTask.h
src/CommsTask.cpp
```

## CommsTask.h (interface)

```cpp
#pragma once
#include "EncoderTask.h"
#include "IMUTask.h"

struct StatusMsg { uint32_t t_ms; const char* state; uint32_t cmd_age_ms; uint32_t overruns; };

class CommsTask {
public:
  bool initialize(uint32_t baud=115200);
  bool start();  void stop();
  QueueHandle_t encoder_q() const { return _enc_q; }
  QueueHandle_t imu_q() const { return _imu_q; }
  QueueHandle_t status_q() const { return _status_q; } // used in Step 3

private:
  static void taskTrampoline(void*); void taskLoop();
  QueueHandle_t _enc_q{nullptr}, _imu_q{nullptr}, _status_q{nullptr};
  TaskHandle_t _task{nullptr};
};
```

## Wiring changes

1. **Create three queues** in `CommsTask::initialize()` and open Serial there.
2. **EncoderTask**: replace its JSON prints with `xQueueSend(comms.encoder_q(), &EncoderQueueData, 0)` at its current publish cadence; nothing else changes.&#x20;
3. **IMUTask**: same—push `IMUData` to `comms.imu_q()`.&#x20;
4. **CommsTask::taskLoop()**: at 50–100 Hz, non-blocking drain:

   * Encode one JSON per line for each newest item you have buffered:

     * `type:"encoder"` payload (keep fields & names your ROS2 subscriber expects).&#x20;
     * `type:"imu"` payload (accel/gyro/mag/temp from your IMU driver).&#x20;
     * `type:"status"` (added in Step 3).
5. **Control/Encoder/IMU tasks**: stop printing to Serial directly; all printing goes through CommsTask.

## Acceptance checklist

* Serial logs are clean (no interleaving).
* ROS2 subscriber from your tutorial still parses `type:"encoder"` unchanged.&#x20;
* Combined throughput at 115200 works at chosen rates (e.g., encoder 50–100 Hz, IMU 50–100 Hz).

---

# Step 3 — Add the Supervisor (state machine / lifecycle)

**Goal:** introduce a small FSM that decides **who owns the setpoint** (for now: PS2 vs IDLE/ESTOP) and publishes a single “authoritative” body-frame command for `ControlTask` to consume. We’ll extend to Pi later.

&#x20;&#x20;

## What you’ll create

```
include/ISetpointProvider.h
include/Supervisor.h
src/Supervisor.cpp
```

## Interfaces

```cpp
// ISetpointProvider.h
#pragma once
struct BodyCmd { float vx, vy, wz; uint32_t t_ms; };
struct ISetpointProvider {
  virtual BodyCmd latest() const = 0;
  virtual const char* stateName() const = 0;
  virtual ~ISetpointProvider() = default;
};
```

```cpp
// Supervisor.h
#pragma once
#include "ISetpointProvider.h"
#include "PS2Controller.h"

enum class RobotState { IDLE, MANUAL_PS2, ESTOP /* (TELEOP_PI/AUTO_PI later) */ };

class Supervisor : public ISetpointProvider {
public:
  Supervisor(PS2Controller& ps2);
  bool initialize();
  bool start(); void stop();
  BodyCmd latest() const override;          // lock-free snapshot
  const char* stateName() const override;   // for status telemetry
private:
  static void taskTrampoline(void*); void taskLoop(); // 50–100 Hz
  PS2Controller& _ps2;
  volatile RobotState _state{RobotState::IDLE};
  BodyCmd _cmd{}; // double-buffer or atomic struct in your impl
};
```

## Minimal behavior to implement now

* Start in `IDLE`.
* If PS2 is healthy → `MANUAL_PS2`; setpoint = `getVx/Vy/Wz * getSpeedScale()` (same math you do today in the control loop).&#x20;
* If PS2 Select (your emergency) → `ESTOP` (latched until cleared); setpoint zero.&#x20;

## Make `ControlTask` consume the provider

* Change its constructor to also receive `ISetpointProvider&`.
* In `taskLoop()`, replace direct PS2 reads with:

  ```cpp
  BodyCmd sp = provider.latest();
  _mc.updateSensors();                 // as before  :contentReference[oaicite:23]{index=23}
  _mc.drive(sp.vx, sp.vy, sp.wz);     // as before  :contentReference[oaicite:24]{index=24}
  ```
* Keep your START toggle that flips `_mc` between `OPEN_LOOP` and `VELOCITY_PID`—that’s orthogonal to *where* the setpoint comes from.&#x20;

## Status publishing

* Have Supervisor push a tiny `StatusMsg` to `CommsTask::status_q()` at \~10 Hz:

  ```json
  {"type":"status","t_ms":12345,"state":"MANUAL_PS2","cmd_age_ms":0,"overruns":0}
  ```
* This complements your existing encoder/IMU lines and matches the ROS2 bridge style.&#x20;

## Acceptance checklist

* Manual driving works exactly as before.
* `status` shows `IDLE → MANUAL_PS2`, and `ESTOP` latches to zero output.
* Control code no longer touches PS2 directly; it only consumes the provider.

---

## Notes on why this order works

* Step 1 isolates the real-time loop with DI (no globals), anchored on your existing `MotionController`/task patterns. &#x20;
* Step 2 prevents print interleaving and aligns cleanly with your ROS2 encoder subscriber tutorial.&#x20;
* Step 3 adds lifecycle without changing motor math: you just swap the setpoint source (provider) and gain visibility via `status`. Later we can add `/cmd_vel` RX in `CommsTask` and a `TELEOP_PI` state with a freshness timeout.
