# Mecanum Robot (Heltec WiFi LoRa 32 V3.1)

Open-loop mecanum drive from a PS2 controller on a Heltec WiFi LoRa 32 V3 / V3.1.

## Features

- Open-loop mecanum mixing (no encoders / IMU / PID)
- Boots **disarmed** — press **START** to arm / disarm
- Armed LED on GPIO 41
- Controller rumble if you try to drive while disarmed
- Onboard OLED shows `Hello Adei` once at boot (GPIO 36 is Vext — leave it free)
- ESTOP: **START + TRIANGLE**; clear with **SELECT + START** (hold 1 s)

## Controls

| Input | Action |
|-------|--------|
| Left stick Y / D-pad up-down | Forward / back |
| Left stick X / D-pad left-right | Rotate |
| Right stick X | Strafe |
| L1 / L2 / R1 | Slow (35%) / medium (50%) / fast (100%) |
| START | Arm / disarm |
| START + TRIANGLE | Emergency stop |
| SELECT + START (1 s) | Clear ESTOP |
| SQUARE | Toggle left eye LED (GPIO 47) |
| CIRCLE | Toggle right eye LED (GPIO 48) |
| X (Cross) | Single wink |
| TRIANGLE | ~45s wink / blink eye show |
| R2 | Fun buzzer sound (GPIO 38/40) |

## Hardware pins

See [`include/RobotPins.h`](include/RobotPins.h).

| Function | GPIOs |
|----------|-------|
| PS2 DAT/CMD/ATT/CLK | 4 / 6 / 5 / 7 |
| M1–M4 RPWM/LPWM | 46/1, 2/3, **0**/37, 45/19 |
| Armed LED | 41 |
| Left / right eye LEDs | 47 / 48 |
| Buzzer (RedBot) | 38 / 40 |
| OLED Vext / RST / SDA / SCL | **36** / 21 / 17 / 18 |

**Note:** GPIO 36 is Heltec OLED power control. M3 forward is on GPIO 0.

## Build & flash

Requires [PlatformIO](https://platformio.org/).

```bash
cd mecanum_robot
pio run -t upload -t monitor
```

Port is auto-detected. In Cursor/VS Code, **Ctrl+Shift+R** runs Build + Flash + Serial (see `.vscode/tasks.json`).

## Layout

```
src/main.cpp          # PS2, arming, mecanum mix, OLED
src/MotorDriver.cpp   # BTS7960 PWM
include/RobotPins.h   # Pin map
include/MotorDriver.h
lib/ArduinoPSX/       # PS2 library (with rumble)
platformio.ini
```

## License

See [LICENSE](LICENSE).
