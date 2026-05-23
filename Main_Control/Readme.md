# Main_Control — ESP32-S3 Robot Control System

An integrated robot control system for the **16th SJTU Freshman Mechanical Creativity Competition**, built on ESP32-S3 with an Xbox wireless controller. Features mecanum omnidirectional drive, servo arm control, stepper lift, MPU6050 slope parking, and battery protection.

> **Competition result:** Qualified for finals (13th in preliminaries)

## Hardware Overview

| Component | Model / Spec | Role |
|-----------|-------------|------|
| **MCU** | ESP32-S3-DevKitC-1 | Main controller, Bluetooth & I2C |
| **Remote** | Xbox Series X/S (BLE) | Wireless control input |
| **Motor driver** | I2C PWM driver (addr `0x26`) | 4-ch DC motor control |
| **Drive motors** | MG513 geared DC ×4 | Mecanum wheel actuation |
| **Servos** | Standard SG90-style ×4 | Arm / gripper mechanism |
| **Stepper** | 2-phase hybrid ×2 | Lift /升降 mechanism |
| **IMU** | MPU6050 | Tilt detection for auto-parking |
| **Relay** | 5V 1-ch | System power reset |

## Pin Connection Guide

### I2C Bus (Motor Driver & MPU6050)

| ESP32-S3 | Device | Wire |
|----------|--------|------|
| GPIO 8 (SDA) | Motor driver SDA, MPU6050 SDA | Data |
| GPIO 9 (SCL) | Motor driver SCL, MPU6050 SCL | Clock |

The I2C bus runs at 100 kHz. Both the motor driver (addr `0x26`) and MPU6050 share the same bus.

### Servo PWM (LEDC, 50 Hz, 14-bit)

| ESP32-S3 | Servo | Signal range |
|----------|-------|-------------|
| GPIO 4 | Servo 1 | 0–180° (500–2500 µs) |
| GPIO 5 | Servo 2 | 0–180° |
| GPIO 6 | Servo 3 | 0–180° |
| GPIO 7 | Servo 4 | 0–180° |

### Stepper Motors (Shared Step/Dir)

| ESP32-S3 | Stepper 1 | Stepper 2 |
|----------|-----------|-----------|
| GPIO 15 | STEP | — |
| GPIO 16 | DIR | — |
| GPIO 17 | SLEEP | — |
| GPIO 18 | — | STEP |
| GPIO 19 | — | DIR |
| GPIO 20 | — | SLEEP |

Both steppers share the same step pulse and direction signals — they move in sync. SLEEP pins are pulled HIGH on init to enable the drivers.

### Relay

| ESP32-S3 | Relay |
|----------|-------|
| GPIO 12 | Signal (active HIGH, normally ON) |

The relay is ON by default. Pressing the Xbox **Home** button triggers a 5-second OFF cycle to reset the system power.

## Code Architecture

The firmware is structured as modular C++ classes, each responsible for one hardware subsystem:

```
main.cpp
├── MotorDriver      — I2C 4-ch PWM (forward/back + speed)
├── ServoController  — LEDC servo PWM with non-blocking homing
├── MPUManager       — MPU6050 tilt read (single-init guard)
├── StepperDriver    — Shared step/dir with acceleration ramp
├── BatteryMonitor   — Voltage polling (2 s interval)
├── XboxInput        — Joystick → speed curve + mutual exclusion
├── ChassisController— Mecanum kinematics + auto-parking
├── RelayControl     — Power reset (non-blocking)
└── ServoButtonHandler— Button-to-servo mapping (40 ms throttle)
```

### Control Loop

```
loop() every 20 ms
├── xbox.update()           — poll BLE
├── relay.update()          — handle reset timer
├── xbox.process()          — stick → speed curve + filter
├── stepper.update()        — acceleration ramp
├── servoButtons.handle()   — button → servo angle (40 ms)
├── servo.heartbeat()       — re-send angle every 800 ms
├── [20 ms throttle]
│   ├── mpu.updateAndGetTilt()
│   ├── battery.update()
│   ├── chassis.compute()   — mecanum solve
│   └── motor.setPWM()
└── Serial debug (500 ms)
```

## Xbox Controller Mapping

### Chassis (Mecanum Drive)

| Stick | Action |
|-------|--------|
| Left stick ↑↓ | Forward / backward |
| Left stick ←→ | Strafe left / right |
| Right stick ←→ | Rotate in place |

### Lift (Stepper Motor)

| Stick | Action |
|-------|--------|
| Right stick ↑↓ | Lift up / down |

Right stick is **mutually exclusive** — when X displacement dominates, rotation is active; when Y dominates, lift is active.

### Servo Arm

| Button | Servo 1 | Servo 2 | Servo 3 | Servo 4 |
|--------|---------|---------|---------|---------|
| RT | +2° | −2° | — | — |
| RB | −2° | +2° | — | — |
| D-pad ← / → | +2° / −2° | — | — | — |
| D-pad ↑ / ↓ | — | −2° / +2° | — | — |
| X / Y | — | — | +2° / −2° | — |
| A / B | — | — | — | +2° / −2° |

### One-Click Presets

| Button | Servo angles [1, 2, 3, 4] |
|--------|---------------------------|
| Share | 50°, 120°, 50°, 130° |
| Start | 116°, 53°, 50°, 130° |
| Select | 74°, 145°, 48°, 130° |

### Other

| Button | Action |
|--------|--------|
| Home | Trigger relay reset (5 s power cycle) |

## Key Features

### Mecanum Kinematics

Standard mecanum-wheel forward kinematics with automatic normalization:

```
FL =  Y + X + R
FR =  Y − X − R
BL =  Y − X + R
BR =  Y + X − R
```

Values exceeding ±1000 are normalized to prevent saturation.

### Slope Auto-Parking

When all joysticks are centered (deadband < 30) and MPU6050 detects a tilt > 12°, the controller applies a holding torque to prevent rollback on slopes.

### Stepper Acceleration Ramp

Step interval ramps smoothly between target speed and current speed (100 µs steps, 20 ms update rate) to avoid mechanical shock.

### Battery Protection

| Voltage | Behavior |
|---------|----------|
| > 6.0 V | Normal |
| 5.5–6.0 V | Warning (displayed in serial) |
| < 5.5 V | Motor output disabled |

## Build & Flash

This project uses **PlatformIO**.

1. Clone the repo:
   ```bash
   git clone https://github.com/your-org/Main_Control.git
   cd Main_Control
   ```
2. Configure your Xbox MAC address in `src/main.cpp`:
   ```cpp
   #define XBOX_MAC "xx:xx:xx:xx:xx:xx"
   ```
3. Build and upload:
   ```bash
   pio run -t upload
   ```
4. Monitor serial output:
   ```bash
   pio device monitor -b 115200
   ```

## Dependencies (platformio.ini)

- NimBLE-Arduino @ 1.4.2
- Adafruit MPU6050 @ 2.2.9
- Adafruit Unified Sensor @ 1.1.15

## Serial Debug Output

Every 500 ms the system prints one line:

```
Mode:0 Park:0 Tilt:0.0 V:7.4 | Joy:0,0 Rot:0 Lift:0 | Servo:110,45,50,110 | Relay:1
```

Fields: operating mode, parking status, tilt angle (°), battery voltage (V), joystick/rotation/lift values, servo angles, relay state.

## License

MIT — feel free to use and modify for your own competition or robotics project.