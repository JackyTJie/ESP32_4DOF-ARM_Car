// Copyright 2026 Blitz Team
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "XboxSeriesXControllerESP32_asukiaaa.hpp"

// ============================================================================
// Pin & hardware configuration
// ============================================================================
#define XBOX_MAC "2d:89:6d:41:41:6a"
#define I2C_SDA 8
#define I2C_SCL 9
#define MOTOR_ADDR 0x26

// Servo PWM pins
#define SERVO1_PIN 4
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 7

// Relay pin
#define RELAY_PIN 12

// Stepper motor pins
const int STEP2_PIN = 15;
const int DIR2_PIN = 16;
const int SLEEP2_PIN = 17;
const int STEP3_PIN = 18;
const int DIR3_PIN = 19;
const int SLEEP3_PIN = 20;

// ============================================================================
// Motor driver — I2C-controlled 4-channel PWM
// ============================================================================
class MotorDriver {
public:
    void begin(TwoWire &wire = Wire, uint8_t addr = MOTOR_ADDR) {
        _wire = &wire;
        _addr = addr;
    }

    void setPWM(const int16_t (&pwm)[4]) {
        _wire->beginTransmission(_addr);
        _wire->write(0x07); // REG_PWM
        for (int i = 0; i < 4; i++) {
            int16_t val = constrain(pwm[i], -3600, 3600);
            _wire->write(val >> 8);
            _wire->write(val & 0xFF);
        }
        _wire->endTransmission();
    }

    void stop() {
        int16_t zero[4] = {0};
        setPWM(zero);
    }

    float readVoltage() {
        _wire->beginTransmission(_addr);
        _wire->write(0x08); // REG_VOLT
        _wire->endTransmission(false);
        if (_wire->requestFrom(_addr, (uint8_t)2) == 2) {
            uint16_t raw = (_wire->read() << 8) | _wire->read();
            return raw / 10.0f;
        }
        return -1.0f; // read failed
    }

private:
    TwoWire *_wire;
    uint8_t _addr;
};

// ============================================================================
// Servo controller — non-blocking PWM via LEDC
// ============================================================================
class ServoController {
public:
    void begin() {
        const uint8_t pins[4] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN};
        for (int i = 0; i < 4 ; i++) {
            ledcSetup(i, 50, 14);          // 50Hz, 14-bit
            ledcAttachPin(pins[i], i);
        }
        // Send initial angles
        uint8_t defaults[4] = {110, 45, 50, 110};
        homeAsync(defaults);
        // Wait for homing to finish (one-time init)
        uint32_t start = millis();
        while (!_homingComplete && millis() - start < 2000) {
            update(millis());
        }
    }

    void setAngle(uint8_t index, uint8_t angle) {
        if (index >= 4) return;
        angle = constrain(angle, 0, 180);
        uint32_t duty = map(angle, 0, 180, _dutyMin, _dutyMax);
        ledcWrite(index, duty);
    }

    // Re-send angles periodically to maintain position
    void heartbeat(uint32_t nowMs) {
        if (!_homingComplete) return;
        if (nowMs - _lastHeartbeat > 800) {
            _lastHeartbeat = nowMs;
            for (int i = 0; i < 4; i++) setAngle(i, _angles[i]);
        }
    }

    // Start homing sequence (applies angles one by one)
    void homeAsync(const uint8_t (&angles)[4]) {
        for (int i = 0; i < 4; i++) _angles[i] = angles[i];
        _homingIndex = 3;  // start from the last servo
        _homingComplete = false;
        _lastHomingStep = 0;
    }

    // Call frequently; steps through servos at 300ms intervals
    void update(uint32_t nowMs) {
        if (_homingComplete) return;
        if (nowMs - _lastHomingStep >= 300) {
            setAngle(_homingIndex, _angles[_homingIndex]);
            _homingIndex--;
            _lastHomingStep = nowMs;
            if (_homingIndex < 0) _homingComplete = true;
        }
    }

    bool isHoming() const { return !_homingComplete; }
    uint8_t &angle(uint8_t idx) { return _angles[idx]; }
    const uint8_t &angle(uint8_t idx) const { return _angles[idx]; }

private:
    uint8_t _angles[4] = {110, 45, 50, 110};
    bool _homingComplete = true;
    int8_t _homingIndex = 0;
    uint32_t _lastHomingStep = 0;
    uint32_t _lastHeartbeat = 0;
    static const uint32_t _dutyMin = (500 * (1<<14)) / 20000;
    static const uint32_t _dutyMax = (2500 * (1<<14)) / 20000;
};

// ============================================================================
// MPU6050 tilt sensor — single-init guard
// ============================================================================
class MPUManager {
public:
    bool begin() {
        _initialized = mpu.begin();
        if (_initialized) {
            mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
            mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        }
        return _initialized;
    }

    bool isInitialized() const { return _initialized; }

    float updateAndGetTilt() {
        if (!_initialized) return 0;
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);
        accelX = a.acceleration.x;
        accelY = a.acceleration.y;
        accelZ = a.acceleration.z;
        // Returns tilt angle (degrees) from XZ plane
        return atan2(fabs(accelX), fabs(accelZ)) * 180.0 / PI;
    }

    // Combined pitch/roll for debugging
    float getDebugAngle() const {
        float pitch = atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0 / PI;
        float roll = atan2(-accelY, accelZ) * 180.0 / PI;
        return std::max(fabs(pitch), fabs(roll));
    }

    float accelX = 0, accelY = 0, accelZ = 0;
private:
    Adafruit_MPU6050 mpu;
    bool _initialized = false;
};

// ============================================================================
// Stepper motor driver — shared step/dir for two motors
// ============================================================================
class StepperDriver {
public:
    void begin() {
        pinMode(STEP2_PIN, OUTPUT); pinMode(DIR2_PIN, OUTPUT); pinMode(SLEEP2_PIN, OUTPUT);
        pinMode(STEP3_PIN, OUTPUT); pinMode(DIR3_PIN, OUTPUT); pinMode(SLEEP3_PIN, OUTPUT);
        digitalWrite(SLEEP2_PIN, HIGH);
        digitalWrite(SLEEP3_PIN, HIGH);
    }

    void setSpeed(int16_t speed) {
        _targetSpeed = speed;
    }

    // Call frequently; handles acceleration ramping
    void update() {
        if (_targetSpeed == 0) {
            _currentInterval = M2_MIN_DELAY;
            return;
        }
        uint32_t now = micros();
        digitalWrite(DIR2_PIN, _targetSpeed < 0 ? HIGH : LOW);
        digitalWrite(DIR3_PIN, _targetSpeed < 0 ? LOW : HIGH);
        int targetInterval = map(abs(_targetSpeed), 0, 1000, M2_MIN_DELAY, M2_MAX_DELAY);
        if (millis() - _lastAccel >= 20) {
            if (_currentInterval > targetInterval)
                _currentInterval = std::max(_currentInterval - M2_ACCEL_STEP, (float)targetInterval);
            else if (_currentInterval < targetInterval)
                _currentInterval = std::min(_currentInterval + M2_ACCEL_STEP, (float)targetInterval);
            _lastAccel = millis();
        }
        if (now - _lastStep >= (uint32_t)_currentInterval) {
            digitalWrite(STEP2_PIN, !digitalRead(STEP2_PIN));
            digitalWrite(STEP3_PIN, !digitalRead(STEP3_PIN));
            _lastStep = now;
        }
    }

    void stop() { setSpeed(0); }

private:
    int16_t _targetSpeed = 0;
    float _currentInterval = M2_MIN_DELAY;
    uint32_t _lastStep = 0, _lastAccel = 0;
    static const int M2_MIN_DELAY = 1500;
    static const int M2_MAX_DELAY = 700;
    static const int M2_ACCEL_STEP = 100;
};

// ============================================================================
// Battery monitoring
// ============================================================================
class BatteryMonitor {
public:
    enum Status { BAT_OK, BAT_LOW, BAT_CRITICAL };
    void update(MotorDriver &motor) {
        if (millis() - _lastCheck < 2000) return;
        _lastCheck = millis();
        float v = motor.readVoltage();
        if (v < 0) return;
        if (v < 5.5) status = BAT_CRITICAL;
        else if (v < 6.0) status = BAT_LOW;
        else status = BAT_OK;
        voltage = v;
    }
    Status status = BAT_OK;
    float voltage = 0;
private:
    uint32_t _lastCheck = 0;
};

// ============================================================================
// Xbox controller input — stick mapping & mutual exclusion preserved
// ============================================================================
class XboxInput {
public:
    void begin() { xbox.begin(); }
    void update() { xbox.onLoop(); connected = xbox.isConnected(); }
    bool isConnected() const { return connected; }

    // Left stick: x = forward/back, y = strafe (matches original ctrl_x/ctrl_y)
    void getLeftStick(int16_t &x, int16_t &y) const {
        x = _lxFiltered;
        y = _lyFiltered;
    }
    // Rotation (right stick X, filtered)
    int16_t getRotation() const { return _rxFiltered; }
    // Lift (right stick Y, unfiltered direct speed-curve result)
    int16_t getLift() const { return _ryValue; }

    bool isLiftActive() const { return _ryValue != 0; }
    bool isRotateActive() const { return _rxDominant; }

    XboxSeriesXControllerESP32_asukiaaa::Core &raw() { return xbox; }

    void process() {
        if (!connected) return;

        // Left stick processing
        int16_t rawY = _processAxis(xbox.xboxNotif.joyLVert);   // forward/back
        int16_t rawX = -_processAxis(xbox.xboxNotif.joyLHori);  // strafe (inverted)
        int16_t curveY = _speedCurve(rawY) * LINEAR_SENSITIVITY;
        int16_t curveX = _speedCurve(rawX) * STRAFE_SENSITIVITY;

        // ctrl_x = -ly, ctrl_y = -lx (preserves original mapping)
        _lxFiltered = _lowPass(_lxFiltered, -curveY, CTRL_ALPHA);
        _lyFiltered = _lowPass(_lyFiltered, -curveX, CTRL_ALPHA);

        // Right stick mutual exclusion (replicates original processRightJoystick exactly)
        int16_t processedRX = _processAxis(xbox.xboxNotif.joyRHori);
        int16_t processedRY = _processAxis(xbox.xboxNotif.joyRVert);
        int16_t absRX = abs(processedRX);
        int16_t absRY = abs(processedRY);

        _rxDominant = false;
        if (absRX > RIGHT_DEADZONE || absRY > RIGHT_DEADZONE) {
            if (absRX > absRY * 1.2) {
                _rxDominant = true;
                _ryValue = 0;
            } else if (absRY > absRX * 1.2) {
                _ryValue = processedRY;
                _rxDominant = false;
            } else {
                _rxDominant = true;
                _ryValue = 0;
            }
        } else {
            _ryValue = 0;
        }

        // Apply speed curve & filter to rotation value
        int16_t rxForRotate = _rxDominant ? processedRX : 0;
        rxForRotate = _speedCurve(rxForRotate);
        _rxFiltered = _lowPass(_rxFiltered, -rxForRotate * RX_SENSITIVITY, RX_ALPHA);

        // Lift value: speed curve only (no filter, no sensitivity scaling)
        if (_ryValue != 0) {
            _ryValue = _speedCurve(_ryValue);
        }
    }

private:
    int16_t _processAxis(uint16_t raw) {
        if (raw > 32767 - DEADZONE && raw < 32767 + DEADZONE) return 0;
        if (raw < 32767) return -map(raw, 0, 32767 - DEADZONE, 1000, 0);
        else return map(raw, 32767 + DEADZONE, 65535, 0, 1000);
    }
    int16_t _speedCurve(int16_t v) {
        if (v == 0) return 0;
        int16_t sign = (v > 0) ? 1 : -1;
        float norm = abs(v) / 1000.0f;
        float curved = pow(norm, CURVE_EXP);
        int16_t out = round(curved * 1000);
        if (out > 0 && out < 3) out = 3;
        return sign * out;
    }
    float _lowPass(float prev, float input, float alpha) {
        return prev * (1.0f - alpha) + input * alpha;
    }

    XboxSeriesXControllerESP32_asukiaaa::Core xbox{XBOX_MAC};
    bool connected = false;
    float _lxFiltered = 0, _lyFiltered = 0;
    float _rxFiltered = 0;
    int16_t _ryValue = 0;
    bool _rxDominant = false;

    static const int16_t DEADZONE = 600;        // joystick deadzone
    static const int16_t RIGHT_DEADZONE = 20;   // right stick deadzone
    static constexpr float CURVE_EXP = 4.0f;
    static constexpr float LINEAR_SENSITIVITY = 0.6f;
    static constexpr float STRAFE_SENSITIVITY = 0.6f;
    static constexpr float CTRL_ALPHA = 0.20f;
    static constexpr float RX_SENSITIVITY = 0.25f;
    static constexpr float RX_ALPHA = 0.25f;
};

// ============================================================================
// Mecanum kinematics & slope parking
// ============================================================================
class ChassisController {
public:
    enum Mode { IDLE, STRAIGHT, STRAFE, ROTATE, MIXED, PARKING };

    void compute(int16_t y, int16_t x, int16_t rot, bool forcePark) {
        _mode = IDLE;
        if (forcePark) {
            _mode = PARKING;
            _pwm[0] =  300; _pwm[1] = -300;
            _pwm[2] = -300; _pwm[3] =  300;
            return;
        }

        if (rot != 0 && x == 0 && y == 0) _mode = ROTATE;
        else if (y != 0 && x == 0 && rot == 0) _mode = STRAIGHT;
        else if (x != 0 && y == 0 && rot == 0) _mode = STRAFE;
        else if (x != 0 || y != 0 || rot != 0) _mode = MIXED;

        float fl = y + x + rot;
        float fr = y - x + rot;
        float bl = y - x - rot;
        float br = y + x - rot;
        float maxVal = std::max({fabs(fl), fabs(fr), fabs(bl), fabs(br)});
        if (maxVal > 1000.0f) {
            fl = fl / maxVal * 1000.0f;
            fr = fr / maxVal * 1000.0f;
            bl = bl / maxVal * 1000.0f;
            br = br / maxVal * 1000.0f;
        }
        auto toPWM = [](float v) {
            if (v == 0) return 0;
            int16_t sign = (v > 0) ? 1 : -1;
            int16_t absV = abs((int16_t)v);
            int16_t pwm = map(absV, 0, 1000, 0, 3600);
            if (pwm > 0 && pwm < 5) pwm = 5;
            return sign * pwm;
        };
        _pwm[0] = toPWM(fl); _pwm[1] = toPWM(fr);
        _pwm[2] = toPWM(bl); _pwm[3] = toPWM(br);
    }

    void getPWM(int16_t (&out)[4]) const {
        for (int i = 0; i < 4; i++) out[i] = _pwm[i];
    }
    Mode mode() const { return _mode; }

    static bool shouldPark(int16_t ly, int16_t lx, int16_t rx, int16_t ry, float tilt) {
        if (abs(ly) > 30 || abs(lx) > 30 || abs(rx) > 30 || abs(ry) > 30) return false;
        return tilt > 12.0f;
    }

private:
    int16_t _pwm[4] = {0};
    Mode _mode = IDLE;
};

// ============================================================================
// Relay control — non-blocking
// ============================================================================
class RelayControl {
public:
    void begin() { pinMode(RELAY_PIN, OUTPUT); set(true); }
    void set(bool on) { digitalWrite(RELAY_PIN, on ? HIGH : LOW); _state = on; }
    bool state() const { return _state; }
    void triggerReset() { _resetActive = true; _resetStart = millis(); set(false); }
    void update() {
        if (_resetActive && millis() - _resetStart >= 5000) {
            set(true);
            _resetActive = false;
        }
    }
    bool isResetting() const { return _resetActive; }
private:
    bool _state = true;
    bool _resetActive = false;
    uint32_t _resetStart = 0;
};

// ============================================================================
// Servo button handler — 40ms throttle
// ============================================================================
class ServoButtonHandler {
public:
    bool handle(XboxInput &xbox, ServoController &servo, uint32_t nowMs) {
        if (servo.isHoming()) return false;
        if (nowMs - _lastUpdate < 40) return false;
        _lastUpdate = nowMs;

        bool update = false;
        uint8_t &s0 = servo.angle(0), &s1 = servo.angle(1);
        uint8_t &s2 = servo.angle(2), &s3 = servo.angle(3);

        if (xbox.raw().xboxNotif.btnShare) { servo.homeAsync({50, 120, 50, 130}); return true; }
        if (xbox.raw().xboxNotif.btnStart)  { servo.homeAsync({116, 53, 50, 130}); return true; }
        if (xbox.raw().xboxNotif.btnSelect){ servo.homeAsync({74, 145, 48, 130}); return true; }

        const int STEP = 2;
        if (xbox.raw().xboxNotif.trigRT > 512)     { s0 = constrain(s0+STEP, 36, 126); s1 = constrain(s1-STEP, 0, 150); update = true; }
        if (xbox.raw().xboxNotif.btnRB)             { s0 = constrain(s0-STEP, 36, 126); s1 = constrain(s1+STEP, 0, 150); update = true; }
        if (xbox.raw().xboxNotif.btnA)              { s3 = constrain(s3+STEP, 83, 135); update = true; }
        if (xbox.raw().xboxNotif.btnB)              { s3 = constrain(s3-STEP, 83, 135); update = true; }
        if (xbox.raw().xboxNotif.btnX)              { s2 = constrain(s2+STEP, 0, 180);  update = true; }
        if (xbox.raw().xboxNotif.btnY)              { s2 = constrain(s2-STEP, 0, 180);  update = true; }
        if (xbox.raw().xboxNotif.btnDirUp)          { s1 = constrain(s1-STEP, 0, 150);  update = true; }
        if (xbox.raw().xboxNotif.btnDirDown)        { s1 = constrain(s1+STEP, 0, 150);  update = true; }
        if (xbox.raw().xboxNotif.btnDirLeft)        { s0 = constrain(s0+STEP, 36, 126); update = true; }
        if (xbox.raw().xboxNotif.btnDirRight)       { s0 = constrain(s0-STEP, 36, 126); update = true; }

        if (update) {
            servo.setAngle(0, s0); servo.setAngle(1, s1);
            servo.setAngle(2, s2); servo.setAngle(3, s3);
        }
        return update;
    }
private:
    uint32_t _lastUpdate = 0;
};

// ============================================================================
// Global objects
// ============================================================================
MotorDriver motor;
ServoController servo;
MPUManager mpu;
StepperDriver stepper;
BatteryMonitor battery;
XboxInput xbox;
ChassisController chassis;
RelayControl relay;
ServoButtonHandler servoButtons;

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    motor.begin();
    servo.begin();
    stepper.begin();
    relay.begin();

    if (!mpu.begin()) Serial.println("MPU6050 init failed");

    xbox.begin();
    Serial.println("System init complete");
}

// ============================================================================
// Main loop — 20ms control throttle
// ============================================================================
void loop() {
    uint32_t now = millis();
    xbox.update();
    relay.update();

    if (!xbox.isConnected()) {
        motor.stop();
        return;
    }

    // Home button triggers relay reset
    static bool prevHome = false;
    bool curHome = xbox.raw().xboxNotif.btnXbox;
    if (curHome && !prevHome) relay.triggerReset();
    prevHome = curHome;

    // Process input every loop for responsive button handling
    xbox.process();

    // Stepper motor — no throttle, runs every loop
    int16_t lift = xbox.getLift();
    if (xbox.isLiftActive()) stepper.setSpeed(lift);
    else stepper.stop();
    stepper.update();

    // Servo button handling (40ms throttle inside)
    servoButtons.handle(xbox, servo, now);
    servo.heartbeat(now);
    servo.update(now);

    // Main control logic runs at 20ms intervals
    static uint32_t lastControlUpdate = 0;
    if (now - lastControlUpdate >= 20) {
        lastControlUpdate = now;

        // Sensors & battery (battery has internal 2s throttle)
        float tilt = mpu.isInitialized() ? mpu.updateAndGetTilt() : 0;
        battery.update(motor);

        // Read left stick (lx=forward/back, ly=strafe)
        int16_t ly, lx;
        xbox.getLeftStick(lx, ly);    // lx=forward/back, ly=strafe
        int16_t rot = xbox.getRotation();

        // Parking check (includes lift axis)
        bool forcePark = ChassisController::shouldPark(ly, lx, rot, lift, tilt);

        // Mecanum kinematics solve
        if (forcePark) { ly = lx = rot = 0; }
        chassis.compute(ly, lx, rot, forcePark);

        // Output PWM to motors
        int16_t pwm[4];
        chassis.getPWM(pwm);
        if (battery.status == BatteryMonitor::BAT_CRITICAL)
            motor.stop();
        else
            motor.setPWM(pwm);

        // Debug output every 500ms
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 500) {
            Serial.printf("Mode:%d Park:%d Tilt:%.1f V:%.2f | Joy:%d,%d Rot:%d Lift:%d | Servo:%d,%d,%d,%d | Relay:%d\n",
                chassis.mode(), forcePark, tilt, battery.voltage,
                ly, lx, rot, lift,
                servo.angle(0), servo.angle(1), servo.angle(2), servo.angle(3),
                relay.state());
            lastPrint = millis();
        }
    }
}
