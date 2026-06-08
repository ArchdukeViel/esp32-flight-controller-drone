# ESP32 Flight Controller Drone

Experimental ESP32 DevKit V1 flight controller firmware using ESP-IDF C++17.

## Hardware Target

- **MCU**: ESP32 DevKit V1 (classic ESP32)
- **IMU**: MPU6050 (I2C 0x68)
- **Barometer**: BMP280 (I2C 0x76/0x77)
- **ESCs**: 30A x4
- **Motors**: 2212 2200KV brushless x4
- **Future**: Android WiFi telemetry/config/control app

## Current Phase: Prompt 11 - Safety State Machine

**Build gate: PASSED** | **Hardware gate: PENDING**

See `docs/current_phase.txt` for detailed status and hardware test results.

### What exists:
- Root CMakeLists.txt with C++17 configuration
- main/app_main.cpp with boot banner, I2C init, sensor init, estimator/PID/mixer/motor-output pipeline, LED control loop
- components/board_config/ with hardware constants and ESC pin definitions
- components/i2c_bus/ with I2C master bus init and scan
- components/mpu6050/ with detection, raw/scaled read, calibration, NVS storage
- components/bmp280/ with detection, compensated temperature/pressure/altitude read
- components/estimator/ with complementary filter (attitude) and altitude fusion
- components/pid/ with 3-axis PID controller and anti-windup
- components/motor_mixer/ with Quad X mixing matrix
- components/motor_output/ with MCPWM 4-channel ESC driver (build-verified only)
- components/safety/ with safety state machine (BOOT → DISARMED → PRE_ARM_CHECK → ARMED, plus FAILSAFE and ERROR), pre-arm checks, failsafe on sensor/estimator loss, error lockout, and emergency stop
- app_main.cpp gates all motor output through `safety_is_motor_output_allowed()` (true only when ARMED)
- sdkconfig.defaults with minimal ESP32 target
- docs/current_phase.txt phase marker

### What does NOT exist yet:
- ❌ Receiver input
- ❌ WiFi telemetry/control
- ❌ Android app
- ❌ Motor-output hardware verification with ESCs connected

## Safety Warning

⚠️ **MOTOR OUTPUT DRIVER EXISTS AND IS SAFETY-GATED; ESC/MOTOR HARDWARE TESTING IS STILL PENDING**

Current firmware state:
- MCPWM motor output driver is initialized and drives ESC pins (GPIO 18, 19, 23, 25)
- Outputs default to 1000 µs idle pulse on boot
- Safety state machine is implemented and integrated (build-gated); all motor output above idle passes through `safety_is_motor_output_allowed()`, which is true only when ARMED
- Non-ARMED states (BOOT, DISARMED, PRE_ARM_CHECK, FAILSAFE, ERROR) force 1000 µs idle output
- Emergency stop and failsafe (on sensor/estimator loss) are implemented; no automatic arming path
- Hardware motor gate: NOT PASSED (build-only verification; ESCs not connected/tested)

**KEEP PROPELLERS REMOVED.** Do not connect ESCs or motors for testing until the no-prop hardware gates have passed and you explicitly request the motor test phase. ESC signal output is now safety-gated in firmware, but ESC/motor hardware behavior has not been validated.

## Build

```bash
# From project root
cmd /c "C:\Akmal\Project-tools\.espressif\v6.0.1\esp-idf\export.bat && idf.py build"
```

## Flash and Monitor (COM3)

```bash
cmd /c "C:\Akmal\Project-tools\.espressif\v6.0.1\esp-idf\export.bat && idf.py -p COM3 flash monitor"
```

## Next Phase

**Prompt 11: Safety State Machine**

Implements arming/disarming logic, pre-arm checks, failsafe behavior, and error lockout states. Required before any motor-output hardware testing with ESCs connected.

See `docs/current_phase.txt` for detailed phase history and hardware test results from earlier prompts.

## References

- Workspace rules: `.clinerules/01-esp32-flight-controller-workspace-rules.md`
- Canonical roadmap: `docs/esp32_flight_controller_canonical_merged_plan_wifi_android.txt`
- Current phase: `docs/current_phase.txt`

## Motor Numbering Convention (Quad X)

```
         FRONT
    M1 (CCW)     M2 (CW)
 front-left   front-right
      \        /
       \      /
        \    /
        /    \
       /      \
      /        \
 rear-left   rear-right
    M4 (CW)      M3 (CCW)
         REAR