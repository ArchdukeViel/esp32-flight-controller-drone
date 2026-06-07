# ESP32 Flight Controller Drone

Experimental ESP32 DevKit V1 flight controller firmware using ESP-IDF C++17.

## Hardware Target

- **MCU**: ESP32 DevKit V1 (classic ESP32)
- **IMU**: MPU6050 (I2C 0x68)
- **Barometer**: BMP280 (I2C 0x76/0x77)
- **ESCs**: 30A x4
- **Motors**: 2212 2200KV brushless x4
- **Future**: Android WiFi telemetry/config/control app

## Current Phase: Prompt 10 - MCPWM Motor Output

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
- sdkconfig.defaults with minimal ESP32 target
- docs/current_phase.txt phase marker

### What does NOT exist yet:
- ❌ Safety state machine (arming/disarming/pre-arm checks/failsafe)
- ❌ Receiver input
- ❌ WiFi telemetry/control
- ❌ Android app
- ❌ Motor-output hardware verification with ESCs connected

## Safety Warning

⚠️ **MOTOR OUTPUT DRIVER EXISTS BUT SAFETY STATE MACHINE IS NOT IMPLEMENTED**

Current firmware state:
- MCPWM motor output driver is initialized and drives ESC pins (GPIO 18, 19, 23, 25)
- Outputs default to 1000 µs idle pulse on boot
- Motors remain DISARMED in current code (motor_output_arm() never called)
- No safety state machine, no arming checks, no failsafe logic
- Hardware motor gate: NOT PASSED (build-only verification)

**KEEP PROPELLERS REMOVED.** Do not connect ESCs or motors for testing until Prompt 11 Safety State Machine is implemented and hardware-verified. ESC signal capability exists but is not safety-gated.

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