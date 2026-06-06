# ESP32 Flight Controller Drone

Experimental ESP32 DevKit V1 flight controller firmware using ESP-IDF C++17.

## Hardware Target

- **MCU**: ESP32 DevKit V1 (classic ESP32)
- **IMU**: MPU6050 (I2C 0x68)
- **Barometer**: BMP280 (I2C 0x76/0x77)
- **ESCs**: 30A x4
- **Motors**: 2212 2200KV brushless x4
- **Future**: Android WiFi telemetry/config/control app

## Current Phase: Prompt 1 - Skeleton + board_config only

This phase creates the minimal ESP-IDF project structure and board configuration constants.

### What exists:
- Root CMakeLists.txt with C++17 configuration
- main/app_main.cpp with boot banner and safety warnings
- components/board_config/ with hardware constants
- sdkconfig.defaults with minimal ESP32 target
- docs/current_phase.txt phase marker

### What does NOT exist (forbidden in this phase):
- ❌ Sensor drivers (MPU6050, BMP280)
- ❌ I2C bus implementation
- ❌ MCPWM / motor output
- ❌ WiFi / Android app
- ❌ PID controller
- ❌ Receiver input
- ❌ Safety state machine
- ❌ Any components beyond board_config

## Safety Warning

⚠️ **THIS FIRMWARE HAS NO MOTOR OUTPUT CAPABILITY IN THIS PHASE**

- No MCPWM initialization
- No ESC signal generation
- No motor test mode
- No arming logic
- No safety state machine

**Do not connect ESCs, motors, or propellers.** This phase only prints a boot banner and blinks an optional onboard LED.

## Build

```bash
# From project root
cmd /c "C:\Akmal\Project-tools\.espressif\v6.0.1\esp-idf\export.bat && idf.py build"
```

## Next Phase

Prompt 2: I2C bus scan (only after build passes).

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