# ESP32 Flight Controller Drone

Experimental ESP32 DevKit V1 flight controller firmware using ESP-IDF C++17.

## Hardware Target

- **MCU**: ESP32 DevKit V1 (classic ESP32)
- **IMU**: MPU6050 (I2C 0x68)
- **Barometer**: BMP280 (I2C 0x76/0x77)
- **ESCs**: 30A x4
- **Motors**: 2212 2200KV brushless x4
- **Future**: Android WiFi telemetry/config/control app

## Current Phase: Prompt 2B - I2C scan hardware result (recorded)

This phase adds I2C master bus initialization and address scanning using the ESP-IDF v6 handle-based API.

### What exists:
- Root CMakeLists.txt with C++17 configuration
- main/app_main.cpp with boot banner, safety warnings, and I2C init/scan
- components/board_config/ with hardware constants
- components/i2c_bus/ with I2C master bus init and scan
- sdkconfig.defaults with minimal ESP32 target
- docs/current_phase.txt phase marker

### What does NOT exist (forbidden in this phase):
- ❌ MPU6050 driver (WHO_AM_I read, FIFO, config)
- ❌ BMP280 driver (chip ID, calibration, compensated read)
- ❌ MCPWM / motor output
- ❌ WiFi / Android app
- ❌ PID controller
- ❌ Receiver input
- ❌ Safety state machine

## Safety Warning

⚠️ **THIS FIRMWARE HAS NO MOTOR OUTPUT CAPABILITY IN THIS PHASE**

- No MCPWM initialization
- No ESC signal generation
- No motor test mode
- No arming logic
- No safety state machine

**Do not connect ESCs, motors, or propellers.** This phase only scans I2C bus and blinks an optional onboard LED.

## Build

```bash
# From project root
cmd /c "C:\Akmal\Project-tools\.espressif\v6.0.1\esp-idf\export.bat && idf.py build"
```

## Flash and Monitor (COM3)

```bash
cmd /c "C:\Akmal\Project-tools\.espressif\v6.0.1\esp-idf\export.bat && idf.py -p COM3 flash monitor"
```

## Expected Results

### If no sensors connected:
```
I (xxx) main: ========================================
I (xxx) main: ESP32 Flight Controller Drone
I (xxx) main: Phase: Prompt 2 - I2C bus scan
...
I (xxx) i2c_bus: Initializing I2C master bus
I (xxx) i2c_bus:   SDA GPIO: 21
I (xxx) i2c_bus:   SCL GPIO: 22
I (xxx) i2c_bus:   Frequency: 400000 Hz
I (xxx) i2c_bus: I2C master bus initialized
I (xxx) i2c_bus: Scanning I2C bus for devices...
W (xxx) i2c_bus: I2C scan complete: 0 device(s) found. Sensors may not be connected.
```

### If MPU6050 connected:
```
I (xxx) i2c_bus: I2C device found at 0x68
I (xxx) i2c_bus:   0x68 -> possible MPU6050 (not confirmed)
```

### If BMP280 connected:
```
I (xxx) i2c_bus: I2C device found at 0x76
I (xxx) i2c_bus:   0x76 -> possible BMP280 (not confirmed)
```

## Prompt 2 Hardware Result

**Hardware test**: ESP32 DevKit V1 / ESP-WROOM-32, **no sensors connected**.
**Result**: I2C scan on SDA GPIO21, SCL GPIO22, 400000 Hz found **0 devices**.

This is a **PASS** for the bus-scan firmware phase. Zero devices expected when no MPU6050 or BMP280 sensors connected to I2C bus. I2C bus init and scan logic execute correctly. LED blink on GPIO2 continues. No crash.

**Prompt 3 hardware success requires**:
- MPU6050 detected at I2C address `0x68`
- `WHO_AM_I` register returning `0x68`

⚠️ **Safety warning remains**: No ESCs, motors, or propellers connected during any Prompt 2 or Prompt 3 testing.

## Next Phase

Prompt 3: MPU6050 detection and WHO_AM_I read (only after I2C scan hardware result recorded).

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