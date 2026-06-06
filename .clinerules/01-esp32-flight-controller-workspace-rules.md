# ESP32 DevKit V1 Flight Controller Workspace Rules

These rules apply only to this workspace. They are written for the ESP32 DevKit V1 flight-controller project using MPU6050, BMP280, 30A ESCs, and 2212 2200KV brushless motors.

## 1. Role and operating model

- Act as the implementation agent for this repository, not the project architect.
- Treat the canonical roadmap/design plan as the source of truth. If a request conflicts with the roadmap, stop and explain the conflict before editing.
- Work in small, reviewable phases. Do not jump ahead to PID, motor output, WiFi control, or propeller-related work before earlier gates pass.
- Do not make broad cleanup, refactors, file moves, dependency changes, or architecture changes unless the user explicitly asks.
- Do not create commits, branches, tags, releases, or push to remote unless explicitly requested.
- Always report:
  - files created/modified,
  - build/test commands run,
  - pass/fail result,
  - safety impact,
  - remaining risks,
  - next recommended gate.

## 2. Project constants and corrected hardware facts

Use these constants unless the user explicitly changes the hardware:

- Target MCU: classic ESP32 DevKit V1.
- Framework: ESP-IDF v5.5.x preferred.
- Language: C++17 for project components.
- No Arduino core in the real-time control path.
- MPU6050 I2C address: usually `0x68`.
- MPU6050 `WHO_AM_I` expected value: `0x68`.
- BMP280 I2C address: usually `0x76` or `0x77`.
- BMP280 chip ID expected value: `0x58`.
- If BMP/BME chip ID is `0x60`, do not pretend it is BMP280. Treat it as a possible BME280 or unsupported variant unless explicitly supported.
- ESC pulse range: `1000-2000 us`.
- Disarmed ESC pulse: `1000 us`.
- Initial safe ESC update frequency: `50 Hz`.
- Initial flight/control loop target: `250 Hz` / `4000 us`.
- RMT is the primary method for RC PWM capture.
- GPIO edge interrupt capture is fallback only.
- MCPWM is required for ESC output.
- BMP280 is optional for attitude-only flight and must not block the fast loop.

## 3. Safety is law

This project controls rotating motors. Never optimize around safety gates.

- Never allow propeller-related testing unless all no-prop gates have already passed and the user explicitly requests the propeller phase.
- Never write code that bypasses arming, pre-arm checks, failsafe, emergency stop, receiver validity, IMU health, or motor-output gating.
- Never write motor output directly from WiFi, UART, receiver, test code, or PID. All motor commands must pass through:
  1. input validation,
  2. safety state machine,
  3. arming manager,
  4. failsafe manager,
  5. mixer or motor-test gate,
  6. MCPWM output clamp.
- On boot, all ESC outputs must initialize to `1000 us` before any motor-capable state is entered.
- Emergency stop must force all outputs to `1000 us` immediately and must override ramp limiting.
- Disarmed, failsafe, and error-lockout states must always command `1000 us`.
- Motor test mode must require an explicit no-prop confirmation and must cap throttle to low pulse values.
- Do not claim a physical safety test passed unless the user provides measured or observed results.

## 4. Phase discipline

Follow this implementation order unless the user explicitly overrides it:

1. ESP-IDF skeleton, board config, serial boot log.
2. I2C bus scan.
3. MPU6050 detection, raw read, scaled read.
4. MPU6050 calibration and NVS storage.
5. BMP280 detection, compensated pressure/temperature/altitude.
6. Sensor telemetry and validation.
7. Attitude estimator.
8. Receiver input or stub input.
9. MCPWM ESC signal generation with ESC power disconnected.
10. Motor-test state machine with no-prop confirmation.
11. Safety state machine, arming manager, failsafe manager.
12. PID controller unit tests with fake input.
13. PID integrated with IMU but motors forced disabled.
14. Quad X mixer unit tests.
15. Integrated dry-run with motor outputs still safety-gated.
16. No-prop motor spin test.
17. WiFi telemetry/config/PID tuning while motor output disabled.
18. Android app telemetry/config interface.
19. Experimental Android WiFi command input dry-run with motor output disabled.
20. No-prop WiFi command experiment.
21. Pre-propeller readiness review.
22. Tethered/very-low hover only after all prior gates pass.

If the user asks for a later phase before earlier prerequisites exist, pause and say which gates are missing.

## 5. Repository structure rules

Prefer this component layout:

```txt
main/
  app_main.cpp
components/
  board_config/
  i2c_bus/
  mpu6050/
  bmp280/
  sensor_calibration/
  sensor_validation/
  estimation/
  receiver/
  control/
  motor_output/
  safety/
  telemetry/
  config/
  diagnostics/
tests/
docs/
```

For each ESP-IDF component:

- Use `include/<component>.h` for public interfaces.
- Use `src/<component>.cpp` for implementation.
- Use a local `CMakeLists.txt`.
- Keep public headers small and typed.
- Do not dump logic into `main/app_main.cpp`.
- `app_main.cpp` should initialize modules, create tasks, and coordinate high-level boot state only.

## 6. C++ and embedded coding rules

- Use C++17.
- Avoid exceptions and RTTI.
- Avoid STL containers in real-time paths.
- Avoid dynamic allocation in real-time paths.
- Prefer static allocation for FreeRTOS tasks, queues, and buffers where practical.
- Return `esp_err_t` or typed status enums from hardware-facing functions.
- Never ignore ESP-IDF return codes.
- Use explicit structs for data exchange:
  - `ImuRawSample`
  - `ImuScaledSample`
  - `BaroSample`
  - `SensorHealth`
  - `AttitudeEstimate`
  - `ReceiverInput`
  - `ControlCommand`
  - `PIDOutput`
  - `MotorCommand`
  - `SystemHealth`
- Keep math units explicit in names:
  - `_us` for microseconds,
  - `_hz` for frequency,
  - `_dps` for degrees per second,
  - `_deg` for degrees,
  - `_pa` for pressure,
  - `_m` for meters,
  - `_g` for acceleration in g.

## 7. Real-time and tasking rules

- The fast control path must not block on:
  - BMP280 reads,
  - WiFi,
  - NVS writes,
  - telemetry formatting,
  - serial input,
  - file/storage operations.
- MPU6050 reads must use short timeouts and error counters.
- BMP280 must run in a slow task and must not be read inside the fast loop.
- WiFi must not run during normal active motor control.
- Telemetry must be queued or snapshotted; do not format/log verbose strings inside the fast loop.
- Use `ESP_LOGx` for lifecycle and errors, but not high-rate telemetry in the fast loop.
- Use task pinning deliberately. Keep flight-critical tasks isolated from WiFi/background work.

## 8. Sensor rules

### MPU6050

- Configure once during initialization, not inside every read.
- Use a 14-byte burst read for accel/temp/gyro.
- Validate `WHO_AM_I == 0x68`.
- Apply gyro/accel calibration from NVS if valid.
- Require stillness before calibration.
- If IMU is missing or persistently failing, enter `ERROR_LOCKOUT` or equivalent critical fault state.

### BMP280

- Validate chip ID `0x58`.
- Read calibration coefficients before compensated measurements.
- Treat BMP280 as optional for attitude-only operation.
- Mark altitude invalid if BMP280 is missing or stale.
- Never use BMP280 for fast altitude control in early phases.

## 9. Estimation and PID rules

- Start with a complementary filter for roll and pitch.
- Do not claim absolute yaw angle without a magnetometer.
- Yaw control is rate-only using gyro Z rate.
- PID must be disabled unless safety state allows active stabilization.
- Reset PID integrators on:
  - disarm,
  - arming transition,
  - failsafe,
  - error lockout,
  - throttle cutoff where applicable.
- Implement anti-windup.
- Prefer derivative-on-measurement or filtered derivative to avoid derivative kick.
- Enforce PID gain bounds.
- Log PID terms at reduced telemetry rate, not every fast-loop cycle unless explicitly requested.

## 10. Motor and mixer rules

Use one motor numbering convention everywhere:

```txt
         FRONT
    M1           M2
 front-left   front-right
      \        /
       \      /
        \    /
        /    \
       /      \
      /        \
 rear-left   rear-right
    M4           M3
         REAR
```

- Do not introduce a second motor numbering convention.
- Every motor-test message must include both motor number and physical location.
- Quad X mixer signs must be unit-tested before motor use.
- Clamp motor outputs to `[1000, 2000] us`.
- Disarmed/failsafe/error output must be `[1000, 1000, 1000, 1000] us`.
- Motor-test mode must spin only selected motors and must auto-timeout.
- Ramp limiting must never delay emergency stop.

## 11. WiFi and Android control rules

The Android app is not the safety system. The ESP32 firmware safety state machine owns safety.

### Allowed early WiFi features

Before no-prop integration is complete, WiFi may only be used for:

- telemetry viewing,
- sensor status,
- calibration commands,
- PID viewing/editing while motor output is disabled,
- configuration viewing,
- logs,
- non-motor diagnostics.

### Forbidden early WiFi features

Do not implement active throttle/attitude motor control over WiFi until the roadmap reaches the WiFi command dry-run phase.

### WiFi control architecture

When active WiFi command input is eventually implemented:

- ESP32 should run local AP mode first. Do not depend on internet/cloud routing.
- Use HTTP or WebSocket for telemetry/config if useful.
- Use UDP for low-latency command packets only after dry-run phases.
- Every control packet must include:
  - magic value,
  - protocol version,
  - sequence number,
  - timestamp or monotonic counter,
  - command mode,
  - roll/pitch/yaw/throttle command,
  - arm/disarm request,
  - dead-man flag,
  - CRC/checksum.
- Packet loss or stale packets must command neutral input and then failsafe.
- Missing command packets for more than the configured watchdog window must cut motor output.
- App backgrounding, screen lock, network loss, or dead-man release must command immediate neutral/disarm behavior.
- WiFi commands must never write MCPWM directly.
- WiFi commands must become `ControlCommand` requests, then pass through the same validation/safety pipeline as receiver input.

### Android app rules

- Start with the simplest interface:
  - connection status,
  - sensor telemetry,
  - safety state,
  - calibration button,
  - PID fields,
  - emergency stop button.
- Active joystick/throttle UI comes later.
- Throttle UI must be spring-return or dead-man gated.
- Emergency stop must be visible and reachable at all times.
- Do not hide safety state behind pretty UI. Show it plainly.

## 12. Testing rules

Every phase must have measurable pass/fail criteria.

Required test categories:

- Build test: `idf.py build`
- Flash/monitor test when hardware is available.
- Unit tests for:
  - PID controller,
  - motor mixer,
  - safety state machine,
  - sensor validation,
  - attitude estimator.
- Hardware tests for:
  - I2C scan,
  - MPU6050 `WHO_AM_I`,
  - BMP280 chip ID,
  - MCPWM pulse widths,
  - receiver input timing,
  - emergency stop latency.
- Long-run tests before integration:
  - sensor telemetry run,
  - no I2C error check,
  - free heap monitoring,
  - loop timing monitoring.

Never mark a hardware phase complete without the user’s measured output or log.

## 13. Documentation and reporting rules

- Update documentation only when it directly reflects implemented behavior.
- Do not generate huge new markdown files unless requested.
- Keep phase notes short and factual.
- When making safety-relevant changes, include a "Safety impact" section in the report.
- When making timing-relevant changes, include loop timing or expected timing impact.
- When making WiFi changes, include how motor output remains gated.

## 14. Files and secrets

- Do not commit secrets, WiFi passwords, API keys, tokens, private keys, or local machine paths.
- Use example config files for secrets.
- Do not edit `.gitignore`, `.env`, partition tables, sdkconfig defaults, or build config unless needed for the current phase.
- If modifying build/system config, explain exactly why.

## 15. Failure behavior

When uncertain:

- Prefer disarmed output.
- Prefer explicit error.
- Prefer `ERROR_LOCKOUT` over silent recovery for critical IMU/motor-output faults.
- Prefer asking the user for measurements over guessing.
- Prefer stopping at the current phase over speculative implementation.

The machine may be small, but the blades are not philosophical. Safety first, then elegance.
