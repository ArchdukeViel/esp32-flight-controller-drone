# ESP32 Flight Controller Workspace Rules

These rules apply only to this workspace:

```txt
C:\Akmal\esp32-flight-controller-drone
```

Project: experimental ESP32 DevKit V1 flight-controller firmware using ESP-IDF C++17.

Hardware target:
- ESP32 DevKit V1
- MPU6050
- BMP280
- 30A ESCs
- 2212 2200KV brushless motors
- Future simple Android WiFi telemetry/config/control app

Canonical reference document:

```txt
docs/esp32_flight_controller_canonical_merged_plan_wifi_android.txt
```

This document is the full roadmap/design reference. Treat it as the project library, not as context that must be read every run.

---

## 1. Role and operating model

- Act as the implementation agent for this repository.
- Do not act as the project architect unless explicitly asked.
- Follow these workspace rules first.
- Treat the canonical roadmap/design document as the source of truth for project direction.
- If a user request conflicts with these rules or the canonical roadmap, stop and report the conflict before editing.
- Work in small, reviewable phases.
- Do not jump ahead to sensors, PID, motor output, WiFi control, Android app, or propeller-related work before earlier gates pass.
- Do not make broad cleanup, refactors, file moves, dependency changes, or architecture changes unless explicitly requested.
- Do not create commits, branches, tags, releases, or push to remote unless explicitly requested.

Every report must include:
- files created/modified,
- commands run,
- build/test result,
- assumptions,
- safety impact,
- remaining risks,
- next recommended gate.

Keep reports short unless debugging requires detail.

---

## 2. Context and token economy rules

The model provider may have RTK/token-saving enabled, but do not rely on that. Reduce unnecessary context yourself.

### Do not read everything by default

- Do not read the full canonical roadmap on every task.
- Do not summarize the full roadmap unless explicitly asked.
- Do not scan the entire repository unless the task requires it.
- Do not paste full file contents into the final report unless there is an error that requires it.

### Preferred context order

For most tasks, inspect only:

```txt
.clinerules/
docs/current_phase.txt
.vscode/settings.json
root directory listing
files directly involved in the requested phase
```

Only inspect the full canonical roadmap when:
- the current phase is unclear,
- a design decision is needed,
- safety boundaries are ambiguous,
- the user explicitly asks,
- the current task references a roadmap section or prompt.

### Recommended current phase file

If present, use this file as the immediate task guide:

```txt
docs/current_phase.txt
```

This file should be small and phase-specific. Prefer reading it instead of the full roadmap.

If `docs/current_phase.txt` is missing, ask whether to create it or proceed only from the user prompt and these workspace rules.

### Output limits

Unless the user asks for detail:
- Keep reports under 80 lines.
- Summarize diffs by filename.
- Do not paste generated code unless needed.
- Do not include long terminal logs; summarize the important error lines only.
- If build fails, include the first relevant error and the likely cause.

---

## 3. Toolchain assumptions

Current intended ESP-IDF setup:

```txt
ESP-IDF: v6.0.1
ESP-IDF path: C:\Akmal\Project-tools\.espressif\v6.0.1\esp-idf
```

When running ESP-IDF commands from a normal terminal, use:

```bat
cmd /c "C:\Akmal\Project-tools\.espressif\v6.0.1\esp-idf\export.bat && <command>"
```

Example:

```bat
cmd /c "C:\Akmal\Project-tools\.espressif\v6.0.1\esp-idf\export.bat && idf.py build"
```

If ESP-IDF tools are unavailable:
- Do not guess.
- Report the missing tool clearly.
- Do not create implementation files until the toolchain gate passes, unless the user explicitly asks for file-only scaffolding without build verification.

---

## 4. Corrected hardware facts and constants

Use these constants unless the user explicitly changes the hardware.

### MCU and framework

- Target MCU: classic ESP32 DevKit V1.
- Framework: ESP-IDF v6.0.1 for this workspace unless changed by the user.
- Language: C++17.
- No Arduino core in the real-time control path.

### MPU6050

- I2C address: usually `0x68`.
- `WHO_AM_I` expected value: `0x68`.
- If `0x70` or `0x71` appears, do not silently accept it as MPU6050. Identify the actual IMU variant first.

### BMP280

- I2C address: usually `0x76` or `0x77`.
- BMP280 chip ID expected value: `0x58`.
- If chip ID is `0x60`, treat it as possible BME280 or unsupported variant unless support is explicitly added.

### ESC

- ESC pulse range: `1000-2000 us`.
- Disarmed pulse: `1000 us`.
- Initial safe ESC update frequency: `50 Hz`.
- Initial control loop target: `250 Hz` / `4000 us`.

### Receiver and timing

- RMT is the primary method for RC PWM capture.
- GPIO edge interrupt capture is fallback only.
- MCPWM is required for ESC output.
- BMP280 is optional for attitude-only flight and must not block the fast loop.

---

## 5. Safety is law

This project controls rotating motors. Never optimize around safety gates.

- Never allow propeller-related testing unless all no-prop gates have passed and the user explicitly requests the propeller phase.
- Never bypass arming, pre-arm checks, failsafe, emergency stop, receiver validity, IMU health, or motor-output gating.
- Never write motor output directly from WiFi, UART, receiver, test code, or PID.
- All motor-capable commands must pass through:
  1. input validation,
  2. safety state machine,
  3. arming manager,
  4. failsafe manager,
  5. mixer or motor-test gate,
  6. MCPWM output clamp.
- On boot, all ESC outputs must initialize to `1000 us` before any motor-capable state is entered.
- Emergency stop must force all outputs to `1000 us` immediately and must override ramp limiting.
- Disarmed, failsafe, and error-lockout states must always command `1000 us`.
- Motor test mode must require explicit no-prop confirmation and low throttle caps.
- Do not claim a physical safety test passed unless the user provides measured or observed results.

---

## 6. Phase discipline

Follow this order unless explicitly overridden:

1. Workspace/toolchain readiness.
2. ESP-IDF skeleton, board config, serial boot log.
3. I2C bus scan.
4. MPU6050 detection, raw read, scaled read.
5. MPU6050 calibration and NVS storage.
6. BMP280 detection, compensated pressure/temperature/altitude.
7. Sensor telemetry and validation.
8. Attitude estimator.
9. Receiver input or stub input.
10. MCPWM ESC signal generation with ESC power disconnected.
11. Motor-test state machine with no-prop confirmation.
12. Safety state machine, arming manager, failsafe manager.
13. PID controller unit tests with fake input.
14. PID integrated with IMU but motors forced disabled.
15. Quad X mixer unit tests.
16. Integrated dry-run with motor outputs still safety-gated.
17. No-prop motor spin test.
18. WiFi telemetry/config/PID tuning while motor output disabled.
19. Android app telemetry/config interface.
20. Experimental Android WiFi command input dry-run with motor output disabled.
21. No-prop WiFi command experiment.
22. Pre-propeller readiness review.
23. Tethered/very-low hover only after all prior gates pass.

If the user asks for a later phase before prerequisites exist:
- stop,
- list missing gates,
- ask for confirmation before proceeding.

---

## 7. Repository structure rules

Prefer this layout:

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
- Do not dump component logic into `main/app_main.cpp`.
- `app_main.cpp` should initialize modules, create tasks, and coordinate high-level boot state only.

---

## 8. C++ and embedded coding rules

- Use C++17.
- Avoid exceptions and RTTI unless there is a deliberate reason.
- Avoid STL containers in real-time paths.
- Avoid dynamic allocation in real-time paths.
- Prefer static allocation for FreeRTOS tasks, queues, and buffers where practical.
- Return `esp_err_t` or typed status enums from hardware-facing functions.
- Never ignore ESP-IDF return codes.
- Keep units explicit in names:
  - `_us` for microseconds,
  - `_hz` for frequency,
  - `_dps` for degrees per second,
  - `_deg` for degrees,
  - `_pa` for pressure,
  - `_m` for meters,
  - `_g` for acceleration in g.

Use explicit structs for data exchange:
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

---

## 9. Real-time and tasking rules

The fast control path must not block on:
- BMP280 reads,
- WiFi,
- NVS writes,
- telemetry formatting,
- serial input,
- file/storage operations.

Additional rules:
- MPU6050 reads must use short timeouts and error counters.
- BMP280 must run in a slow task and must not be read inside the fast loop.
- WiFi must not run during normal active motor control.
- Telemetry must be queued or snapshotted.
- Do not format/log verbose strings inside the fast loop.
- Use `ESP_LOGx` for lifecycle and errors, not high-rate telemetry in the fast loop.
- Use task pinning deliberately. Keep flight-critical tasks isolated from WiFi/background work where possible.

---

## 10. Sensor rules

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

---

## 11. Estimation and PID rules

- Start with a complementary filter for roll and pitch.
- Do not claim absolute yaw angle without a magnetometer.
- Yaw control is rate-only using gyro Z rate.
- PID must be disabled unless safety state allows active stabilization.
- Reset PID integrators on disarm, arming transition, failsafe, error lockout, and throttle cutoff where applicable.
- Implement anti-windup.
- Prefer derivative-on-measurement or filtered derivative to avoid derivative kick.
- Enforce PID gain bounds.
- Log PID terms at reduced telemetry rate, not every fast-loop cycle unless explicitly requested.

---

## 12. Motor and mixer rules

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

---

## 13. WiFi and Android control rules

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

### WiFi command architecture

When active WiFi command input is eventually implemented:
- ESP32 should run local AP mode first.
- Do not depend on internet/cloud routing.
- Use HTTP or WebSocket for telemetry/config if useful.
- Use UDP for low-latency command packets only after dry-run phases.
- Every control packet must include magic value, protocol version, sequence number, timestamp or monotonic counter, command mode, roll/pitch/yaw/throttle command, arm/disarm request, dead-man flag, and CRC/checksum.
- Packet loss or stale packets must command neutral input and then failsafe.
- Missing command packets for more than the configured watchdog window must cut motor output.
- App backgrounding, screen lock, network loss, or dead-man release must command immediate neutral/disarm behavior.
- WiFi commands must never write MCPWM directly.
- WiFi commands must become `ControlCommand` requests, then pass through the same validation/safety pipeline as receiver input.

### Android app rules

- Start with connection status, sensor telemetry, safety state, calibration button, PID fields, and emergency stop button.
- Active joystick/throttle UI comes later.
- Throttle UI must be spring-return or dead-man gated.
- Emergency stop must be visible and reachable at all times.
- Show safety state plainly.

---

## 14. Testing rules

Every phase must have measurable pass/fail criteria.

Required test categories:
- Build test: `idf.py build`
- Flash/monitor test when hardware is available.
- Unit tests for PID controller, motor mixer, safety state machine, sensor validation, and attitude estimator.
- Hardware tests for I2C scan, MPU6050 `WHO_AM_I`, BMP280 chip ID, MCPWM pulse widths, receiver input timing, and emergency stop latency.
- Long-run tests before integration: sensor telemetry run, no I2C error check, free heap monitoring, loop timing monitoring.

Never mark a hardware phase complete without the user’s measured output or log.

---

## 15. Documentation and reporting rules

- Update documentation only when it directly reflects implemented behavior.
- Do not generate huge new markdown files unless requested.
- Keep phase notes short and factual.
- When making safety-relevant changes, include a `Safety impact` section.
- When making timing-relevant changes, include loop timing or expected timing impact.
- When making WiFi changes, explain how motor output remains gated.
- Prefer creating/updating `docs/current_phase.txt` over repeatedly reading the full roadmap.

---

## 16. Git, files, and secrets

- Do not commit secrets, WiFi passwords, API keys, tokens, private keys, or local machine paths.
- Do not commit machine-specific `.vscode/settings.json` unless explicitly requested.
- Prefer `.gitignore` entry:

```gitignore
.vscode/settings.json
```

- Use example config files for secrets.
- Do not edit `.gitignore`, `.env`, partition tables, sdkconfig defaults, or build config unless needed for the current phase.
- If modifying build/system config, explain exactly why.

---

## 17. Failure behavior

When uncertain:
- Prefer disarmed output.
- Prefer explicit error.
- Prefer `ERROR_LOCKOUT` over silent recovery for critical IMU/motor-output faults.
- Prefer asking the user for measurements over guessing.
- Prefer stopping at the current phase over speculative implementation.

The machine may be small, but the blades are not philosophical. Safety first, then elegance.
