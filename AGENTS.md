# AGENTS.md — ESP32 Flight Controller Drone

Hermes-facing operating contract for this repository. Safety-critical firmware
that drives rotating brushless motors.

## Source of truth (read in this order)
1. `.clinerules/` — authoritative workspace rules (safety, phases, reporting).
2. `docs/current_phase.txt` — current gate status and immediate task guide.
3. This file — a compact summary of the above for Hermes sessions.

Do **not** read the full canonical roadmap
(`docs/esp32_flight_controller_canonical_merged_plan_wifi_android.txt`) unless the
current phase is unclear, a design decision is needed, safety boundaries are
ambiguous, or the user explicitly asks. Prefer `docs/current_phase.txt`.

If a request conflicts with `.clinerules` or the roadmap, STOP and report the
conflict before editing.

## Safety is law
This project controls rotating motors. Never optimize around safety gates.
- **No propeller testing** until all no-prop gates pass AND the user explicitly
  requests the propeller phase.
- **No direct motor-output writes.** Every motor-capable command must pass through:
  input validation → safety state machine → arming manager → failsafe manager →
  mixer/motor-test gate → MCPWM output clamp. Gate output with
  `safety_is_motor_output_allowed()`; no direct `motor_output_arm()` calls.
- **No WiFi motor control** before roadmap dry-run phases (18–21). Early WiFi is
  telemetry/config/calibration/PID-view only, motors disabled. WiFi commands must
  never write MCPWM directly — they become `ControlCommand` requests through the
  same pipeline as receiver input.
- On boot, all ESC outputs = 1000 µs before any motor-capable state. Disarmed,
  failsafe, and error states always command 1000 µs. Emergency stop forces 1000 µs
  immediately and overrides ramp limiting.
- Never claim a hardware/safety test passed without the user's measured result.

## Phase discipline (23-phase roadmap — .clinerules §6)
1 toolchain · 2 skeleton+boot log · 3 I2C scan · 4 MPU6050 detect/read · 5 MPU6050
cal+NVS · 6 BMP280 · 7 sensor telemetry/validation · 8 attitude estimator · 9
receiver/stub · 10 MCPWM signal (ESC power off) · 11 motor-test SM (no-prop conf) ·
12 safety SM + arming + failsafe · 13 PID unit tests · 14 PID+IMU motors disabled ·
15 mixer unit tests · 16 dry-run, outputs safety-gated · 17 no-prop spin test ·
18 WiFi telemetry/config, motors disabled · 19 Android telemetry UI · 20 Android
WiFi command dry-run, motors disabled · 21 no-prop WiFi command · 22 pre-prop
readiness review · 23 tethered/low hover.

- **No phase skipping.** Never implement phase N+1 until phase N's hardware result
  is recorded in `docs/current_phase.txt`.
- If asked for a later phase before prerequisites exist: STOP, list missing gates,
  ask for confirmation.
- **Current status: always read `docs/current_phase.txt`. Do not infer the active
  phase from this file.**
- **Do not assume Prompt N equals roadmap phase N. Check `docs/current_phase.txt`
  and `.clinerules` before mapping prompt numbers to roadmap phases.**

## Build (ESP-IDF v6.0.1, native Windows cmd — NOT git-bash)
```bat
cmd /c "C:\Akmal\Project-tools\.espressif\v6.0.1\esp-idf\export.bat && idf.py -C C:\Akmal\esp32-flight-controller-drone build"
```
ESP-IDF v6+ does not support MSYS/Mingw. Use `cmd.exe`. Project name is from the
root `CMakeLists.txt` `project()` line (`esp32_flight_controller_drone`); artifacts
are `build/esp32_flight_controller_drone.{elf,bin,map}`. `idf.py monitor` runs in
cmd.exe only; in git-bash use PuTTY/CoolTerm at 115200 baud.

## Mandatory report format (every change set, keep under ~80 lines)
1. Files changed (by path)
2. Commands run
3. Build/test result (build gate PASS/FAIL + first error/cause; hardware gate PASS/FAIL/PENDING)
4. Assumptions
5. Safety impact (how motor output stays gated)
6. Remaining risks
7. Next recommended gate

Update `docs/current_phase.txt` after every change set. One logical change per
commit with a descriptive message — never commit, branch, tag, or push unless the
user explicitly requests it.

## Scope discipline
No broad cleanups, refactors, file moves, dependency changes, or architecture
changes unless explicitly requested. Prefer disarmed output, explicit errors, and
stopping at the current phase over speculative work.
