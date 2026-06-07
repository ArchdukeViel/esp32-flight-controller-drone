#include "safety.h"
#include "motor_output.h"
#include "pid.h"
#include "estimator.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "safety";

// State machine state
static safety_state_t s_state = SAFETY_STATE_BOOT;
static uint32_t s_state_entry_time_ms = 0;
static uint32_t s_last_transition_time_ms = 0;
static const char* s_last_failure_reason = NULL;
static bool s_arm_requested = false;
static bool s_disarm_requested = false;

// Health tracking
static uint32_t s_last_estimator_update_ms = 0;

// Timeouts and limits
#define PRE_ARM_CHECK_TIMEOUT_MS  5000
#define ESTIMATOR_STALE_TIMEOUT_MS 200

static const char* state_to_string(safety_state_t state)
{
    switch (state) {
        case SAFETY_STATE_BOOT: return "BOOT";
        case SAFETY_STATE_DISARMED: return "DISARMED";
        case SAFETY_STATE_PRE_ARM_CHECK: return "PRE_ARM_CHECK";
        case SAFETY_STATE_ARMED: return "ARMED";
        case SAFETY_STATE_FAILSAFE: return "FAILSAFE";
        case SAFETY_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

static void transition_to(safety_state_t new_state, const char* reason)
{
    safety_state_t old_state = s_state;
    s_state = new_state;
    s_state_entry_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_last_transition_time_ms = s_state_entry_time_ms;

    ESP_LOGI(TAG, "State transition: %s -> %s (%s)",
             state_to_string(old_state), state_to_string(new_state), reason);

    // State entry actions
    switch (new_state) {
        case SAFETY_STATE_BOOT:
            // Force idle output immediately
            motor_output_disarm();
            break;

        case SAFETY_STATE_DISARMED:
            motor_output_disarm();
            pid_reset();
            s_arm_requested = false;
            break;

        case SAFETY_STATE_PRE_ARM_CHECK:
            // Still disarmed during checks
            motor_output_disarm();
            break;

        case SAFETY_STATE_ARMED:
            motor_output_arm();
            pid_reset();
            s_arm_requested = false;
            break;

        case SAFETY_STATE_FAILSAFE:
            motor_output_emergency_stop();
            pid_reset();
            s_last_failure_reason = reason;
            ESP_LOGE(TAG, "FAILSAFE: %s", reason);
            break;

        case SAFETY_STATE_ERROR:
            motor_output_emergency_stop();
            pid_reset();
            s_last_failure_reason = reason;
            ESP_LOGE(TAG, "ERROR LOCKOUT: %s", reason);
            break;
    }
}

static bool check_estimator_health(void)
{
    estimator_state_t est_state = {};
    estimator_get_state(&est_state);

    if (!est_state.initialized) {
        s_last_failure_reason = "Estimator not initialized";
        return false;
    }

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t est_age_ms = now_ms - (est_state.timestamp_us / 1000);

    if (est_age_ms > ESTIMATOR_STALE_TIMEOUT_MS) {
        s_last_failure_reason = "Estimator data stale";
        return false;
    }

    s_last_estimator_update_ms = now_ms;
    return true;
}

static bool check_sensor_health(void)
{
    mpu6050_error_count_t mpu_errs = {};
    mpu6050_get_error_counts(&mpu_errs);

    // Check init errors (persistent failure)
    if (mpu_errs.init_errors > 0) {
        s_last_failure_reason = "MPU6050 init errors";
        return false;
    }

    // Note: Runtime IMU read failures are caught via estimator staleness check
    // (estimator_update skipped on read failure -> timestamp stale -> failsafe)

    return true;
}

static bool check_motor_output_ready(void)
{
    if (!motor_output_is_initialized()) {
        s_last_failure_reason = "Motor output not initialized";
        return false;
    }
    return true;
}

static bool run_pre_arm_checks(void)
{
    // Check 1: Motor output ready
    if (!check_motor_output_ready()) {
        s_last_failure_reason = "Motor output not ready";
        return false;
    }

    // Check 2: Estimator initialized and fresh
    if (!check_estimator_health()) {
        return false;
    }

    // Check 3: Sensors healthy
    if (!check_sensor_health()) {
        return false;
    }

    // Check 4: Not already armed
    if (motor_output_is_armed()) {
        s_last_failure_reason = "Already armed";
        return false;
    }

    // Check 5: Throttle at idle (currently no external throttle source exists)
    // Note: When a throttle command source is added (receiver/WiFi/etc),
    // this check must validate that throttle command is at idle before arming.
    // Current system has target_throttle hardcoded to 0.0f in app_main.

    return true;
}

static void handle_boot_state(void)
{
    // Boot state: initialize and transition to DISARMED
    // Check critical systems
    if (!check_sensor_health()) {
        transition_to(SAFETY_STATE_ERROR, "Critical sensor failure at boot");
        return;
    }

    // Successful boot - enter DISARMED
    transition_to(SAFETY_STATE_DISARMED, "Boot complete");
}

static void handle_disarmed_state(void)
{
    // Monitor for arm request
    if (s_arm_requested) {
        transition_to(SAFETY_STATE_PRE_ARM_CHECK, "Arm requested");
        return;
    }

    // Monitor for critical failures
    if (!check_sensor_health()) {
        transition_to(SAFETY_STATE_ERROR, "Critical sensor failure");
        return;
    }

    // Ensure outputs stay at idle
    if (motor_output_is_armed()) {
        motor_output_disarm();
    }
}

static void handle_pre_arm_check_state(void)
{
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t time_in_state_ms = now_ms - s_state_entry_time_ms;

    // Check for disarm request
    if (s_disarm_requested) {
        transition_to(SAFETY_STATE_DISARMED, "Disarm requested during pre-arm");
        return;
    }

    // Timeout check
    if (time_in_state_ms > PRE_ARM_CHECK_TIMEOUT_MS) {
        transition_to(SAFETY_STATE_DISARMED, "Pre-arm check timeout");
        return;
    }

    // Run all checks
    if (run_pre_arm_checks()) {
        transition_to(SAFETY_STATE_ARMED, "Pre-arm checks passed");
    } else {
        transition_to(SAFETY_STATE_DISARMED, s_last_failure_reason);
    }
}

static void handle_armed_state(void)
{
    // Check for disarm request
    if (s_disarm_requested) {
        transition_to(SAFETY_STATE_DISARMED, "Disarm requested");
        return;
    }

    // Continuous health monitoring
    if (!check_estimator_health()) {
        transition_to(SAFETY_STATE_FAILSAFE, s_last_failure_reason);
        return;
    }

    if (!check_sensor_health()) {
        transition_to(SAFETY_STATE_FAILSAFE, s_last_failure_reason);
        return;
    }

    // Verify motor output is still armed
    if (!motor_output_is_armed()) {
        transition_to(SAFETY_STATE_FAILSAFE, "Motor output unexpectedly disarmed");
        return;
    }
}

static void handle_failsafe_state(void)
{
    // Ensure outputs are safe
    if (motor_output_is_armed()) {
        motor_output_emergency_stop();
    }

    // Manual recovery only - wait for disarm request
    if (s_disarm_requested) {
        transition_to(SAFETY_STATE_DISARMED, "Manual recovery from failsafe");
        return;
    }

    // Escalate to ERROR if sensors critically failed
    if (!check_sensor_health()) {
        transition_to(SAFETY_STATE_ERROR, "Persistent sensor failure");
        return;
    }
}

static void handle_error_state(void)
{
    // ERROR is terminal - outputs locked at idle
    // No automatic recovery, no transitions
    // Requires reboot or explicit external reset (not implemented)
    
    // Ensure outputs stay safe
    if (motor_output_is_armed()) {
        motor_output_emergency_stop();
    }
}

esp_err_t safety_init(void)
{
    s_state = SAFETY_STATE_BOOT;
    s_state_entry_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_last_transition_time_ms = s_state_entry_time_ms;
    s_last_failure_reason = NULL;
    s_arm_requested = false;
    s_disarm_requested = false;
    s_last_estimator_update_ms = 0;

    // Force safe outputs immediately
    motor_output_disarm();

    ESP_LOGI(TAG, "Safety state machine initialized in BOOT state");
    return ESP_OK;
}

esp_err_t safety_update(void)
{
    // State machine execution
    switch (s_state) {
        case SAFETY_STATE_BOOT:
            handle_boot_state();
            break;
        case SAFETY_STATE_DISARMED:
            handle_disarmed_state();
            break;
        case SAFETY_STATE_PRE_ARM_CHECK:
            handle_pre_arm_check_state();
            break;
        case SAFETY_STATE_ARMED:
            handle_armed_state();
            break;
        case SAFETY_STATE_FAILSAFE:
            handle_failsafe_state();
            break;
        case SAFETY_STATE_ERROR:
            handle_error_state();
            break;
        default:
            transition_to(SAFETY_STATE_ERROR, "Invalid state");
            break;
    }

    // Clear disarm request after processing
    s_disarm_requested = false;

    return ESP_OK;
}

esp_err_t safety_request_arm(void)
{
    if (s_state != SAFETY_STATE_DISARMED) {
        ESP_LOGW(TAG, "Arm request rejected: not in DISARMED state (current: %s)",
                 state_to_string(s_state));
        return ESP_ERR_INVALID_STATE;
    }

    s_arm_requested = true;
    ESP_LOGI(TAG, "Arm request accepted");
    return ESP_OK;
}

esp_err_t safety_request_disarm(void)
{
    if (s_state == SAFETY_STATE_ERROR) {
        ESP_LOGW(TAG, "Disarm request rejected: in ERROR state (terminal)");
        return ESP_ERR_INVALID_STATE;
    }

    s_disarm_requested = true;
    ESP_LOGI(TAG, "Disarm request accepted");
    return ESP_OK;
}

void safety_emergency_stop(void)
{
    ESP_LOGE(TAG, "EMERGENCY STOP triggered");
    transition_to(SAFETY_STATE_FAILSAFE, "Emergency stop");
}

safety_state_t safety_get_state(void)
{
    return s_state;
}

void safety_get_status(safety_status_t* status)
{
    if (!status) return;

    status->state = s_state;
    status->state_entry_time_ms = s_state_entry_time_ms;
    status->last_transition_time_ms = s_last_transition_time_ms;
    status->can_arm = (s_state == SAFETY_STATE_DISARMED);
    status->last_failure_reason = s_last_failure_reason;
}

bool safety_is_armed(void)
{
    return s_state == SAFETY_STATE_ARMED;
}

bool safety_is_motor_output_allowed(void)
{
    return s_state == SAFETY_STATE_ARMED;
}
