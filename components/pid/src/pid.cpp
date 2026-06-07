#include "pid.h"
#include "esp_log.h"
#include "math.h"

static const char* TAG = "pid";

static pid_controller_t s_controllers[PID_COUNT] = {};

static void pid_controller_init(pid_controller_t* c, float kp, float ki, float kd)
{
    c->gains.kp = kp;
    c->gains.ki = ki;
    c->gains.kd = kd;
    c->gains.integral_limit = PID_DEFAULT_INTEGRAL_LIMIT;
    c->gains.output_limit = PID_DEFAULT_OUTPUT_LIMIT;
    c->gains.derivative_limit = 0.0f;
    c->state.integral = 0.0f;
    c->state.prev_error = 0.0f;
    c->state.prev_derivative = 0.0f;
    c->state.initialized = false;
}

static float pid_controller_compute(pid_controller_t* c, float error, float measurement, float dt_s)
{
    if (!c->state.initialized) {
        c->state.prev_error = error;
        c->state.initialized = true;
    }

    // Proportional term
    float p_term = c->gains.kp * error;

    // Integral term with anti-windup (clamping)
    c->state.integral += error * dt_s;
    if (c->state.integral > c->gains.integral_limit) c->state.integral = c->gains.integral_limit;
    if (c->state.integral < -c->gains.integral_limit) c->state.integral = -c->gains.integral_limit;
    float i_term = c->gains.ki * c->state.integral;

    // Derivative on measurement (not error) to avoid derivative kick
    // d/dt(measurement) = -d/dt(error) when target is constant
    float derivative = 0.0f;
    if (dt_s > 0.0f) {
        derivative = (measurement - c->state.prev_error) / dt_s;  // prev_error holds prev measurement
    }
    // Low-pass filter derivative (optional, simple)
    derivative = 0.9f * c->state.prev_derivative + 0.1f * derivative;
    c->state.prev_derivative = derivative;
    float d_term = c->gains.kd * derivative;

    // Update prev_error for next iteration (store current measurement)
    c->state.prev_error = measurement;

    // Sum terms
    float output = p_term + i_term + d_term;

    // Output saturation
    if (output > c->gains.output_limit) output = c->gains.output_limit;
    if (output < -c->gains.output_limit) output = -c->gains.output_limit;

    return output;
}

esp_err_t pid_init(void)
{
    pid_controller_init(&s_controllers[PID_ROLL],  PID_DEFAULT_KP_ROLL,  PID_DEFAULT_KI_ROLL,  PID_DEFAULT_KD_ROLL);
    pid_controller_init(&s_controllers[PID_PITCH], PID_DEFAULT_KP_PITCH, PID_DEFAULT_KI_PITCH, PID_DEFAULT_KD_PITCH);
    pid_controller_init(&s_controllers[PID_YAW],   PID_DEFAULT_KP_YAW,   PID_DEFAULT_KI_YAW,   PID_DEFAULT_KD_YAW);

    ESP_LOGI(TAG, "PID initialized: roll(kp=%.2f ki=%.3f kd=%.3f) pitch(kp=%.2f ki=%.3f kd=%.3f) yaw(kp=%.2f ki=%.3f kd=%.3f)",
             PID_DEFAULT_KP_ROLL, PID_DEFAULT_KI_ROLL, PID_DEFAULT_KD_ROLL,
             PID_DEFAULT_KP_PITCH, PID_DEFAULT_KI_PITCH, PID_DEFAULT_KD_PITCH,
             PID_DEFAULT_KP_YAW, PID_DEFAULT_KI_YAW, PID_DEFAULT_KD_YAW);
    return ESP_OK;
}

void pid_reset(void)
{
    for (int i = 0; i < PID_COUNT; i++) {
        s_controllers[i].state.integral = 0.0f;
        s_controllers[i].state.prev_error = 0.0f;
        s_controllers[i].state.prev_derivative = 0.0f;
        s_controllers[i].state.initialized = false;
    }
    ESP_LOGI(TAG, "PID states reset");
}

esp_err_t pid_compute(const estimator_attitude_t* target,
                      const estimator_attitude_t* current,
                      float dt_s,
                      float output[3])
{
    if (!target || !current || !output || dt_s <= 0.0f) return ESP_ERR_INVALID_ARG;

    // Roll PID: error = target_roll - current_roll
    float roll_error = target->roll - current->roll;
    output[PID_ROLL] = pid_controller_compute(&s_controllers[PID_ROLL], roll_error, current->roll, dt_s);

    // Pitch PID: error = target_pitch - current_pitch
    float pitch_error = target->pitch - current->pitch;
    output[PID_PITCH] = pid_controller_compute(&s_controllers[PID_PITCH], pitch_error, current->pitch, dt_s);

    // Yaw PID: target is yaw RATE (rad/s), current is yaw rate from gyro
    // For now, we'll use a simple rate controller on yaw
    // Target yaw rate comes from stick input (0 when centered)
    // Current yaw rate would come from gyro Z
    // Since estimator doesn't track yaw rate, we'll use 0 target and 0 current for now
    // TODO: Pass yaw rate target and gyro Z measurement
    float yaw_error = target->yaw - current->yaw;  // Position error for now
    output[PID_YAW] = pid_controller_compute(&s_controllers[PID_YAW], yaw_error, current->yaw, dt_s);

    return ESP_OK;
}

void pid_set_gains(pid_axis_t axis, const pid_gains_t* gains)
{
    if (axis >= PID_COUNT || !gains) return;
    s_controllers[axis].gains = *gains;
}

void pid_get_gains(pid_axis_t axis, pid_gains_t* gains)
{
    if (axis >= PID_COUNT || !gains) return;
    *gains = s_controllers[axis].gains;
}

void pid_set_integral_limit(pid_axis_t axis, float limit)
{
    if (axis >= PID_COUNT) return;
    s_controllers[axis].gains.integral_limit = fabsf(limit);
}

void pid_set_output_limit(pid_axis_t axis, float limit)
{
    if (axis >= PID_COUNT) return;
    s_controllers[axis].gains.output_limit = fabsf(limit);
}

void pid_get_state(pid_axis_t axis, pid_state_t* state)
{
    if (axis >= PID_COUNT || !state) return;
    *state = s_controllers[axis].state;
}