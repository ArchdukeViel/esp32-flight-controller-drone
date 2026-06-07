#include "motor_output.h"
#include "board_config.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "motor_output";

static motor_output_config_t s_config = {
    .protocol = MOTOR_OUTPUT_PWM,
    .frequency_hz = MOTOR_OUTPUT_DEFAULT_FREQ_HZ,
    .min_pulse_us = MOTOR_OUTPUT_DEFAULT_MIN_US,
    .max_pulse_us = MOTOR_OUTPUT_DEFAULT_MAX_US,
    .idle_pulse_us = MOTOR_OUTPUT_DEFAULT_IDLE_US,
    .inverted = false
};

static motor_output_state_t s_state = {};
static bool s_initialized = false;
static bool s_armed = false;

// MCPWM handles
static mcpwm_timer_handle_t s_timer = NULL;
static mcpwm_oper_handle_t s_operators[4] = {NULL};
static mcpwm_cmpr_handle_t s_comparators[4] = {NULL};
static mcpwm_gen_handle_t s_generators[4] = {NULL};

// Motor GPIO pins (mapped from board_config ESC_Mx_GPIO to mixer index)
// mixer[0] = FR  -> ESC_M2_GPIO (19)
// mixer[1] = FL  -> ESC_M1_GPIO (18)
// mixer[2] = RR  -> ESC_M3_GPIO (23)
// mixer[3] = RL  -> ESC_M4_GPIO (25)
static const int s_motor_gpios[4] = {
    ESC_M2_GPIO,
    ESC_M1_GPIO,
    ESC_M3_GPIO,
    ESC_M4_GPIO
};

static inline uint32_t pulse_us_to_ticks(uint16_t pulse_us, uint32_t freq_hz)
{
    // MCPWM timer counts at resolution_hz (default 10 MHz = 10,000,000 Hz)
    // Period = resolution_hz / freq_hz ticks
    // Duty cycle = pulse_us * resolution_hz / 1,000,000 ticks
    const uint32_t resolution_hz = 10000000; // 10 MHz
    return (uint32_t)((uint64_t)pulse_us * resolution_hz / 1000000);
}

static esp_err_t setup_mcpwm_timer(void)
{
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, // 10 MHz
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 10000000 / s_config.frequency_hz,
        .intr_priority = 0,
        .flags = {
            .update_period_on_empty = false,
            .update_period_on_sync = false,
            .allow_pd = false,
        },
    };
    return mcpwm_new_timer(&timer_config, &s_timer);
}

static esp_err_t setup_mcpwm_operators(void)
{
    mcpwm_operator_config_t oper_config = {
        .group_id = 0,
        .intr_priority = 0,
        .flags = {
            .update_gen_action_on_tez = true,
            .update_gen_action_on_tep = false,
            .update_gen_action_on_sync = false,
            .update_dead_time_on_tez = false,
            .update_dead_time_on_tep = false,
            .update_dead_time_on_sync = false,
        },
    };
    for (int i = 0; i < 4; i++) {
        esp_err_t ret = mcpwm_new_operator(&oper_config, &s_operators[i]);
        if (ret != ESP_OK) return ret;
        ret = mcpwm_operator_connect_timer(s_operators[i], s_timer);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static esp_err_t setup_mcpwm_comparators(void)
{
    mcpwm_comparator_config_t cmp_config = {
        .intr_priority = 0,
        .flags = {
            .update_cmp_on_tez = true,
            .update_cmp_on_tep = false,
            .update_cmp_on_sync = false,
        },
    };
    for (int i = 0; i < 4; i++) {
        esp_err_t ret = mcpwm_new_comparator(s_operators[i], &cmp_config, &s_comparators[i]);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static esp_err_t setup_mcpwm_generators(void)
{
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = -1, // Set per motor below
        .flags = {
            .invert_pwm = false,
        },
    };
    for (int i = 0; i < 4; i++) {
        gen_config.gen_gpio_num = s_motor_gpios[i];
        esp_err_t ret = mcpwm_new_generator(s_operators[i], &gen_config, &s_generators[i]);
        if (ret != ESP_OK) return ret;

        // Set actions: high on timer empty (TEZ), low on compare match (TCMP)
        ret = mcpwm_generator_set_action_on_timer_event(s_generators[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        if (ret != ESP_OK) return ret;

        ret = mcpwm_generator_set_action_on_compare_event(s_generators[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_comparators[i], MCPWM_GEN_ACTION_LOW));
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static void set_all_pulse_us(uint16_t pulse_us)
{
    uint32_t ticks = pulse_us_to_ticks(pulse_us, s_config.frequency_hz);
    for (int i = 0; i < 4; i++) {
        if (s_comparators[i]) {
            mcpwm_comparator_set_compare_value(s_comparators[i], ticks);
        }
        s_state.pulse_us[i] = pulse_us;
    }
}

esp_err_t motor_output_init(const motor_output_config_t* config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    if (config) {
        s_config = *config;
    }

    // Validate GPIOs
    for (int i = 0; i < 4; i++) {
        if (s_motor_gpios[i] < 0) {
            ESP_LOGE(TAG, "Motor %d GPIO not defined in board_config", i);
            return ESP_ERR_INVALID_ARG;
        }
    }

    ESP_LOGI(TAG, "Initializing MCPWM motor output");
    ESP_LOGI(TAG, "Protocol: %d, Freq: %lu Hz, Min: %u us, Max: %u us, Idle: %u us",
             s_config.protocol, s_config.frequency_hz,
             s_config.min_pulse_us, s_config.max_pulse_us, s_config.idle_pulse_us);

    // Initialize MCPWM
    esp_err_t ret = setup_mcpwm_timer();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Timer create failed: %s", esp_err_to_name(ret)); return ret; }

    ret = mcpwm_timer_enable(s_timer);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Timer enable failed: %s", esp_err_to_name(ret)); return ret; }

    ret = setup_mcpwm_operators();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Operator init failed: %s", esp_err_to_name(ret)); return ret; }

    ret = setup_mcpwm_comparators();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Comparator init failed: %s", esp_err_to_name(ret)); return ret; }

    ret = setup_mcpwm_generators();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Generator init failed: %s", esp_err_to_name(ret)); return ret; }

    // Start timer
    ret = mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Timer start failed: %s", esp_err_to_name(ret)); return ret; }

    // Set initial idle pulse
    set_all_pulse_us(s_config.idle_pulse_us);

    s_initialized = true;
    s_armed = false;
    s_state.armed = false;

    ESP_LOGI(TAG, "MCPWM motor output initialized (disarmed)");
    return ESP_OK;
}

void motor_output_deinit(void)
{
    if (!s_initialized) return;

    motor_output_disarm();

    if (s_timer) {
        mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_STOP_EMPTY);
        mcpwm_timer_disable(s_timer);
        mcpwm_del_timer(s_timer);
        s_timer = NULL;
    }
    for (int i = 0; i < 4; i++) {
        if (s_generators[i]) { mcpwm_del_generator(s_generators[i]); s_generators[i] = NULL; }
        if (s_comparators[i]) { mcpwm_del_comparator(s_comparators[i]); s_comparators[i] = NULL; }
        if (s_operators[i]) { mcpwm_del_operator(s_operators[i]); s_operators[i] = NULL; }
    }
    s_initialized = false;
    ESP_LOGI(TAG, "MCPWM motor output deinitialized");
}

esp_err_t motor_output_set(const motor_mixer_output_t* output)
{
    if (!s_initialized || !output) return ESP_ERR_INVALID_STATE;

    if (!s_armed) {
        // Disarmed: keep at idle
        return ESP_OK;
    }

    for (int i = 0; i < 4; i++) {
        float norm = output->motor[i];
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;

        uint16_t pulse_us = s_config.min_pulse_us +
            (uint16_t)(norm * (s_config.max_pulse_us - s_config.min_pulse_us));

        uint32_t ticks = pulse_us_to_ticks(pulse_us, s_config.frequency_hz);
        if (s_comparators[i]) {
            mcpwm_comparator_set_compare_value(s_comparators[i], ticks);
        }
        s_state.pulse_us[i] = pulse_us;
    }
    return ESP_OK;
}

esp_err_t motor_output_arm(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (s_armed) return ESP_OK;

    // Set to idle before arming
    set_all_pulse_us(s_config.idle_pulse_us);
    s_armed = true;
    s_state.armed = true;
    ESP_LOGW(TAG, "MOTOR OUTPUT ARMED - PROPELLERS MAY SPIN!");
    return ESP_OK;
}

esp_err_t motor_output_disarm(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!s_armed) return ESP_OK;

    set_all_pulse_us(s_config.idle_pulse_us);
    s_armed = false;
    s_state.armed = false;
    ESP_LOGW(TAG, "MOTOR OUTPUT DISARMED");
    return ESP_OK;
}

bool motor_output_is_initialized(void)
{
    return s_initialized;
}

bool motor_output_is_armed(void)
{
    return s_armed;
}

void motor_output_get_state(motor_output_state_t* state)
{
    if (state) {
        *state = s_state;
        state->armed = s_armed;
    }
}

esp_err_t motor_output_set_config(const motor_output_config_t* config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    if (s_armed) return ESP_ERR_INVALID_STATE; // Can't change config while armed

    // Stop timer to reconfigure
    if (s_timer) {
        mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_STOP_EMPTY);
    }

    s_config = *config;

    // Reconfigure timer period
    if (s_timer) {
        mcpwm_timer_set_period(s_timer, 10000000 / s_config.frequency_hz);
        mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP);
    }

    // Update all pulses to new idle
    set_all_pulse_us(s_config.idle_pulse_us);

    ESP_LOGI(TAG, "Config updated: Freq=%lu Hz, Min=%u us, Max=%u us",
             s_config.frequency_hz, s_config.min_pulse_us, s_config.max_pulse_us);
    return ESP_OK;
}

void motor_output_get_config(motor_output_config_t* config)
{
    if (config) *config = s_config;
}

void motor_output_emergency_stop(void)
{
    if (!s_initialized) return;

    // Immediately set all to minimum (idle) without ramping
    for (int i = 0; i < 4; i++) {
        if (s_comparators[i]) {
            uint32_t ticks = pulse_us_to_ticks(s_config.min_pulse_us, s_config.frequency_hz);
            mcpwm_comparator_set_compare_value(s_comparators[i], ticks);
        }
        s_state.pulse_us[i] = s_config.min_pulse_us;
    }
    s_armed = false;
    s_state.armed = false;
    ESP_LOGE(TAG, "EMERGENCY STOP - ALL MOTORS SET TO IDLE");
}