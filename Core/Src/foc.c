#include "foc.h"

#include <math.h>
#include <sys/stat.h>

#define FOC_TWO_PI                (2.0f * PI)
#define FOC_ENCODER_CPR           4096
#define FOC_DIRECTION_TEST_ANGLE  (0.5f * PI)
#define FOC_ALIGNMENT_SAMPLE_COUNT 8U
#define FOC_ALIGNMENT_SAMPLE_DELAY_MS 1U
#define FOC_ALIGNMENT_VOLTAGE_PU   0.15f
#define FOC_ALIGNMENT_RAMP_UP_MS   200U
#define FOC_ALIGNMENT_ANGLE_RAMP_MS 400U
#define FOC_ALIGNMENT_SETTLE_MS    100U
#define FOC_ALIGNMENT_RAMP_DOWN_MS 100U
#define FOC_ALIGNMENT_RAMP_STEP_MS 2U
#define FOC_DIRECTION_MIN_RATIO   0.25f
#define FOC_DIRECTION_MAX_RATIO   1.75f
#define FOC_ADC_MAX_VALUE         4095U
#define FOC_ADC_CALIBRATION_DISCARD_COUNT 32U
#define FOC_ADC_CALIBRATION_SAMPLE_COUNT 1024U
#define FOC_ADC_CALIBRATION_TIMEOUT_MS 200U
#define DRV8323_WAKE_DELAY_MS     1U
#define DRV8323_RESET_DELAY_MS    1U

static float32_t max3(float32_t a, float32_t b, float32_t c) {
    if (a>b) {
        if (a>c) {
            return a;
        }
    } else {
        if (b>c) {
            return b;
        }
    }
    return c;
}

static float32_t min3(float32_t a, float32_t b, float32_t c) {
    if (a>b) {
        if (b>c) {
            return c;
        }
        return b;
    }
    if (a>c) {
        return c;
    }
    return a;
}

static HAL_StatusTypeDef limit_dq_voltage(float32_t *U_q, float32_t *U_d) {
    const float32_t limitation = 0.577350269f * FOC_VOLTAGE_MARGIN;

    if (U_q == NULL || U_d == NULL || !isfinite(*U_q) || !isfinite(*U_d)) {
        return HAL_ERROR;
    }

    const float32_t magnitude_squared =
        (*U_q * *U_q) + (*U_d * *U_d);

    if (magnitude_squared <= limitation * limitation) {
        return HAL_OK;
    }

    float32_t magnitude;
    if (arm_sqrt_f32(magnitude_squared, &magnitude) != ARM_MATH_SUCCESS || magnitude<=0.0f) {
        *U_q = 0.0f;
        *U_d = 0.0f;
        return HAL_ERROR;
    }

    const float32_t scale = limitation / magnitude;
    *U_q *= scale;
    *U_d *= scale;
    return HAL_OK;
}

static float32_t clamp_duty(float32_t duty) {
    if (duty < 0.0f) {
        return 0.0f;
    }
    if (duty > 1.0f) {
        return 1.0f;
    }
    return duty;
}

static HAL_StatusTypeDef pwm_update_duty(
    TIM_HandleTypeDef *htim,
    float32_t duty_a,
    float32_t duty_b,
    float32_t duty_c) {
    if (htim == NULL || htim->Instance == NULL ||
        !isfinite(duty_a) || !isfinite(duty_b) || !isfinite(duty_c)) {
        return HAL_ERROR;
    }

    const uint32_t tim_period = htim->Instance->ARR;
    const uint32_t ccr_ch1 = (uint32_t)lroundf(clamp_duty(duty_a) * (float32_t)tim_period);
    const uint32_t ccr_ch2 = (uint32_t)lroundf(clamp_duty(duty_b) * (float32_t)tim_period);
    const uint32_t ccr_ch3 = (uint32_t)lroundf(clamp_duty(duty_c) * (float32_t)tim_period);

    __HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, ccr_ch1);
    __HAL_TIM_SetCompare(htim, TIM_CHANNEL_2, ccr_ch2);
    __HAL_TIM_SetCompare(htim, TIM_CHANNEL_3, ccr_ch3);
    return HAL_OK;
}

static uint8_t driver_fault_active(const FOC_HandleTypeDef *self) {
    return HAL_GPIO_ReadPin(self->driver_fault_port, self->driver_fault_pin) == GPIO_PIN_RESET;
}

static void pwm_stop_all(FOC_HandleTypeDef *self) {
    if (self == NULL || self->htim == NULL) {
        return;
    }

    __HAL_TIM_MOE_DISABLE(self->htim);
    (void)HAL_TIMEx_PWMN_Stop(self->htim, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Stop(self->htim, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(self->htim, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(self->htim, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(self->htim, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(self->htim, TIM_CHANNEL_1);
    self->pwm_started = 0U;
}

static HAL_StatusTypeDef pwm_start_all(FOC_HandleTypeDef *self) {
    HAL_StatusTypeDef status;

    if (self == NULL || self->htim == NULL || self->htim->Instance == NULL) {
        return HAL_ERROR;
    }

    status = pwm_update_duty(self->htim, 0.5f, 0.5f, 0.5f);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_TIM_PWM_Start(self->htim, TIM_CHANNEL_1);
    if (status != HAL_OK) {
        goto fail;
    }
    status = HAL_TIM_PWM_Start(self->htim, TIM_CHANNEL_2);
    if (status != HAL_OK) {
        goto fail;
    }
    status = HAL_TIM_PWM_Start(self->htim, TIM_CHANNEL_3);
    if (status != HAL_OK) {
        goto fail;
    }
    status = HAL_TIMEx_PWMN_Start(self->htim, TIM_CHANNEL_1);
    if (status != HAL_OK) {
        goto fail;
    }
    status = HAL_TIMEx_PWMN_Start(self->htim, TIM_CHANNEL_2);
    if (status != HAL_OK) {
        goto fail;
    }
    status = HAL_TIMEx_PWMN_Start(self->htim, TIM_CHANNEL_3);
    if (status != HAL_OK) {
        goto fail;
    }

    self->pwm_started = 1U;
    return HAL_OK;

fail:
    pwm_stop_all(self);
    return status;
}

static void driver_shutdown(FOC_HandleTypeDef *self, FOC_StateTypeDef next_state) {
    if (self == NULL) {
        return;
    }

    if (self->htim != NULL) {
        __HAL_TIM_MOE_DISABLE(self->htim);
    }
    if (self->driver_enable_port != NULL && self->driver_enable_pin != 0U) {
        HAL_GPIO_WritePin(self->driver_enable_port, self->driver_enable_pin, GPIO_PIN_RESET);
    }
    pwm_stop_all(self);
    self->Target_Ud = 0.0f;
    self->Target_Uq = 0.0f;
    self->current_u = 0.0f;
    self->current_v = 0.0f;
    self->current_w = 0.0f;
    self->state = next_state;
}

static HAL_StatusTypeDef driver_wake(FOC_HandleTypeDef *self) {
    if (self == NULL || self->driver_enable_port == NULL ||
        self->driver_enable_pin == 0U) {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(self->driver_enable_port, self->driver_enable_pin, GPIO_PIN_SET);
    HAL_Delay(DRV8323_WAKE_DELAY_MS);

    if (driver_fault_active(self)) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef driver_enable(FOC_HandleTypeDef *self) {
    HAL_StatusTypeDef status = driver_wake(self);
    if (status != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return status;
    }

    status = pwm_start_all(self);
    if (status != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return status;
    }

    return HAL_OK;
}

static float32_t wrap_angle_2pi(float32_t angle) {
    angle = fmodf(angle, FOC_TWO_PI);
    if (angle < 0.0f) {
        angle += FOC_TWO_PI;
    }
    return angle;
}

static HAL_StatusTypeDef read_encoder_average(
    FOC_HandleTypeDef *self,
    float32_t *average_raw_angle) {
    if (self == NULL || self->as5600_handle == NULL || average_raw_angle == NULL) {
        return HAL_ERROR;
    }

    uint16_t reference_raw_angle;
    if (as5600_read(self->as5600_handle, &reference_raw_angle) != HAL_OK) {
        return HAL_ERROR;
    }

    int32_t sample_sum = (int32_t)reference_raw_angle;
    for (uint32_t sample = 1U; sample < FOC_ALIGNMENT_SAMPLE_COUNT; ++sample) {
        HAL_Delay(FOC_ALIGNMENT_SAMPLE_DELAY_MS);

        uint16_t raw_angle;
        if (as5600_read(self->as5600_handle, &raw_angle) != HAL_OK) {
            return HAL_ERROR;
        }

        int32_t sample_delta = (int32_t)raw_angle - (int32_t)reference_raw_angle;
        if (sample_delta > FOC_ENCODER_CPR / 2) {
            sample_delta -= FOC_ENCODER_CPR;
        } else if (sample_delta < -(FOC_ENCODER_CPR / 2)) {
            sample_delta += FOC_ENCODER_CPR;
        }
        sample_sum += (int32_t)reference_raw_angle + sample_delta;
    }

    *average_raw_angle =
        (float32_t)sample_sum / (float32_t)FOC_ALIGNMENT_SAMPLE_COUNT;
    return HAL_OK;
}

static HAL_StatusTypeDef apply_alignment_vector(
    FOC_HandleTypeDef *self,
    float32_t voltage,
    float32_t electrical_angle) {
    float32_t duty_a;
    float32_t duty_b;
    float32_t duty_c;
    const float32_t U_alpha = voltage * arm_cos_f32(electrical_angle);
    const float32_t U_beta = voltage * arm_sin_f32(electrical_angle);

    svpwm(U_alpha, U_beta, &duty_a, &duty_b, &duty_c);
    return pwm_update_duty(self->htim, duty_a, duty_b, duty_c);
}

static HAL_StatusTypeDef transition_alignment_vector(
    FOC_HandleTypeDef *self,
    float32_t start_voltage,
    float32_t end_voltage,
    float32_t start_angle,
    float32_t end_angle,
    uint32_t duration_ms) {
    if (self == NULL || duration_ms == 0U) {
        return HAL_ERROR;
    }

    uint32_t step_count = duration_ms / FOC_ALIGNMENT_RAMP_STEP_MS;
    if (step_count == 0U) {
        step_count = 1U;
    }

    if (driver_fault_active(self)) {
        return HAL_ERROR;
    }

    for (uint32_t step = 1U; step <= step_count; ++step) {
        const float32_t progress = (float32_t)step / (float32_t)step_count;
        const float32_t smooth_progress =
            progress * progress * (3.0f - 2.0f * progress);
        const float32_t voltage =
            start_voltage + (end_voltage - start_voltage) * smooth_progress;
        const float32_t angle =
            start_angle + (end_angle - start_angle) * smooth_progress;

        if (apply_alignment_vector(self, voltage, angle) != HAL_OK) {
            return HAL_ERROR;
        }
        HAL_Delay(FOC_ALIGNMENT_RAMP_STEP_MS);
        if (driver_fault_active(self)) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef adc_enable(FOC_HandleTypeDef *self) {
    if (
        HAL_ADCEx_InjectedStart(self->hadc2) != HAL_OK ||
        HAL_ADCEx_InjectedStart(self->hadc3) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(self->hadc1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(self->htim, TIM_CHANNEL_4) != HAL_OK) {
        return HAL_ERROR;
    }
    /* Starting a channel on an advanced timer enables MOE globally. Keep the
     * three phase outputs disabled until pwm_start_all() is called. */
    __HAL_TIM_MOE_DISABLE(self->htim);
    return HAL_OK;
}

static HAL_StatusTypeDef rotor_align(
    FOC_HandleTypeDef *self,
    float32_t alignment_voltage_pu,
    uint32_t alignment_time_ms) {
    float32_t initial_raw_angle;
    float32_t probe_raw_angle;
    float32_t aligned_raw_angle;

    self->state = FOC_STATE_ALIGNING;
    if (transition_alignment_vector(
            self,
            0.0f,
            alignment_voltage_pu,
            0.0f,
            0.0f,
            FOC_ALIGNMENT_RAMP_UP_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(alignment_time_ms);
    if (driver_fault_active(self)) {
        return HAL_ERROR;
    }
    if (read_encoder_average(self, &initial_raw_angle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (transition_alignment_vector(
            self,
            alignment_voltage_pu,
            alignment_voltage_pu,
            0.0f,
            FOC_DIRECTION_TEST_ANGLE,
            FOC_ALIGNMENT_ANGLE_RAMP_MS) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(alignment_time_ms);
    if (driver_fault_active(self) ||
        read_encoder_average(self, &probe_raw_angle) != HAL_OK) {
        return HAL_ERROR;
    }

    float32_t encoder_delta = probe_raw_angle - initial_raw_angle;
    if (encoder_delta > (float32_t)FOC_ENCODER_CPR / 2.0f) {
        encoder_delta -= (float32_t)FOC_ENCODER_CPR;
    } else if (encoder_delta < -(float32_t)FOC_ENCODER_CPR / 2.0f) {
        encoder_delta += (float32_t)FOC_ENCODER_CPR;
    }

    const float32_t expected_encoder_delta =
        (float32_t)FOC_ENCODER_CPR * FOC_DIRECTION_TEST_ANGLE /
        (FOC_TWO_PI * (float32_t)self->pole_pairs);
    const float32_t absolute_encoder_delta = fabsf(encoder_delta);
    if (absolute_encoder_delta < expected_encoder_delta * FOC_DIRECTION_MIN_RATIO ||
        absolute_encoder_delta > expected_encoder_delta * FOC_DIRECTION_MAX_RATIO) {
        return HAL_ERROR;
    }
    self->rotor_direction = encoder_delta > 0
        ? FOC_ROTOR_DIRECTION_SAME
        : FOC_ROTOR_DIRECTION_OPPOSITE;

    if (transition_alignment_vector(
            self,
            alignment_voltage_pu,
            alignment_voltage_pu,
            FOC_DIRECTION_TEST_ANGLE,
            0.0f,
            FOC_ALIGNMENT_ANGLE_RAMP_MS) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(alignment_time_ms);
    if (driver_fault_active(self) ||
        read_encoder_average(self, &aligned_raw_angle) != HAL_OK) {
        return HAL_ERROR;
    }

    const float32_t mechanical_angle =
        aligned_raw_angle * FOC_TWO_PI / (float32_t)FOC_ENCODER_CPR;
    const float32_t direction_sign =
        self->rotor_direction == FOC_ROTOR_DIRECTION_SAME ? 1.0f : -1.0f;
    self->rotor_offset = wrap_angle_2pi(
        -(direction_sign * (float32_t)self->pole_pairs * mechanical_angle));
    self->Target_Ud = 0.0f;
    self->Target_Uq = 0.0f;

    if (transition_alignment_vector(
            self,
            alignment_voltage_pu,
            0.0f,
            0.0f,
            0.0f,
            FOC_ALIGNMENT_RAMP_DOWN_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    return pwm_update_duty(self->htim, 0.5f, 0.5f, 0.5f);
}

void inv_clarke(float32_t U_alpha, float32_t U_beta, float32_t *U_a, float32_t *U_b,
                float32_t *U_c) {
    *U_a = U_alpha;
    *U_b = -0.5f * U_alpha + 0.866025404f * U_beta;
    *U_c = -0.5f * U_alpha - 0.866025404f * U_beta;
}

void inv_park(float32_t arc_angle, float32_t U_q, float32_t U_d, float32_t *U_alpha, float32_t *U_beta) {
    float32_t cos = arm_cos_f32(arc_angle);
    float32_t sin = arm_sin_f32(arc_angle);

    *U_alpha = U_d * cos - U_q * sin;
    *U_beta = U_d * sin + U_q * cos;
}

void svpwm(float32_t U_alpha, float32_t U_beta, float32_t *duty_a, float32_t *duty_b, float32_t *duty_c) {
    float32_t U_a, U_b, U_c;
    inv_clarke(U_alpha, U_beta, &U_a, &U_b, &U_c);

    float32_t U_max = max3(U_a, U_b, U_c);
    float32_t U_min = min3(U_a, U_b, U_c);

    float32_t span = U_max - U_min;
    if (span >= 1.0f) {
        const float32_t scale = 1.0f/span;
        U_a *= scale;
        U_b *= scale;
        U_c *= scale;

        U_max *= scale;
        U_min *= scale;
    }

    float32_t U_0 = -0.5f * (U_max + U_min);

    *duty_a = 0.5f + (U_a + U_0);
    *duty_b = 0.5f + (U_b + U_0);
    *duty_c = 0.5f + (U_c + U_0);
}

HAL_StatusTypeDef FOCInitialize(
    FOC_HandleTypeDef *self,
    TIM_HandleTypeDef *htim,
    I2C_HandleTypeDef *as5600_handle,
    ADC_HandleTypeDef *hadc1,
    ADC_HandleTypeDef *hadc2,
    ADC_HandleTypeDef *hadc3,
    uint8_t pole_pairs,
    GPIO_TypeDef *driver_enable_port,
    uint16_t driver_enable_pin,
    GPIO_TypeDef *driver_fault_port,
    uint16_t driver_fault_pin) {
    if (self == NULL || htim == NULL || htim->Instance == NULL ||
        hadc1 == NULL || hadc2 == NULL || hadc3 == NULL ||
        as5600_handle == NULL || pole_pairs == 0U ||
        driver_enable_port == NULL || driver_enable_pin == 0U ||
        driver_fault_port == NULL || driver_fault_pin == 0U) {
        return HAL_ERROR;
    }

    self->htim = htim;
    self->as5600_handle = as5600_handle;
    self->hadc1 = hadc1;
    self->hadc2 = hadc2;
    self->hadc3 = hadc3;
    self->driver_enable_port = driver_enable_port;
    self->driver_enable_pin = driver_enable_pin;
    self->driver_fault_port = driver_fault_port;
    self->driver_fault_pin = driver_fault_pin;
    self->pole_pairs = pole_pairs;
    self->rotor_direction = FOC_ROTOR_DIRECTION_SAME;
    self->rotor_offset = 0.0f;
    self->pwm_started = 0U;
    self->Target_Uq = 0.0f;
    self->Target_Ud = 0.0f;
    self->state = FOC_STATE_UNINITIALIZED;
    self->current_u = 0.0f;
    self->current_v = 0.0f;
    self->current_w = 0.0f;
    self->current_offset_u = 0.0f;
    self->current_offset_v = 0.0f;
    self->current_offset_w = 0.0f;
    self->adc_calibration_sum_u = 0U;
    self->adc_calibration_sum_v = 0U;
    self->adc_calibration_sum_w = 0U;
    self->adc_calibration_count = 0U;

    HAL_GPIO_WritePin(self->driver_enable_port, self->driver_enable_pin, GPIO_PIN_RESET);
    __HAL_TIM_MOE_DISABLE(self->htim);

    HAL_StatusTypeDef status = as5600_init(self->as5600_handle);
    if (status != HAL_OK) {
        return status;
    }

    if (pwm_update_duty(self->htim, 0.0f, 0.0f, 0.0f) != HAL_OK) {
        return HAL_ERROR;
    }

    status = driver_wake(self);
    if (status != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return status;
    }

    self->state = FOC_STATE_CALIBRATING;
    status = adc_enable(self);
    if (status != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return status;
    }

    const uint32_t calibration_start_tick = HAL_GetTick();
    while (self->state == FOC_STATE_CALIBRATING) {
        if ((HAL_GetTick() - calibration_start_tick) >=
            FOC_ADC_CALIBRATION_TIMEOUT_MS) {
            driver_shutdown(self, FOC_STATE_FAULT);
            return HAL_TIMEOUT;
        }
    }
    if (self->state != FOC_STATE_ALIGNING) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return HAL_ERROR;
    }

    status = driver_enable(self);
    if (status != HAL_OK) {
        return status;
    }

    status = rotor_align(
        self,
        FOC_ALIGNMENT_VOLTAGE_PU,
        FOC_ALIGNMENT_SETTLE_MS);
    if (status != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return status;
    }

    driver_shutdown(self, FOC_STATE_DISABLED);
    return HAL_OK;
}

HAL_StatusTypeDef FOC_Start(
    FOC_HandleTypeDef *self) {

    if (self == NULL || self->state != FOC_STATE_DISABLED) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = driver_enable(self);
    if (status != HAL_OK) {
        return status;
    }

    self->state = FOC_STATE_RUNNING;
    return HAL_OK;
}

HAL_StatusTypeDef FOC_Stop(FOC_HandleTypeDef *self) {
    if (self == NULL || self->state == FOC_STATE_UNINITIALIZED) {
        return HAL_ERROR;
    }

    if (self->state == FOC_STATE_FAULT) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return HAL_OK;
    }

    driver_shutdown(self, FOC_STATE_DISABLED);
    return HAL_OK;
}

HAL_StatusTypeDef FOC_ClearFault(FOC_HandleTypeDef *self) {
    if (self == NULL || self->state != FOC_STATE_FAULT) {
        return HAL_ERROR;
    }

    driver_shutdown(self, FOC_STATE_FAULT);
    HAL_Delay(DRV8323_RESET_DELAY_MS);

    HAL_StatusTypeDef status = driver_enable(self);
    if (status != HAL_OK) {
        return status;
    }

    driver_shutdown(self, FOC_STATE_DISABLED);
    return HAL_OK;
}

HAL_StatusTypeDef FOC_SetTarget(FOC_HandleTypeDef *self, float32_t Target_Uq, float32_t Target_Ud) {
    if (self != NULL) {
        self->Target_Uq = Target_Uq;
        self->Target_Ud = Target_Ud;
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef FOC_Update(FOC_HandleTypeDef *self) {
    if (self == NULL || self->state != FOC_STATE_RUNNING ||
        self->htim == NULL || self->as5600_handle == NULL) {
        return HAL_ERROR;
    }

    if (driver_fault_active(self)) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return HAL_ERROR;
    }

    uint16_t raw_angle;
    float32_t Target_Uq = self->Target_Uq;
    float32_t Target_Ud = self->Target_Ud;
    float32_t Target_Ualpha, Target_Ubeta;
    float32_t duty_a, duty_b, duty_c;
    if (as5600_read(self->as5600_handle, &raw_angle) != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return HAL_ERROR;
    }
    float32_t magnitude_angle = (float32_t) raw_angle * 2 * PI  / 4096.0f;
    const float32_t direction_sign =
        self->rotor_direction == FOC_ROTOR_DIRECTION_SAME ? 1.0f : -1.0f;
    float32_t electrical_angle =
        direction_sign * (float32_t)self->pole_pairs * magnitude_angle + self->rotor_offset;
    if (limit_dq_voltage(&Target_Uq, &Target_Ud) != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return HAL_ERROR;
    }
    inv_park(electrical_angle, Target_Uq, Target_Ud, &Target_Ualpha, &Target_Ubeta);
    svpwm(Target_Ualpha, Target_Ubeta, &duty_a, &duty_b, &duty_c);
    if (pwm_update_duty(self->htim, duty_a, duty_b, duty_c) != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef FOC_AdcCalibrate(
    FOC_HandleTypeDef *self,
    uint32_t raw_u,
    uint32_t raw_v,
    uint32_t raw_w) {
    if (self == NULL || self->state != FOC_STATE_CALIBRATING ||
        raw_u > FOC_ADC_MAX_VALUE || raw_v > FOC_ADC_MAX_VALUE ||
        raw_w > FOC_ADC_MAX_VALUE) {
        if (self != NULL) {
            self->state = FOC_STATE_FAULT;
        }
        return HAL_ERROR;
    }

    if (self->adc_calibration_count < FOC_ADC_CALIBRATION_DISCARD_COUNT) {
        ++self->adc_calibration_count;
        return HAL_OK;
    }

    self->adc_calibration_sum_u += raw_u;
    self->adc_calibration_sum_v += raw_v;
    self->adc_calibration_sum_w += raw_w;
    ++self->adc_calibration_count;

    const uint32_t valid_sample_count =
        self->adc_calibration_count - FOC_ADC_CALIBRATION_DISCARD_COUNT;
    if (valid_sample_count < FOC_ADC_CALIBRATION_SAMPLE_COUNT) {
        return HAL_OK;
    }

    self->current_offset_u =
        (float32_t)self->adc_calibration_sum_u /
        (float32_t)FOC_ADC_CALIBRATION_SAMPLE_COUNT;
    self->current_offset_v =
        (float32_t)self->adc_calibration_sum_v /
        (float32_t)FOC_ADC_CALIBRATION_SAMPLE_COUNT;
    self->current_offset_w =
        (float32_t)self->adc_calibration_sum_w /
        (float32_t)FOC_ADC_CALIBRATION_SAMPLE_COUNT;
    self->current_u = 0.0f;
    self->current_v = 0.0f;
    self->current_w = 0.0f;
    self->state = FOC_STATE_ALIGNING;

    return HAL_OK;
}

HAL_StatusTypeDef FOC_CurrentSample(
    FOC_HandleTypeDef *self,
    uint32_t raw_u,
    uint32_t raw_v,
    uint32_t raw_w) {
    if (self == NULL) {
        return HAL_ERROR;
    }
    if (self->state == FOC_STATE_CALIBRATING) {
        return FOC_AdcCalibrate(self, raw_u, raw_v, raw_w);
    }
    if (self->state != FOC_STATE_ALIGNING &&
        self->state != FOC_STATE_RUNNING) {
        self->current_u = 0.0f;
        self->current_v = 0.0f;
        self->current_w = 0.0f;
        return HAL_OK;
    }
    const float32_t current_scale =
        3.3f /
        (4095.0f * FOC_CURRENT_SAMPLE_RESISTER * FOC_CURRENT_SAMPLE_GAIN);
    const float32_t measured_current_u =
        ((float32_t)raw_u - self->current_offset_u) * current_scale;
    const float32_t measured_current_v =
        ((float32_t)raw_v - self->current_offset_v) * current_scale;
    const float32_t measured_current_w =
        ((float32_t)raw_w - self->current_offset_w) * current_scale;
    const float32_t common_mode_current =
        (measured_current_u + measured_current_v + measured_current_w) / 3.0f;

    self->current_u = measured_current_u - common_mode_current;
    self->current_v = measured_current_v - common_mode_current;
    self->current_w = measured_current_w - common_mode_current;
    return HAL_OK;
}
