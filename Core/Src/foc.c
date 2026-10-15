#include "foc.h"

#include <math.h>

#define FOC_TWO_PI                (2.0f * PI)
#define FOC_ENCODER_CPR           4096
#define FOC_DIRECTION_TEST_ANGLE  (0.5f * PI)
#define FOC_DIRECTION_MIN_DELTA   2
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
    self->state = next_state;
}

static HAL_StatusTypeDef driver_enable(FOC_HandleTypeDef *self) {
    HAL_StatusTypeDef status = pwm_start_all(self);
    if (status != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return status;
    }

    HAL_GPIO_WritePin(self->driver_enable_port, self->driver_enable_pin, GPIO_PIN_SET);
    HAL_Delay(DRV8323_WAKE_DELAY_MS);

    if (driver_fault_active(self)) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return HAL_ERROR;
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

static HAL_StatusTypeDef rotor_align(
    FOC_HandleTypeDef *self,
    float32_t alignment_voltage_pu,
    uint32_t alignment_time_ms) {
    uint16_t initial_raw_angle;
    uint16_t probe_raw_angle;
    uint16_t aligned_raw_angle;

    self->state = FOC_STATE_ALIGNING;
    if (apply_alignment_vector(self, alignment_voltage_pu, 0.0f) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(alignment_time_ms);
    if (driver_fault_active(self)) {
        return HAL_ERROR;
    }
    if (as5600_read(self->as5600_handle, &initial_raw_angle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (apply_alignment_vector(
            self,
            alignment_voltage_pu,
            FOC_DIRECTION_TEST_ANGLE) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(alignment_time_ms);
    if (driver_fault_active(self) ||
        as5600_read(self->as5600_handle, &probe_raw_angle) != HAL_OK) {
        return HAL_ERROR;
    }

    int32_t encoder_delta = (int32_t)probe_raw_angle - (int32_t)initial_raw_angle;
    if (encoder_delta > FOC_ENCODER_CPR / 2) {
        encoder_delta -= FOC_ENCODER_CPR;
    } else if (encoder_delta < -(FOC_ENCODER_CPR / 2)) {
        encoder_delta += FOC_ENCODER_CPR;
    }
    if (encoder_delta >= -FOC_DIRECTION_MIN_DELTA &&
        encoder_delta <= FOC_DIRECTION_MIN_DELTA) {
        return HAL_ERROR;
    }
    self->rotor_direction = encoder_delta > 0
        ? FOC_ROTOR_DIRECTION_SAME
        : FOC_ROTOR_DIRECTION_OPPOSITE;

    if (apply_alignment_vector(self, alignment_voltage_pu, 0.0f) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(alignment_time_ms);
    if (driver_fault_active(self) ||
        as5600_read(self->as5600_handle, &aligned_raw_angle) != HAL_OK) {
        return HAL_ERROR;
    }

    const float32_t mechanical_angle =
        (float32_t)aligned_raw_angle * FOC_TWO_PI / (float32_t)FOC_ENCODER_CPR;
    const float32_t direction_sign =
        self->rotor_direction == FOC_ROTOR_DIRECTION_SAME ? 1.0f : -1.0f;
    self->rotor_offset = wrap_angle_2pi(
        -(direction_sign * (float32_t)self->pole_pairs * mechanical_angle));
    self->Target_Ud = 0.0f;
    self->Target_Uq = 0.0f;

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
    uint8_t pole_pairs,
    GPIO_TypeDef *driver_enable_port,
    uint16_t driver_enable_pin,
    GPIO_TypeDef *driver_fault_port,
    uint16_t driver_fault_pin) {
    if (self == NULL || htim == NULL || htim->Instance == NULL ||
        as5600_handle == NULL || pole_pairs == 0U ||
        driver_enable_port == NULL || driver_enable_pin == 0U ||
        driver_fault_port == NULL || driver_fault_pin == 0U) {
        return HAL_ERROR;
    }

    self->htim = htim;
    self->as5600_handle = as5600_handle;
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

    HAL_GPIO_WritePin(self->driver_enable_port, self->driver_enable_pin, GPIO_PIN_RESET);
    __HAL_TIM_MOE_DISABLE(self->htim);

    HAL_StatusTypeDef status = as5600_init(self->as5600_handle);
    if (status != HAL_OK) {
        return status;
    }

    if (pwm_update_duty(self->htim, 0.5f, 0.5f, 0.5f) != HAL_OK) {
        return HAL_ERROR;
    }

    if (driver_enable(self) != HAL_OK) {
        return status;
    }

    status = rotor_align(self, 0.5f, 5.0f);
    if (status != HAL_OK) {
        driver_shutdown(self, FOC_STATE_FAULT);
        return status;
    }

    driver_shutdown(self, FOC_STATE_FAULT);
    return HAL_OK;
}

HAL_StatusTypeDef FOC_Start(
    FOC_HandleTypeDef *self) {

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
