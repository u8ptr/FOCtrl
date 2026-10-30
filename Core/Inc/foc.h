#ifndef FOCTRL_FOC_H
#define FOCTRL_FOC_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "as5600.h"

#define FOC_VOLTAGE_MARGIN 0.98f
#define FOC_ROTOR_DIRECTION_OPPOSITE 0U
#define FOC_ROTOR_DIRECTION_SAME     1U
#define FOC_CURRENT_SAMPLE_GAIN 5U
#define FOC_CURRENT_SAMPLE_RESISTER 0.004f

typedef enum {
    FOC_STATE_UNINITIALIZED = 0,
    FOC_STATE_DISABLED,
    FOC_STATE_CALIBRATING,
    FOC_STATE_ALIGNING,
    FOC_STATE_RUNNING,
    FOC_STATE_FAULT
} FOC_StateTypeDef;

typedef struct {
    TIM_HandleTypeDef *htim;
    I2C_HandleTypeDef *as5600_handle;
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;
    ADC_HandleTypeDef *hadc3;

    GPIO_TypeDef *driver_enable_port;
    uint16_t driver_enable_pin;
    GPIO_TypeDef *driver_fault_port;
    uint16_t driver_fault_pin;

    uint8_t pole_pairs;
    float32_t rotor_offset;
    uint8_t rotor_direction;
    float32_t electrical_angle;
    uint8_t pwm_started;
    volatile FOC_StateTypeDef state;

    float32_t Target_Uq;
    float32_t Target_Ud;

    float32_t I_q;
    float32_t I_d;

    float32_t current_u;
    float32_t current_v;
    float32_t current_w;
    float32_t current_offset_u;
    float32_t current_offset_v;
    float32_t current_offset_w;

    uint64_t adc_calibration_sum_u;
    uint64_t adc_calibration_sum_v;
    uint64_t adc_calibration_sum_w;
    uint32_t adc_calibration_count;
} FOC_HandleTypeDef;

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
    uint16_t driver_fault_pin);

HAL_StatusTypeDef FOC_Start(FOC_HandleTypeDef *self);
HAL_StatusTypeDef FOC_Stop(FOC_HandleTypeDef *self);
HAL_StatusTypeDef FOC_ClearFault(FOC_HandleTypeDef *self);
HAL_StatusTypeDef FOC_SetTarget(FOC_HandleTypeDef *self, float32_t Target_Uq, float32_t Target_Ud);
HAL_StatusTypeDef FOC_Update(FOC_HandleTypeDef *self);

HAL_StatusTypeDef FOC_CurrentSample(
    FOC_HandleTypeDef *self,
    uint32_t raw_u,
    uint32_t raw_v,
    uint32_t raw_w);

void inv_park(float32_t arc_angle, float32_t U_q, float32_t U_d, float32_t *U_alpha, float32_t *U_beta);
void inv_clarke(float32_t U_alpha, float32_t U_beta, float32_t *U_a, float32_t *U_b,
              float32_t *U_c);
void svpwm(float32_t U_alpha, float32_t U_beta, float32_t *duty_a, float32_t *duty_b, float32_t *duty_c);
void clarke(float32_t I_u, float32_t I_v, float32_t I_w, float32_t *I_a, float32_t *I_b);

#endif //FOCTRL_FOC_H
