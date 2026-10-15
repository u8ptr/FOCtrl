#ifndef FOCTRL_FOC_H
#define FOCTRL_FOC_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "as5600.h"

#define FOC_VOLTAGE_MARGIN 0.98f
#define FOC_ROTOR_DIRECTION_OPPOSITE 0U
#define FOC_ROTOR_DIRECTION_SAME     1U

typedef enum {
    FOC_STATE_UNINITIALIZED = 0,
    FOC_STATE_DISABLED,
    FOC_STATE_ALIGNING,
    FOC_STATE_RUNNING,
    FOC_STATE_FAULT
} FOC_StateTypeDef;

typedef struct {
    TIM_HandleTypeDef *htim;
    I2C_HandleTypeDef *as5600_handle;

    GPIO_TypeDef *driver_enable_port;
    uint16_t driver_enable_pin;
    GPIO_TypeDef *driver_fault_port;
    uint16_t driver_fault_pin;

    uint8_t pole_pairs;
    float32_t rotor_offset;
    uint8_t rotor_direction;
    uint8_t pwm_started;
    FOC_StateTypeDef state;

    float32_t Target_Uq;
    float32_t Target_Ud;
} FOC_HandleTypeDef;

HAL_StatusTypeDef FOCInitialize(
    FOC_HandleTypeDef *self,
    TIM_HandleTypeDef *htim,
    I2C_HandleTypeDef *as5600_handle,
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

void inv_park(float32_t arc_angle, float32_t U_q, float32_t U_d, float32_t *U_alpha, float32_t *U_beta);
void inv_clarke(float32_t U_alpha, float32_t U_beta, float32_t *U_a, float32_t *U_b,
              float32_t *U_c);
void svpwm(float32_t U_alpha, float32_t U_beta, float32_t *duty_a, float32_t *duty_b, float32_t *duty_c);

#endif //FOCTRL_FOC_H
