#ifndef FOCTRL_AS5600_H
#define FOCTRL_AS5600_H

#include "stm32f4xx_hal.h"

#define AS5600_ADDR             (0x36U << 1)

#define AS5600_REG_RAW_ANGLE_H  0x0CU
#define AS5600_REG_RAW_ANGLE_L  0x0DU
#define AS5600_REG_STATUS       0x0BU

#define AS5600_STATUS_MD_MASK   0x20U


HAL_StatusTypeDef as5600_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef as5600_read(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle);

#endif //FOCTRL_AS5600_H