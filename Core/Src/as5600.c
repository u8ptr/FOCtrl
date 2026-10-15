#include "as5600.h"

#include <stdio.h>


HAL_StatusTypeDef as5600_init(I2C_HandleTypeDef *hi2c) {
    if (hi2c != NULL) {
        uint8_t status = 0;
        HAL_StatusTypeDef i2c_status = HAL_I2C_Mem_Read(
            hi2c,
            AS5600_ADDR,
            AS5600_REG_STATUS,
            I2C_MEMADD_SIZE_8BIT,
            &status,
            1,
            100);
        // if (i2c_status != HAL_OK) {
        //     return i2c_status;
        // }
        // if ((status & AS5600_STATUS_MD_MASK) == 0U){
        //     return HAL_ERROR;
        // }
        uint8_t reg = AS5600_REG_RAW_ANGLE_H;
        i2c_status = HAL_I2C_Master_Transmit(
            hi2c,
            AS5600_ADDR,
            &reg,
            1,
            100);
        if (i2c_status != HAL_OK) {
            return i2c_status;
        }
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef as5600_read(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle) {
    if (hi2c != NULL && raw_angle != NULL) {
        uint8_t buffer[2];
        HAL_StatusTypeDef i2c_status = HAL_I2C_Master_Receive(
            hi2c,
            AS5600_ADDR,
            buffer,
            2,
            100);
        if (i2c_status != HAL_OK) {
            return i2c_status;
        }
        *raw_angle = (buffer[0] << 8 | buffer[1]) & 0x0FFF;
        return HAL_OK;
    }
    return HAL_ERROR;
}
