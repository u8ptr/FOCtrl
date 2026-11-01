#include "as5600.h"

static uint8_t rx_buffer[2];

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

HAL_StatusTypeDef as5600_trigger(I2C_HandleTypeDef *hi2c) {
    if (hi2c == NULL) {
        return HAL_ERROR;
    }
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(
        hi2c,
        AS5600_ADDR,
        rx_buffer,
        sizeof(rx_buffer));
    return status;
}

HAL_StatusTypeDef as5600_callback(float32_t *magnitude_angle) {
    if (magnitude_angle == NULL) {
        return HAL_ERROR;
    }

    const uint16_t raw_angle =
        (((uint16_t)rx_buffer[0] << 8U) | (uint16_t)rx_buffer[1]) & 0x0FFFU;
    *magnitude_angle = (float32_t)raw_angle * 2.0f * PI / 4096.0f;
    return HAL_OK;
}

HAL_StatusTypeDef as5600_read_blocking(
    I2C_HandleTypeDef *hi2c,
    uint16_t *raw_angle) {
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
        *raw_angle =
            (((uint16_t)buffer[0] << 8U) | (uint16_t)buffer[1]) & 0x0FFFU;
        return HAL_OK;
    }
    return HAL_ERROR;
}
