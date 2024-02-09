#ifndef MPU6050_H_
#define MPU6050_H_

#include "main.h"
#include "stdint.h"

#define MPU6050_I2C__MEM_ADDR_SIZE  0x01      /**<*/
#define MPU6050_I2C__ADDRESS		0xD0      /**< 208/2 is 104 which is 0x68. */
#define MPU6050_I2C__CLKSEL         0x00      /**< Internal 8MHz oscillator. */
#define MPU6050_I2C__DLPF_CFG       0x07
#define MPU6050_I2C__GYRO_FS_SEL    0x01      /**< ± 500 °/s. */
#define MPU6050_I2C__ACCEL_AFS_SEL  0x02      /**< 4096 LSB/g. */

extern I2C_HandleTypeDef hi2c1;

extern HAL_StatusTypeDef mpu6050_i2c_result;
extern uint8_t mpu6050_1byte_data;
extern uint8_t mpu6050_6byte_data[6];
extern int16_t accel_x_raw, accel_y_raw, accel_z_raw;
extern int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
extern int16_t temp_raw;
extern int16_t manual_gyro_pitch_cal_value, manual_gyro_roll_cal_value, manual_gyro_yaw_cal_value;
extern int16_t manual_accel_roll_cal_value, manual_accel_pitch_cal_value;
extern int32_t gyro_calibration_value_roll, gyro_calibration_value_pitch, gyro_calibration_value_yaw;
extern int32_t accel_calibration_value_roll, accel_calibration_value_pitch;
extern float   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp_in_c;
extern float   gyro_input_roll, gyro_input_pitch, gyro_input_yaw;

typedef enum {
    MPU6050_SUCCESS = 0,
    MPU6050_ERROR,
} mpu6050_result_t;

typedef enum {
    MPU6050_MEMORY_ADDRESS_SMPRT_DIV    = 0x19,
    MPU6050_MEMORY_ADDRESS_CONFIG       = 0x1A,
    MPU6050_MEMORY_ADDRESS_GYRO_CONFIG  = 0x1B,
    MPU6050_MEMORY_ADDRESS_ACCEL_CONFIG = 0x1C,
    MPU6050_MEMORY_ADDRESS_ACCEL_XOUT_H = 0x3B,
    MPU6050_MEMORY_ADDRESS_TEMP_OUT_H   = 0x41,
    MPU6050_MEMORY_ADDRESS_GYRO_XOUT_H  = 0x43,
    MPU6050_MEMORY_ADDRESS_PWR_MGMT_1   = 0x6B,
    MPU6050_MEMORY_ADDRESS_WHO_AM_I     = 0x75,
} mpu6050_memory_address_t;

typedef mpu6050_result_t (*mpu6050_callback_t)(uint8_t *, uint8_t *, mpu6050_memory_address_t *, uint8_t);

mpu6050_result_t mpu6050_callback_write(uint8_t *mpu6050_data, uint8_t *mpu6050_i2c_result, mpu6050_memory_address_t *memory_address, uint8_t mpu6050_data_size);
mpu6050_result_t mpu6050_callback_read(uint8_t *mpu6050_data, uint8_t *mpu6050_i2c_result, mpu6050_memory_address_t *memory_address, uint8_t mpu6050_data_size);

mpu6050_result_t mpu6050_wake_up(mpu6050_callback_t mpu6050_callback, HAL_StatusTypeDef *mpu6050_i2c_result);
mpu6050_result_t mpu6050_set_data_rate(mpu6050_callback_t mpu6050_callback, HAL_StatusTypeDef *mpu6050_i2c_result);
mpu6050_result_t mpu6050_accel_config(mpu6050_callback_t mpu6050_callback, HAL_StatusTypeDef *mpu6050_i2c_result);
mpu6050_result_t mpu6050_gyro_config(mpu6050_callback_t mpu6050_callback, HAL_StatusTypeDef *mpu6050_i2c_result);
mpu6050_result_t mpu6050_init(uint8_t *mpu6050_1byte_data, HAL_StatusTypeDef *mpu6050_i2c_result);
mpu6050_result_t mpu6050_accel_read(mpu6050_callback_t mpu6050_callback, uint8_t *mpu6050_data, HAL_StatusTypeDef *mpu6050_i2c_result);
mpu6050_result_t mpu6050_gyro_read(mpu6050_callback_t mpu6050_callback, uint8_t *mpu6050_data, HAL_StatusTypeDef *mpu6050_i2c_result);
mpu6050_result_t mpu6050_temp_read(mpu6050_callback_t mpu6050_callback, uint8_t *mpu6050_data, HAL_StatusTypeDef *mpu6050_i2c_result);
mpu6050_result_t mpu6050_gyro_calibrate();
mpu6050_result_t mpu6050_accel_calibrate();

#endif  /* MPU6050_H_ */
