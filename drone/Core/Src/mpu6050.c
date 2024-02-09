#include "mpu6050.h"

HAL_StatusTypeDef mpu6050_i2c_result;
uint8_t mpu6050_1byte_data;
uint8_t mpu6050_6byte_data[6];
int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int16_t temp_raw;
int16_t manual_gyro_pitch_cal_value, manual_gyro_roll_cal_value, manual_gyro_yaw_cal_value;
int16_t manual_accel_roll_cal_value, manual_accel_pitch_cal_value;
int32_t gyro_calibration_value_roll, gyro_calibration_value_pitch, gyro_calibration_value_yaw;
int32_t accel_calibration_value_roll, accel_calibration_value_pitch;
float   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp_in_c;
float   gyro_input_roll, gyro_input_pitch, gyro_input_yaw;

mpu6050_result_t mpu6050_callback_write(uint8_t *mpu6050_data, uint8_t *mpu6050_i2c_result, mpu6050_memory_address_t *memory_address, uint8_t mpu6050_data_size) {
    if (mpu6050_data == NULL || mpu6050_i2c_result == NULL || memory_address == NULL) {
        return MPU6050_ERROR;
    }

    *mpu6050_i2c_result = HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C__ADDRESS, *memory_address, MPU6050_I2C__MEM_ADDR_SIZE,
                                            mpu6050_data, mpu6050_data_size, 1000);

    if (*mpu6050_i2c_result == HAL_OK) {
        return MPU6050_SUCCESS;
    } else {
        return MPU6050_ERROR;
    }
}

mpu6050_result_t mpu6050_callback_read(uint8_t *mpu6050_data, uint8_t *mpu6050_i2c_result, mpu6050_memory_address_t *memory_address, uint8_t mpu6050_data_size) {
    if (mpu6050_data == NULL || mpu6050_i2c_result == NULL || memory_address == NULL) {
        return MPU6050_ERROR;
    }

    *mpu6050_i2c_result = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C__ADDRESS, *memory_address, MPU6050_I2C__MEM_ADDR_SIZE,
                                            mpu6050_data, mpu6050_data_size, 1000);

    if (*mpu6050_i2c_result == HAL_OK) {
        return MPU6050_SUCCESS;
    } else {
        return MPU6050_ERROR;
    }
}

mpu6050_result_t mpu6050_wake_up(mpu6050_callback_t mpu6050_callback, HAL_StatusTypeDef *mpu6050_i2c_result) {
    mpu6050_memory_address_t memory_address = MPU6050_MEMORY_ADDRESS_PWR_MGMT_1;
    uint8_t mpu6050_data = MPU6050_I2C__CLKSEL;

    return mpu6050_callback(&mpu6050_data, mpu6050_i2c_result, &memory_address, 1);
}

mpu6050_result_t mpu6050_set_data_rate(mpu6050_callback_t mpu6050_callback, HAL_StatusTypeDef *mpu6050_i2c_result) {
    mpu6050_memory_address_t memory_address = MPU6050_MEMORY_ADDRESS_SMPRT_DIV;
    uint8_t mpu6050_data = MPU6050_I2C__DLPF_CFG;

    return mpu6050_callback(&mpu6050_data, mpu6050_i2c_result, &memory_address, 1);
}

mpu6050_result_t mpu6050_accel_config(mpu6050_callback_t mpu6050_callback, HAL_StatusTypeDef *mpu6050_i2c_result) {
    mpu6050_memory_address_t memory_address = MPU6050_MEMORY_ADDRESS_ACCEL_CONFIG;
    // uint8_t mpu6050_data = MPU6050_I2C__ACCEL_AFS_SEL;
    uint8_t mpu6050_data = 0x10;

    return mpu6050_callback(&mpu6050_data, mpu6050_i2c_result, &memory_address, 1);
}

mpu6050_result_t mpu6050_gyro_config(mpu6050_callback_t mpu6050_callback, HAL_StatusTypeDef *mpu6050_i2c_result) {
    mpu6050_memory_address_t memory_address = MPU6050_MEMORY_ADDRESS_GYRO_CONFIG;
    // uint8_t mpu6050_data = MPU6050_I2C__GYRO_FS_SEL;
    uint8_t mpu6050_data = 0x08;

    return mpu6050_callback(&mpu6050_data, mpu6050_i2c_result, &memory_address, 1);
}

mpu6050_result_t mpu6050_config(mpu6050_callback_t mpu6050_callback, HAL_StatusTypeDef *mpu6050_i2c_result) {
    mpu6050_memory_address_t memory_address = MPU6050_MEMORY_ADDRESS_CONFIG;
    uint8_t mpu6050_data = 0x03;

    return mpu6050_callback(&mpu6050_data, mpu6050_i2c_result, &memory_address, 1);
}

mpu6050_result_t mpu6050_init(uint8_t *mpu6050_data, HAL_StatusTypeDef *mpu6050_i2c_result) {
    if (mpu6050_data == NULL) {
        return MPU6050_ERROR;
    }

    mpu6050_memory_address_t memory_address = MPU6050_MEMORY_ADDRESS_WHO_AM_I;
    *mpu6050_i2c_result = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C__ADDRESS, memory_address, MPU6050_I2C__MEM_ADDR_SIZE,
                                           mpu6050_data, MPU6050_I2C__MEM_ADDR_SIZE, 1000);

    if (*mpu6050_data != 0x68) {
        return MPU6050_ERROR;
    }

    if (mpu6050_wake_up(mpu6050_callback_write, mpu6050_i2c_result) == MPU6050_ERROR) {
        return MPU6050_ERROR;
    }
    if (mpu6050_set_data_rate(mpu6050_callback_write, mpu6050_i2c_result) == MPU6050_ERROR) {
        return MPU6050_ERROR;
    }
    if (mpu6050_accel_config(mpu6050_callback_write, mpu6050_i2c_result) == MPU6050_ERROR) {
        return MPU6050_ERROR;
    }
    if (mpu6050_gyro_config(mpu6050_callback_write, mpu6050_i2c_result) == MPU6050_ERROR) {
        return MPU6050_ERROR;
    }
    if (mpu6050_config(mpu6050_callback_write, mpu6050_i2c_result) == MPU6050_ERROR) {
        return MPU6050_ERROR;
    }

    return MPU6050_SUCCESS;
}

mpu6050_result_t mpu6050_accel_read(mpu6050_callback_t mpu6050_callback, uint8_t *mpu6050_data, HAL_StatusTypeDef *mpu6050_i2c_result) {
    mpu6050_memory_address_t memory_address = MPU6050_MEMORY_ADDRESS_ACCEL_XOUT_H;
    
    if (mpu6050_callback(mpu6050_6byte_data, mpu6050_i2c_result, &memory_address, 6) == MPU6050_ERROR) {
        return MPU6050_ERROR;
    }

	accel_y_raw = (int16_t)((mpu6050_data[0] << 8) | mpu6050_data [1]);
	accel_x_raw = (int16_t)((mpu6050_data[2] << 8) | mpu6050_data [3]);
	accel_z_raw = (int16_t)((mpu6050_data[4] << 8) | mpu6050_data [5]);

    accel_x_raw -= manual_accel_roll_cal_value;
    accel_y_raw -= manual_accel_pitch_cal_value;
	// accel_x = accel_x_raw/4096.0;
	// accel_y = accel_y_raw/4096.0;
	// accel_z = accel_z_raw/4096.0;

	return MPU6050_SUCCESS;
}

mpu6050_result_t mpu6050_gyro_read(mpu6050_callback_t mpu6050_callback, uint8_t *mpu6050_data, HAL_StatusTypeDef *mpu6050_i2c_result) {
    mpu6050_memory_address_t memory_address = MPU6050_MEMORY_ADDRESS_GYRO_XOUT_H;

    if (mpu6050_callback(mpu6050_data, mpu6050_i2c_result, &memory_address, 6) == MPU6050_ERROR) {
        return MPU6050_ERROR;
    }

	gyro_x_raw = (int16_t)((mpu6050_data[0] << 8) | mpu6050_data [1]);
	gyro_y_raw = (int16_t)((mpu6050_data[2] << 8) | mpu6050_data [3]);
	gyro_z_raw = (int16_t)((mpu6050_data[4] << 8) | mpu6050_data [5]);

    gyro_y_raw *= (-1);
    gyro_z_raw *= (-1);

    gyro_x_raw -= manual_gyro_roll_cal_value;
	gyro_y_raw -= manual_gyro_pitch_cal_value;
	gyro_z_raw -= manual_gyro_yaw_cal_value;

	// gyro_x = gyro_x_raw/65.5;
	// gyro_y = gyro_y_raw/65.5;
	// gyro_z = gyro_z_raw/65.5;

	return MPU6050_SUCCESS;
}

mpu6050_result_t mpu6050_temp_read(mpu6050_callback_t mpu6050_callback, uint8_t *mpu6050_data, HAL_StatusTypeDef *mpu6050_i2c_result) {
    mpu6050_memory_address_t memory_address = MPU6050_MEMORY_ADDRESS_TEMP_OUT_H;

    if (mpu6050_callback(mpu6050_data, mpu6050_i2c_result, &memory_address, 2) == MPU6050_ERROR) {
        return MPU6050_ERROR;
    }

	temp_raw = (int16_t)(mpu6050_data[0] << 8 | mpu6050_data [1]);

    temp_in_c = (temp_raw / 340) + 36.53;

	return MPU6050_SUCCESS;
}

mpu6050_result_t mpu6050_gyro_calibrate() {
    for (int index = 0; index < 2000; ++index) {
        mpu6050_gyro_read(mpu6050_callback_read, mpu6050_6byte_data, &mpu6050_i2c_result);

        gyro_calibration_value_roll  += gyro_x_raw;
        gyro_calibration_value_pitch += gyro_y_raw;
        gyro_calibration_value_yaw   += gyro_z_raw;
        HAL_Delay(4);
    }

    gyro_calibration_value_roll  /= 2000;
    gyro_calibration_value_pitch /= 2000;
    gyro_calibration_value_yaw   /= 2000;

    manual_gyro_roll_cal_value = gyro_calibration_value_roll;                                     //Set the manual pitch calibration variable to the detected value.
    manual_gyro_pitch_cal_value = gyro_calibration_value_pitch;                                   //Set the manual roll calibration variable to the detected value.
    manual_gyro_yaw_cal_value = gyro_calibration_value_yaw;                                       //Set the manual yaw calibration variable to the detected value.

    return MPU6050_SUCCESS;
}

mpu6050_result_t mpu6050_accel_calibrate() {
    for (int index = 0; index < 2000; ++index) {
        mpu6050_accel_read(mpu6050_callback_read, mpu6050_6byte_data, &mpu6050_i2c_result);

        accel_calibration_value_roll  += accel_x_raw;
        accel_calibration_value_pitch += accel_y_raw;
        HAL_Delay(4);
    }

    accel_calibration_value_roll  /= 2000;
    accel_calibration_value_pitch /= 2000;

    manual_accel_roll_cal_value = accel_calibration_value_roll;                                     //Set the manual pitch calibration variable to the detected value.
    manual_accel_pitch_cal_value = accel_calibration_value_pitch;                                       //Set the manual roll calibration variable to the detected value.

    return MPU6050_SUCCESS;
}
