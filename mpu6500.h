// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

typedef err_code_t (*mpu6500_func_i2c_recv)(uint8_t *buf_read, uint16_t len, uint32_t timeout_ms);
typedef err_code_t (*mpu6500_func_i2c_send)(uint8_t *buf_send, uint16_t len, uint32_t timeout_ms);
typedef void (*mpu6500_func_delay)(uint32_t ms);

/**
 * @brief   Handle structure.
 */
typedef struct mpu6500 *mpu6500_handle_t;

/**
 * @brief   Clock source select.
 */
typedef enum {
	MPU6500_CLKSEL_INTERNAL_20_MHZ = 0,     /*!< Internal 20 MHz clock source */
	MPU6500_CLKSEL_AUTO,                    /*!< Auto select available clock source */
	MPU6500_CLKSEL_MAX
} mpu6500_clksel_t;

/**
 * @brief   Bandwidth select.
 */
typedef enum {
	MPU6500_250ACEL_4000GYRO_BW_HZ = 0,     /*!< 250 Hz accelerometer bandwidth, 4000 Hz gyroscope bandwidth */
	MPU6500_184ACEL_188GYRO_BW_HZ,          /*!< 184 Hz accelerometer bandwidth, 188 Hz gyroscope bandwidth */
	MPU6500_92ACEL_98GYRO_BW_HZ,            /*!< 92 Hz accelerometer bandwidth, 98 Hz gyroscope bandwidth */
	MPU6500_41ACEL_42GYRO_BW_HZ,            /*!< 41 Hz accelerometer bandwidth, 42 Hz gyroscope bandwidth */
	MPU6500_20ACEL_20GYRO_BW_HZ,            /*!< 20 Hz accelerometer bandwidth, 20 Hz gyroscope bandwidth */
	MPU6500_10ACEL_10GYRO_BW_HZ,            /*!< 10 Hz accelerometer bandwidth, 10 Hz gyroscope bandwidth */
	MPU6500_5ACEL_5GYRO_BW_HZ,              /*!< 5 Hz accelerometer bandwidth, 5 Hz gyroscope bandwidth */
	MPU6500_DLPF_CFG_MAX
} mpu6500_dlpf_cfg_t;

/**
 * @brief   Sleep mode.
 */
typedef enum {
	MPU6500_DISABLE_SLEEP_MODE = 0,         /*!< Disable sleep mode */
	MPU6500_LOW_PWR_SLEEP_MODE,             /*!< Low power mode */
	MPU6500_SLEEP_MODE_MAX
} mpu6500_sleep_mode_t;

/**
 * @brief   Gyroscope full scale.
 */
typedef enum {
	MPU6500_GFS_SEL_250 = 0,                /*!< 250 deg/s */
	MPU6500_GFS_SEL_500,                    /*!< 500 deg/s */
	MPU6500_GFS_SEL_1000,                   /*!< 1000 deg/s */
	MPU6500_GFS_SEL_2000,                   /*!< 2000 deg/s */
	MPU6500_GFS_SEL_MAX
} mpu6500_gfs_sel_t;

/**
 * @brief   Accelerometer full scale.
 */
typedef enum {
	MPU6500_AFS_SEL_2G = 0,                 /*!< 2g */
	MPU6500_AFS_SEL_4G,                     /*!< 4g */
	MPU6500_AFS_SEL_8G,                     /*!< 8g */
	MPU6500_AFS_SEL_16G,                    /*!< 16g */
	MPU6500_AFS_SEL_MAX
} mpu6500_afs_sel_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	mpu6500_clksel_t        	clksel;         			/*!< MPU6500 clock source */
	mpu6500_dlpf_cfg_t      	dlpf_cfg;       			/*!< MPU6500 digital low pass filter (DLPF) */
	mpu6500_sleep_mode_t    	sleep_mode;     			/*!< MPU6500 sleep mode */
	mpu6500_gfs_sel_t        	gfs_sel;         			/*!< MPU6500 gyroscope full scale range */
	mpu6500_afs_sel_t       	afs_sel;        			/*!< MPU6500 accelerometer full scale range */
	int16_t                     accel_bias_x;               /*!< Accelerometer bias of x axis */
	int16_t                     accel_bias_y;               /*!< Accelerometer bias of y axis */
	int16_t                     accel_bias_z;               /*!< Accelerometer bias of z axis */
	int16_t                     gyro_bias_x;                /*!< Gyroscope bias of x axis */
	int16_t                     gyro_bias_y;                /*!< Gyroscope bias of y axis */
	int16_t                     gyro_bias_z;                /*!< Gyroscope bias of z axis */
	mpu6500_func_i2c_recv       i2c_recv;         			/*!< MPU6500 receive bytes */
	mpu6500_func_i2c_send       i2c_send;        			/*!< MPU6500 send bytes */
	mpu6500_func_delay          delay;                 		/*!< MPU6500 delay function */
} mpu6500_cfg_t;

/*
 * @brief   Initialize MPU6500 with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mpu6500_handle_t mpu6500_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_set_config(mpu6500_handle_t handle, mpu6500_cfg_t config);

/*
 * @brief   Configure MPU6500 to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_config(mpu6500_handle_t handle);

/*
 * @brief   Get accelerometer raw value.
 *
 * @param   handle Handle structure.
 * @param   raw_x Raw value x axis.
 * @param   raw_y Raw value y axis.
 * @param   raw_z Raw value z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_get_accel_raw(mpu6500_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);

/*
 * @brief   Get accelerometer calibrated data.
 *
 * @param   handle Handle structure.
 * @param   calib_x Calibrated data x axis.
 * @param   calib_y Calibrated data y axis.
 * @param   calib_z Calibrated data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_get_accel_calib(mpu6500_handle_t handle, int16_t *calib_x, int16_t *calib_y, int16_t *calib_z);

/*
 * @brief   Get accelerometer scaled data.
 *
 * @param   handle Handle structure.
 * @param   scale_x Scaled data x axis.
 * @param   scale_y Scaled data y axis.
 * @param   scale_z Scaled data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_get_accel_scale(mpu6500_handle_t handle, float *scale_x, float *scale_y, float *scale_z);

/*
 * @brief   Get gyroscope raw value.
 *
 * @param   handle Handle structure.
 * @param   raw_x Raw value x axis.
 * @param   raw_y Raw value y axis.
 * @param   raw_z Raw value z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_get_gyro_raw(mpu6500_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);

/*
 * @brief   Get gyroscope calibrated data.
 *
 * @param   handle Handle structure.
 * @param   calib_x Calibrated data x axis.
 * @param   calib_y Calibrated data y axis.
 * @param   calib_z Calibrated data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_get_gyro_calib(mpu6500_handle_t handle, int16_t *calib_x, int16_t *calib_y, int16_t *calib_z);

/*
 * @brief   Get gyroscope scaled data.
 *
 * @param   handle Handle structure.
 * @param   scale_x Scaled data x axis.
 * @param   scale_y Scaled data y axis.
 * @param   scale_z Scaled data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_get_gyro_scale(mpu6500_handle_t handle, float *scale_x, float *scale_y, float *scale_z);

/*
 * @brief   Set accelerometer bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_set_accel_bias(mpu6500_handle_t handle, int16_t bias_x, int16_t bias_y, int16_t bias_z);

/*
 * @brief   Set gyroscope bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_set_gyro_bias(mpu6500_handle_t handle, int16_t bias_x, int16_t bias_y, int16_t bias_z);

/*
 * @brief   Get accelerometer bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_get_accel_bias(mpu6500_handle_t handle, int16_t *bias_x, int16_t *bias_y, int16_t *bias_z);

/*
 * @brief   Get gyroscope bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_get_gyro_bias(mpu6500_handle_t handle, int16_t *bias_x, int16_t *bias_y, int16_t *bias_z);

/*
 * @brief   Auto calibrate all acceleromter and gyroscope bias value.
 *
 * @param   handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t mpu6500_auto_calib(mpu6500_handle_t handle);


#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */