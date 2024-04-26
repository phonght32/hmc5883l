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

#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

#define HMC5883L_I2C_ADDR		(0x1E)

typedef err_code_t (*hmc5883l_func_i2c_send)(uint8_t reg_addr, uint8_t *buf_send, uint16_t len);
typedef err_code_t (*hmc5883l_func_i2c_recv)(uint8_t reg_addr, uint8_t *buf_recv, uint16_t len);
typedef void (*hmc5883l_func_delay)(uint32_t ms);

/**
 * @brief   Handle structure.
 */
typedef struct hmc5883l *hmc5883l_handle_t;

/**
 * @brief   Sensor field range.
 */
typedef enum {
	HMC5883L_RANGE_0_88GA = 0,								/*!< +/- 0.88 Ga. Gain 1370 LSB/Gauss */
	HMC5883L_RANGE_1_3GA,									/*!< +/- 1.3 Ga. Gain 1090 LSB/Gauss */
	HMC5883L_RANGE_1_9GA,									/*!< +/- 1.9 Ga. Gain 820 LSB/Gauss */
	HMC5883L_RANGE_2_5GA,									/*!< +/- 2.5 Ga. Gain 660 LSB/Gauss */
	HMC5883L_RANGE_4_0GA,									/*!< +/- 4.0 Ga. Gain 440 LSB/Gauss */
	HMC5883L_RANGE_4_7GA,									/*!< +/- 4.7 Ga. Gain 390 LSB/Gauss */
	HMC5883L_RANGE_5_6GA,									/*!< +/- 5.6 Ga. Gain 330 LSB/Gauss */
	HMC5883L_RANGE_8_1GA									/*!< +/- 8.1 Ga. Gain 230 LSB/Gauss */
} hmc5883l_range_t;

/**
 * @brief   Operating mode.
 */
typedef enum {
	HMC5883L_OPR_MODE_CONTINUOUS = 0,						/*!< Continuous measurement */
	HMC5883L_OPR_MODE_SINGLE,								/*!< Single measurement */
	HMC5883L_OPR_MODE_IDLE									/*!< Idle */
} hmc5883l_opr_mode_t;

/**
 * @brief   Data output rate.
 */
typedef enum {
	HMC5883L_DATA_RATE_0_75HZ = 0,							/*!< 0.75 Hz */
	HMC5883L_DATA_RATE_1_5HZ,								/*!< 1.5 Hz */
	HMC5883L_DATA_RATE_3HZ,									/*!< 3 Hz */
	HMC5883L_DATA_RATE_7_5HZ,								/*!< 7.5 Hz */
	HMC5883L_DATA_RATE_15HZ,								/*!< 15 Hz */
	HMC5883L_DATA_RATE_30HZ,								/*!< 30 Hz */
	HMC5883L_DATA_RATE_75HZ,								/*!< 75 Hz */
} hmc5883l_data_rate_t;

/**
 * @brief   Number of samples averaged per measurement output.
 */
typedef enum {
	HMC5883L_SAMPLE_1 = 0,									/*!< 1 sample */
	HMC5883L_SAMPLE_2,										/*!< 2 samples */
	HMC5883L_SAMPLE_4,										/*!< 4 samples */
	HMC5883L_SAMPLE_8,										/*!< 8 samples */
} hmc5883l_sample_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	hmc5883l_range_t  			range;						/*!< Sensor field range */
	hmc5883l_opr_mode_t  		opr_mode;  					/*!< Operating mode */
	hmc5883l_data_rate_t  		data_rate;					/*!< Data output rate */
	hmc5883l_sample_t  			samples;					/*!< Number of samples averaged */
	int 						mag_bias_x;  				/*!< Magnetometer bias x axis */
	int 						mag_bias_y;  				/*!< Magnetometer bias y axis */
	int 						mag_bias_z;  				/*!< Magnetometer bias z axis */
	hmc5883l_func_i2c_send      i2c_send;        			/*!< HMC5883L send bytes */
	hmc5883l_func_i2c_recv      i2c_recv;         			/*!< HMC5883L receive bytes */
	hmc5883l_func_delay         delay;                 		/*!< HMC5883L delay function */
} hmc5883l_cfg_t;

/*
 * @brief   Initialize HMC5883L with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
hmc5883l_handle_t hmc5883l_init(void);

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
err_code_t hmc5883l_set_config(hmc5883l_handle_t handle, hmc5883l_cfg_t config);

/*
 * @brief   Configure HMC5883L to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hmc5883l_config(hmc5883l_handle_t handle);

/*
 * @brief   Get magnetometer raw value.
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
err_code_t hmc5883l_get_mag_raw(hmc5883l_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);

/*
 * @brief   Get magnetometer calibrated data.
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
err_code_t hmc5883l_get_mag_calib(hmc5883l_handle_t handle, int16_t *calib_x, int16_t *calib_y, int16_t *calib_z);

/*
 * @brief   Get magnetometer scaled data.
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
err_code_t hmc5883l_get_mag_scale(hmc5883l_handle_t handle, float *scale_x, float *scale_y, float *scale_z);

/*
 * @brief   Set magnetometer bias data.
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
err_code_t hmc5883l_set_mag_bias(hmc5883l_handle_t handle, int16_t bias_x, int16_t bias_y, int16_t bias_z);

/*
 * @brief   Set magnetometer bias data.
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
err_code_t hmc5883l_get_mag_bias(hmc5883l_handle_t handle, int16_t *bias_x, int16_t *bias_y, int16_t *bias_z);

#ifdef __cplusplus
}
#endif

#endif /* __HMC5883L_H__ */