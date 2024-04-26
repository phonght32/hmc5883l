#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "hmc5883l.h"

#define HMC5883L_REG_CONFIG_A         	0x00
#define HMC5883L_REG_CONFIG_B         	0x01
#define HMC5883L_REG_MODE             	0x02
#define HMC5883L_REG_OUT_X_M          	0x03
#define HMC5883L_REG_OUT_X_L          	0x04
#define HMC5883L_REG_OUT_Z_M          	0x05
#define HMC5883L_REG_OUT_Z_L          	0x06
#define HMC5883L_REG_OUT_Y_M          	0x07
#define HMC5883L_REG_OUT_Y_L          	0x08
#define HMC5883L_REG_STATUS           	0x09
#define HMC5883L_REG_IDENT_A          	0x0A
#define HMC5883L_REG_IDENT_B          	0x0B
#define HMC5883L_REG_IDENT_C          	0x0C

typedef struct hmc5883l {
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
	float 						mag_scaling_factor;			/*!< Magnetometer scaling factor */
} hmc5883l_t;

hmc5883l_handle_t hmc5883l_init(void)
{
	hmc5883l_handle_t handle = calloc(1, sizeof(hmc5883l_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

err_code_t hmc5883l_set_config(hmc5883l_handle_t handle, hmc5883l_cfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	float mag_scaling_factor = 0;

	switch (config.range)
	{
	case HMC5883L_RANGE_0_88GA:
		mag_scaling_factor = 0.073f;
		break;

	case HMC5883L_RANGE_1_3GA:
		mag_scaling_factor = 0.92f;
		break;

	case HMC5883L_RANGE_1_9GA:
		mag_scaling_factor = 1.22f;
		break;

	case HMC5883L_RANGE_2_5GA:
		mag_scaling_factor = 1.52f;
		break;

	case HMC5883L_RANGE_4_0GA:
		mag_scaling_factor = 2.27f;
		break;

	case HMC5883L_RANGE_4_7GA:
		mag_scaling_factor = 2.56f;
		break;

	case HMC5883L_RANGE_5_6GA:
		mag_scaling_factor = 3.03f;
		break;

	case HMC5883L_RANGE_8_1GA:
		mag_scaling_factor = 4.35f;
		break;

	default:
		break;
	}

	handle->range = config.range;
	handle->opr_mode = config.opr_mode;
	handle->data_rate = config.data_rate;
	handle->samples = config.samples;
	handle->mag_bias_x = config.mag_bias_x;
	handle->mag_bias_y = config.mag_bias_y;
	handle->mag_bias_z = config.mag_bias_z;
	handle->i2c_send = config.i2c_send;
	handle->i2c_recv = config.i2c_recv;
	handle->delay = config.delay;
	handle->mag_scaling_factor = mag_scaling_factor;

	return ERR_CODE_SUCCESS;
}

err_code_t hmc5883l_config(hmc5883l_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t cmd_data = 0;

	/* Set range */
	cmd_data = handle->range << 5;
	handle->i2c_send(HMC5883L_REG_CONFIG_B, &cmd_data, 1);

	/* Set operating mode */
	handle->i2c_recv(HMC5883L_REG_MODE, &cmd_data, 1);
	cmd_data &= 0b11111100;
	cmd_data |= handle->opr_mode;
	handle->i2c_send(HMC5883L_REG_MODE, &cmd_data, 1);

	/* Set data rate */
	handle->i2c_recv(HMC5883L_REG_CONFIG_A, &cmd_data, 1);
	cmd_data &= 0b11100011;
	cmd_data |= (handle->data_rate << 2);
	handle->i2c_send(HMC5883L_REG_CONFIG_A, &cmd_data, 1);

	/* Set samples */
	handle->i2c_recv(HMC5883L_REG_CONFIG_A, &cmd_data, 1);
	cmd_data &= 0b10011111;
	cmd_data |= (handle->samples << 5);
	handle->i2c_send(HMC5883L_REG_CONFIG_A, &cmd_data, 1);

	return ERR_CODE_SUCCESS;
}

err_code_t hmc5883l_get_mag_raw(hmc5883l_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t mag_raw_data[6];
	handle->i2c_recv(HMC5883L_REG_OUT_X_M, mag_raw_data, 6);

	*raw_x = (int16_t)((int16_t)(mag_raw_data[0] << 8) | mag_raw_data[1]);
	*raw_y = (int16_t)((int16_t)(mag_raw_data[2] << 8) | mag_raw_data[3]);
	*raw_z = (int16_t)((int16_t)(mag_raw_data[4] << 8) | mag_raw_data[5]);

	return ERR_CODE_SUCCESS;
}

err_code_t hmc5883l_get_mag_calib(hmc5883l_handle_t handle, int16_t *calib_x, int16_t *calib_y, int16_t *calib_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t mag_raw_data[6];
	int16_t raw_x = 0, raw_y = 0, raw_z = 0;

	handle->i2c_recv(HMC5883L_REG_OUT_X_M, mag_raw_data, 6);

	raw_x = (int16_t)((int16_t)(mag_raw_data[0] << 8) | mag_raw_data[1]);
	raw_y = (int16_t)((int16_t)(mag_raw_data[2] << 8) | mag_raw_data[3]);
	raw_z = (int16_t)((int16_t)(mag_raw_data[4] << 8) | mag_raw_data[5]);

	*calib_x = raw_x - handle->mag_bias_x;
	*calib_y = raw_y - handle->mag_bias_y;
	*calib_z = raw_z - handle->mag_bias_z;

	return ERR_CODE_SUCCESS;
}

err_code_t hmc5883l_get_mag_scale(hmc5883l_handle_t handle, float *scale_x, float *scale_y, float *scale_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t mag_raw_data[6];
	int16_t raw_x = 0, raw_y = 0, raw_z = 0;

	handle->i2c_recv(HMC5883L_REG_OUT_X_M, mag_raw_data, 6);

	raw_x = (int16_t)((int16_t)(mag_raw_data[0] << 8) | mag_raw_data[1]);
	raw_y = (int16_t)((int16_t)(mag_raw_data[2] << 8) | mag_raw_data[3]);
	raw_z = (int16_t)((int16_t)(mag_raw_data[4] << 8) | mag_raw_data[5]);

	*scale_x = (raw_x - handle->mag_bias_x) * handle->mag_scaling_factor;
	*scale_y = (raw_y - handle->mag_bias_y) * handle->mag_scaling_factor;
	*scale_z = (raw_z - handle->mag_bias_z) * handle->mag_scaling_factor;

	return ERR_CODE_SUCCESS;
}

err_code_t hmc5883l_set_mag_bias(hmc5883l_handle_t handle, int16_t bias_x, int16_t bias_y, int16_t bias_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->mag_bias_x = bias_x;
	handle->mag_bias_y = bias_y;
	handle->mag_bias_z = bias_z;

	return ERR_CODE_SUCCESS;
}

err_code_t hmc5883l_get_mag_bias(hmc5883l_handle_t handle, int16_t *bias_x, int16_t *bias_y, int16_t *bias_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	*bias_x = handle->mag_bias_x;
	*bias_y = handle->mag_bias_y;
	*bias_z = handle->mag_bias_z;

	return ERR_CODE_SUCCESS;
}
