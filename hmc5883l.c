#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "hmc5883l.h"

typedef struct hmc5883l {
	hmc5883l_range_t  			range;						/*!< Sensor field range */
	hmc5883l_opr_mode_t  		opr_mode;  					/*!< Operating mode */
	hmc5883l_data_rate_t  		data_rate;					/*!< Data output rate */
	hmc5883l_sample_t  			sample;						/*!< Number of samples averaged */
	hmc5883l_func_i2c_send      i2c_send;        			/*!< HMC5883L send bytes */
	hmc5883l_func_i2c_recv      i2c_recv;         			/*!< HMC5883L receive bytes */
	hmc5883l_func_delay         delay;                 		/*!< HMC5883L delay function */
} hmc5883l_t;