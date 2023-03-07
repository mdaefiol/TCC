/*
 * acelerometro.c
 *
 *  Created on: Mar 6, 2023
 *      Author: mdaef
 */

#include "acelerometro.h"

extern I2C_HandleTypeDef hi2c1;

int16_t  Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
float Ax, Ay, Az;
int16_t  Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
float Gx, Gy, Gz;


void accel_Init (void)
{
	//MPU6050

	uint8_t check;
	uint8_t data;

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000); 	// WHO_AM_I ~ 6050

	if (check == 104) // devise is present
	{
		// power management register 0x6B we should write all 0s to wake the sensor up
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

		// set DATA RATE of 1KHz by writing SMPLRT_DIV register
		data  =  0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

		// set accelerometer e gyroscopic configuration in ACCEL_CONFIG and GYRO_CONFIG
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}
}

void read_accel ()
//void read_accel(void)
{
	uint8_t rec_data[6];

	// Lê 6 BYTES de dados a partir do registrador ACCEL_XOUT_H [ACELERÔMETRO]

	HAL_I2C_Mem_Read  (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6, 1000);

	Accel_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data [1]);
	Accel_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data [3]);
	Accel_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data [5]);
	/* converter os valores RAW em aceleração em 'g'
	   dividir de acordo com o valor Full scale definido em FS_SEL
	   FS_SEL = 0. Então, deve ser dividido por 16384 */

	Ax = Accel_X_RAW/16384.0;   // obtém o float g
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;

}

void read_gyro ()
{
	uint8_t rec_data[6];

	// Lê 6 BYTES de dados a partir do registrador GYRO_XOUT_H [GIROSCÓPIO]
	HAL_I2C_Mem_Read  (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec_data, 6, 1000);

	Gyro_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data [1]);
	Gyro_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data [3]);
	Gyro_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data [5]);

	/* converter os valores RAW em dps (°/s)
	   dividir de acordo com o valor Full scale definido em FS_SEL
	   FS_SEL = 0. Então, deve ser dividido por 131.0  */

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;

}


