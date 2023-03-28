/*
 * acelerometro.c
 *
 *  Created on: Mar 6, 2023
 *      Author: mdaef
 */

#include "acelerometro.h"

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;

HAL_DMA_StateTypeDef status1;
HAL_DMA_CallbackIDTypeDef ret1;
HAL_DMA_LevelCompleteTypeDef x1;
HAL_DMA_LevelCompleteTypeDef x2;
//extern DMA_HandleTypeDef hdma_i2c1_tx;

uint16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW, Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
AccelData acc_data;
GyroData gy_data;

void MPU6050_Config(void)
{
	//MPU6050
	uint8_t check;
	uint8_t data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000); 	// WHO_AM_I ~ 6050

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
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}
}

void read_accel(void)
//void read_accel(void)
{
	uint8_t rec_data[6];
	//ret1 = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6);
	// Lê 6 BYTES de dados a partir do registrador ACCEL_XOUT_H [ACELERÔMETRO]
	status1 = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6);

	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6);
	x1 = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY); // Espera a transferência DMA ser completada
	x2 = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6);
	ret1 = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6);



	/* converter os valores RAW em aceleração em 'g'
	   dividir de acordo com o valor Full scale definido em FS_SEL
	   FS_SEL = 0. Então, deve ser dividido por 16384 */

	Accel_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data [1]);
	Accel_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data [3]);
	Accel_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data [5]);

	acc_data.Ax = Accel_X_RAW/16384.0;  // get the float g
	acc_data.Ay = Accel_Y_RAW/16384.0;
	acc_data.Az = Accel_Z_RAW/16384.0;
	//ret3 = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6);
}

void read_gyro(void)
{
	uint8_t rec_data[6];

	// Lê 6 BYTES de dados a partir do registrador GYRO_XOUT_H [GIROSCÓPIO]
	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec_data, 6);
	//ret3 = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec_data, 6);

	/* converter os valores RAW em dps (°/s)
	   dividir de acordo com o valor Full scale definido em FS_SEL
	   FS_SEL = 0. Então, deve ser dividido por 131.0  */

	Gyro_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data[1]);
	Gyro_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data[3]);
	Gyro_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data[5]);

	gy_data.Gx = Gyro_X_RAW/131.0;
	gy_data.Gy = Gyro_Y_RAW/131.0;
	gy_data.Gz = Gyro_Z_RAW/131.0;
}
