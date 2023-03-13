/*
 * acelerometro.c
 *
 *  Created on: Mar 6, 2023
 *      Author: mdaef
 */

#include "acelerometro.h"

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
//extern DMA_HandleTypeDef hdma_i2c1_tx;

int16_t  Accel_RAW[3], Gyro_RAW[3];
float Accel_data[3], Gyro_data[3];


void accel_Init (void)
{
	//MPU6050

	uint8_t check;
	uint8_t data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000); 	// WHO_AM_I ~ 6050
	//HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1);

	if (check == 104) // devise is present
	{
		// power management register 0x6B we should write all 0s to wake the sensor up
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
		//HAL_I2C_Mem_Write_DMA(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1);

		// set DATA RATE of 1KHz by writing SMPLRT_DIV register
		data  =  0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
		//HAL_I2C_Mem_Write_DMA(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1);

		// set accelerometer e gyroscopic configuration in ACCEL_CONFIG and GYRO_CONFIG
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
		//HAL_I2C_Mem_Write_DMA(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1);

		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
		//HAL_I2C_Mem_Write_DMA(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1);
	}
}

void read_accel(float *Accel_data)
//void read_accel(void)
{
	uint8_t rec_data[6];

	// Lê 6 BYTES de dados a partir do registrador ACCEL_XOUT_H [ACELERÔMETRO]
	//HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6, 1000);
	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6);

	/* converter os valores RAW em aceleração em 'g'
	   dividir de acordo com o valor Full scale definido em FS_SEL
	   FS_SEL = 0. Então, deve ser dividido por 16384 */

	for (int i = 0; i < 3 ; i++ ){
		Accel_RAW[i] = (int16_t)(rec_data[i*2] << 8 | rec_data [(i*2)+1]);
		Accel_data[i] = (Accel_RAW[i])/16384.0;   // obtém o float g
		//memcpy(&data[i*2], &Accel_data[i], 2);
	}
}

void read_gyro(float *Gyro_data)
{
	uint8_t rec_data2[6];

	// Lê 6 BYTES de dados a partir do registrador GYRO_XOUT_H [GIROSCÓPIO]
	//HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec_data2, 6, 1000);
	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec_data2, 6);
	/* converter os valores RAW em dps (°/s)
	   dividir de acordo com o valor Full scale definido em FS_SEL
	   FS_SEL = 0. Então, deve ser dividido por 131.0  */

	for (int i = 0; i < 3 ; i++ ){
		Gyro_RAW[i] = (int16_t)(rec_data2[i*2] << 8 | rec_data2 [(i*2)+1]);
		Gyro_data[i] = (Gyro_RAW[i])/131.0;
		//memcpy(&data[i*2], &Gyro_data[i], 2);
	}
}
