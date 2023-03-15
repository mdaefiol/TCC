/*
 * barometro.c
 *
 *  Created on: 6 de mar de 2023
 *      Author: mdaef
 */

#include "barometro.h"

extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;

#define BMP280_I2C &hi2c2
#define BMP280_ADD 0xEC //como o SDIO está no terra, o endereço c/ 7bits é 0x76, mas 0x76<<1= 0xEC

BMP280_data bmp_data;

uint16_t dig_T1,dig_P1;
uint16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t chipID;
uint8_t TrimParam[36];
int32_t tRaw, pRaw;

float pressureSeaLevel = 101325;

// leitura de corte, sao dados armazenados na memoria do sensor (ja vem de fabrica) e precisam ser utilizados para calculos dos novos valores
void dataRead(void)
{
	uint8_t trimdata[32];
	// Read NVM from 0x88 to 0xA1
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADD, 0x88, 1, trimdata, 25, 1000);

	dig_T1 = (trimdata[1]<<8) | trimdata[0];
	dig_T2 = (trimdata[3]<<8) | trimdata[2];
	dig_T3 = (trimdata[5]<<8) | trimdata[4];
	dig_P1 = (trimdata[7]<<8) | trimdata[5];
	dig_P2 = (trimdata[9]<<8) | trimdata[6];
	dig_P3 = (trimdata[11]<<8) | trimdata[10];
	dig_P4 = (trimdata[13]<<8) | trimdata[12];
	dig_P5 = (trimdata[15]<<8) | trimdata[14];
	dig_P6 = (trimdata[17]<<8) | trimdata[16];
	dig_P7 = (trimdata[19]<<8) | trimdata[18];
	dig_P8 = (trimdata[21]<<8) | trimdata[20];
	dig_P9 = (trimdata[23]<<8) | trimdata[22];

}

int BMPReadRaw(void)
{
	uint8_t Data[8];

	// Check the chip ID before reading
	HAL_I2C_Mem_Read(&hi2c2, BMP280_ADD, ID_REG, 1, &chipID, 1, 1000);

	if (chipID == 0x58) //“id” register contains the chip identification number chip_id[7:0], which is 0x58
	{
		// Read the Registers 0xF7 to 0xFC
		HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADD, PRESS_MSB_REG, 1, Data, 6, 1000);

		/* Calculate the Raw data for the parameters
		 * Here the Pressure and Temperature are in 20 bit format and humidity in 16 bit format
		 */
		pRaw = (Data[0]<<12)|(Data[1]<<4)|(Data[2]>>4);
		tRaw = (Data[3]<<12)|(Data[4]<<4)|(Data[5]>>4);

		return 0;
	}

	else return -1;
}

int32_t t_fine;
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3)-((int32_t)dig_T1<<1)))*((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4)-((int32_t)dig_T1))*((adc_T>>4)-((int32_t)dig_T1))) >> 12)*((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}


int32_t bmp280_compensate_P_int32(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine)-128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 +((var1*(int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >>8) + ((var1 * (int64_t)dig_P2) <<12);
	var1 = (((((int64_t)1) <<47)+var1))*((int64_t)dig_P1) >>33;

	if (var1 == 0)
			{
				return 0; // avoid exception caused by division by zero
			}

	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (int64_t)p;

}


int BMP280_Config (uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
	// Read the Trimming parameters
	dataRead();

	uint8_t datatowrite = 0;
	uint8_t datacheck = 0;

	// Reset the device
	datatowrite = 0xB6;  // reset sequence
	if (HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADD, RESET_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
	{
		return -1;
	}

	HAL_Delay (100);


	// write the standby time and IIR filter coeff to 0xF5
	datatowrite = (t_sb <<5) |(filter << 2);
	if (HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADD, CONFIG_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
	{
		return -1;
	}
	HAL_Delay (100);
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADD, CONFIG_REG, 1, &datacheck, 1, 1000);
	if (datacheck != datatowrite)
	{
		return -1;
	}


	// write the pressure and temp oversampling along with mode to 0xF4
	datatowrite = (osrs_t <<5) |(osrs_p << 2) | mode;
	if (HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADD, CTRL_MEAS_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
	{
		return -1;
	}
	HAL_Delay (100);
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADD, CTRL_MEAS_REG, 1, &datacheck, 1, 1000);
	if (datacheck != datatowrite)
	{
		return -1;
	}

	return 0;
}

/* To be used when doing the force measurement
 * the Device need to be put in forced mode every time the measurement is needed
 */
void BMP280_WakeUP(void)
{
	uint8_t datatowrite = 0;

	// first read the register
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADD, CTRL_MEAS_REG, 1, &datatowrite, 1, 1000);

	// modify the data with the forced mode
	datatowrite = datatowrite | MODE_FORCED;

	// write the new data to the register
	HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADD, CTRL_MEAS_REG, 1, &datatowrite, 1, 1000);

	HAL_Delay (100);
}

/* measure the temp, pressure
 * the values will be stored in the parameters passed to the function
 */

void BMP280_Measure(void)
{
	const int32_t INVALID_RAW_VALUE = 0x800000;

	if (BMPReadRaw() == 0)
	{
		  if (tRaw != INVALID_RAW_VALUE) {
			  bmp_data.temperature = (bmp280_compensate_T_int32 (tRaw))/100.0;  // temp x100
		  }
		  else bmp_data.temperature = 1; // value in case temp measurement was disabled

		  if (pRaw != INVALID_RAW_VALUE) {
			  bmp_data.pressure = (bmp280_compensate_P_int32 (pRaw))/-256;  //  Pa
		  }
		  else
		  {
			  bmp_data.pressure = 1; // value in case temp measurement was disabled
		  }
	}

	else
	{
		bmp_data.temperature = bmp_data.pressure = 1;
	}
	float bmp_pressure = bmp_data.pressure;

	bmp_data.altitude = (float)(44330 * (1 - (pow((bmp_pressure/ pressureSeaLevel), (1/5.255)))));
}

// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
