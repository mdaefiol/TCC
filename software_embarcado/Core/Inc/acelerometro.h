/*
 * acelerometro.h
 *
 *  Created on: Mar 6, 2023
 *      Author: mdaef
 */

#ifndef INC_ACELEROMETRO_H_
#define INC_ACELEROMETRO_H_

#include "stm32f1xx_hal.h"

// ACELERÔMETRO
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

//void read_accel (int *array_x, int *array_y, int *array_z);
void accel_Init (void);
void read_accel ();
void read_gyro ();

#endif /* INC_ACELEROMETRO_H_ */
