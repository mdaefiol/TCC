/*
 * barometro.h
 *
 *  Created on: 6 de mar de 2023
 *      Author: mdaef
 */

#ifndef INC_BAROMETRO_H_
#define INC_BAROMETRO_H_

#include "stm32f1xx_hal.h"

int BMP280_Config (uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter);
void dataRead(void);
void BMP280_WakeUP(void);
void BMP280_Measure(void);
//void BMP280_Measure(float *array, float *array_p);

// Oversampling definitions
#define OSRS_OFF    	0x00
#define OSRS_1      	0x01
#define OSRS_2      	0x02
#define OSRS_4      	0x03
#define OSRS_8      	0x04
#define OSRS_16     	0x05

// MODE Definitions
#define MODE_SLEEP      0x00
#define MODE_FORCED     0x01
#define MODE_NORMAL     0x03

// Standby Time
#define T_SB_0p5    	0x00
#define T_SB_62p5   	0x01
#define T_SB_125    	0x02
#define T_SB_250    	0x03
#define T_SB_500    	0x04
#define T_SB_1000   	0x05
#define T_SB_2000     	0x06
#define T_SB_4000     	0x07

// IIR Filter Coefficients ??
#define IIR_OFF     	0x00
#define IIR_2       	0x01
#define IIR_4       	0x02
#define IIR_8       	0x03
#define IIR_16      	0x04


// REGISTERS DEFINITIONS
#define ID_REG      	0xD0	//0x58
#define RESET_REG  		0xE0	//0xB6
#define STATUS_REG      0xF3
#define CTRL_MEAS_REG   0xF4
#define CONFIG_REG      0xF5
#define PRESS_MSB_REG   0xF7

#endif /* INC_BAROMETRO_H_ */
