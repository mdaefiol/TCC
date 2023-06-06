/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "bmp280.h"
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"
#include "ssd1306.h"
#include "animation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define X_Orig 0
#define Y_Orig 0
#define X_Offset 4
#define W1 21
#define W2 33
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
uint32_t VR[2];

BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity, altitude;
char pressure_str[100], temperature_str[100], altitude_str[100],
		address_str[100];
char inicialization_error[] = "BMP280 Initialization failed";
char reading_error[] = "BMP280 Reading failed";
uint8_t menu_vertical = 0;
uint8_t menu_horizontal = 0;
uint8_t on_off = 0;

bool one_click = true;
bool started_temp_menu = true;
bool has_return_to_middle = false;
bool has_cleaned_var = false;

uint16_t size_temp, size_press, size_alt, size_address;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, VR, 2);

	bmp280_init_user_params(&bmp280.params, BMP280_MODE_NORMAL,
			BMP280_FILTER_OFF, BMP280_STANDARD, BMP280_STANDARD,
			BMP280_STANDARD, BMP280_STANDBY_250);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	ssd1306_Init();
	ssd1306_SetCursor(X_Orig + X_Offset, W1);
	ssd1306_WriteString("Initializing", Font_7x10, White);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
	ssd1306_WriteString(".", Font_7x10, White);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
	ssd1306_WriteString(".", Font_7x10, White);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
	ssd1306_WriteString(".", Font_7x10, White);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		// Escrever mensagem de erro de inicialização no OLED **********************************
		ssd1306_Fill(Black);
		ssd1306_SetCursor(X_Orig, W1);
		ssd1306_WriteString(inicialization_error, Font_7x10, White);
		ssd1306_UpdateScreen();
		HAL_Delay(2000);
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			// Escrever mensagem de erro de medição no OLED **********************************
			ssd1306_Fill(Black);
			ssd1306_SetCursor(X_Orig, W1);
			ssd1306_WriteString(reading_error, Font_7x10, White);
			ssd1306_UpdateScreen();
			HAL_Delay(2000);

		}

		altitude = BMP280_GetAlt(pressure);

		if (on_off == 1) {
			switch (menu_vertical) {
			case 1: // Temperatura
				switch (menu_horizontal) {
				case 1:
					if (!has_cleaned_var) {
						memset(temperature_str, 0, sizeof(temperature_str));
						memset(pressure_str, 0, sizeof(pressure_str));
						memset(altitude_str, 0, sizeof(altitude_str));
						has_cleaned_var = true;
					}
					size_temp = sprintf(temperature_str, "Temp.: %.2f C",
							temperature);
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig, W1);
					ssd1306_WriteString(temperature_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					break;

				case 2:
					if (!has_cleaned_var) {
						memset(temperature_str, 0, sizeof(temperature_str));
						memset(pressure_str, 0, sizeof(pressure_str));
						memset(altitude_str, 0, sizeof(altitude_str));
						has_cleaned_var = true;
					}
					size_temp = sprintf(temperature_str, "Temp.: %.2f F",
							((temperature * ((float) 9 / 5)) + 32));
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig, W1);
					ssd1306_WriteString(temperature_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					break;

				case 3:
					if (!has_cleaned_var) {
						memset(temperature_str, 0, sizeof(temperature_str));
						memset(pressure_str, 0, sizeof(pressure_str));
						memset(altitude_str, 0, sizeof(altitude_str));
						has_cleaned_var = true;
					}
					size_temp = sprintf(temperature_str, "Temp.: %.2f K",
							(temperature + 273.15));
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig, W1);
					ssd1306_WriteString(temperature_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					break;
				}
				break;

			case 2: // Pressão
				switch (menu_horizontal) {
				case 1:
					if (!has_cleaned_var) {
						memset(temperature_str, 0, sizeof(temperature_str));
						memset(pressure_str, 0, sizeof(pressure_str));
						memset(altitude_str, 0, sizeof(altitude_str));
						has_cleaned_var = true;
					}
					size_press = sprintf(pressure_str, "Pres.: %.2f Pa",
							pressure);
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig, W1);
					ssd1306_WriteString(pressure_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					break;

				case 2:
					if (!has_cleaned_var) {
						memset(temperature_str, 0, sizeof(temperature_str));
						memset(pressure_str, 0, sizeof(pressure_str));
						memset(altitude_str, 0, sizeof(altitude_str));
						has_cleaned_var = true;
					}
					size_press = sprintf(pressure_str, "Pres.: %.4f atm",
							(pressure / (float) atmPress));
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig, W1);
					ssd1306_WriteString(pressure_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					break;

				case 3:
					if (!has_cleaned_var) {
						memset(temperature_str, 0, sizeof(temperature_str));
						memset(pressure_str, 0, sizeof(pressure_str));
						memset(altitude_str, 0, sizeof(altitude_str));
						has_cleaned_var = true;
					}
					size_press = sprintf(pressure_str, "Pres.: %.2f Torr",
							(pressure / (float) 133));
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig, W1);
					ssd1306_WriteString(pressure_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					break;
				}
				break;

			case 3: // Altitude
				switch (menu_horizontal) {
				case 1:
					if (!has_cleaned_var) {
						memset(temperature_str, 0, sizeof(temperature_str));
						memset(pressure_str, 0, sizeof(pressure_str));
						memset(altitude_str, 0, sizeof(altitude_str));
						has_cleaned_var = true;
					}
					size_alt = sprintf(altitude_str, "Alt.: %.2f m", altitude);
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig, W1);
					ssd1306_WriteString(altitude_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					break;

				case 2:
					if (!has_cleaned_var) {
						memset(temperature_str, 0, sizeof(temperature_str));
						memset(pressure_str, 0, sizeof(pressure_str));
						memset(altitude_str, 0, sizeof(altitude_str));
						has_cleaned_var = true;
					}
					size_alt = sprintf(altitude_str, "Alt.: %.2f in",
							(altitude * (float) 39.37));
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig, W1);
					ssd1306_WriteString(altitude_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					break;

				case 3:
					if (!has_cleaned_var) {
						memset(temperature_str, 0, sizeof(temperature_str));
						memset(pressure_str, 0, sizeof(pressure_str));
						memset(altitude_str, 0, sizeof(altitude_str));
						has_cleaned_var = true;
					}
					size_alt = sprintf(altitude_str, "Alt.: %.2f ft",
							(altitude * (float) 3.281));
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig, W1);
					ssd1306_WriteString(altitude_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					break;
				}
				break;
			}
		}

		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET) // Botão do joystick (pull-up in SW)
				{
			HAL_GPIO_WritePin(xmin_GPIO_Port, xmin_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(xmax_GPIO_Port, xmax_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ymin_GPIO_Port, ymin_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ymax_GPIO_Port, ymax_Pin, GPIO_PIN_RESET);

			one_click = true;
		}

		else {
			// do nothing
			HAL_GPIO_WritePin(xmin_GPIO_Port, xmin_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(xmax_GPIO_Port, xmax_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ymin_GPIO_Port, ymin_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ymax_GPIO_Port, ymax_Pin, GPIO_PIN_SET);

			if (one_click) {
				on_off++;
				if (on_off > 3) {
					on_off = 0;
				}

				one_click = false;
			}

			switch (on_off) {
			case 0:
				started_temp_menu = true;
				break;

			case 1: // Liga o display
				if (started_temp_menu) {
					// Tela inicialização do display com o endereço I2C ************************************
					ssd1306_Fill(Black);
					ssd1306_SetCursor(X_Orig + X_Offset, W1);
					ssd1306_WriteString("Welcome, user!", Font_7x10, White);
					ssd1306_SetCursor(X_Orig + X_Offset, W2);
					ssd1306_WriteString("Micro II - UEL.", Font_7x10, White);
					ssd1306_DrawRectangle(1, 1, 127, 63, White);
					ssd1306_UpdateScreen();
					HAL_Delay(2000);
					ssd1306_Fill(Black);
					ssd1306_UpdateScreen();

					ssd1306_SetCursor(X_Orig + X_Offset, W1);
					ssd1306_WriteString("BMP280 Interface.", Font_7x10, White);
					ssd1306_SetCursor(X_Orig + X_Offset, W2);
					size_address = sprintf(address_str, "ADDRESS: 0x%02X.",
							bmp280.addr);
					ssd1306_WriteString(address_str, Font_7x10, White);
					ssd1306_UpdateScreen();
					HAL_Delay(2000);
					ssd1306_Fill(Black);
					ssd1306_UpdateScreen();

					menu_vertical = 1;
					menu_horizontal = 1;
					started_temp_menu = false;
				}

				break;

			case 2: // Teste completo do display (dá pra colocar uma animação/imagem própria, usando o drawbitmap)
				// Animação aleatória
				ssd1306_Fill(Black);
				ssd1306_DrawBitmap(0, 0, imag1, 128, 64, White);
				ssd1306_UpdateScreen();
				HAL_Delay(2000);
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();

				ssd1306_DrawBitmap(0, 0, imag2, 128, 64, White);
				ssd1306_UpdateScreen();
				HAL_Delay(2000);
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();

				ssd1306_DrawBitmap(0, 0, imag3, 128, 64, White);
				ssd1306_UpdateScreen();
				HAL_Delay(2000);
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();

				ssd1306_DrawBitmap(0, 0, imag4, 128, 64, White);
				ssd1306_UpdateScreen();
				HAL_Delay(2000);
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();
				break;

			case 3: // Desliga o display
				ssd1306_Fill(Black);
				ssd1306_SetCursor(X_Orig, 22);
				ssd1306_WriteString("See you, user!", Font_7x10, White);
				ssd1306_UpdateScreen();
				HAL_Delay(2000);
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();
				on_off = 0;
				break;
			}
		}

		// VR[0] corresponde ao PA1 (ou seja, nesse caso com o Rx)
		// VR[1] corresponde ao PA2 (ou seja, nesse caso com o Ry)

		// Notei que:
		// Quando joystick está parado (no meio) temos: 2900 < (VR[0] = VR[1]) < 3000
		// Quando vão para seus extremos, ficamos com aproximademente: (VR[0] = VR[1]) < 10 | (VR[0] = VR[1]) > 3000

		// Então, tomando como referencia uma visão frontal,
		// com os pinos na esquerda (mesma visão para ler a silkscreen), temos:

		// Esquerda: VR[0] < 10 | 2900 < VR [1] < 3000
		// Cima: 2900 < VR [0] < 3000 |  VR [1] < 10
		// Direita: VR[0] > 3000 | 2900 < VR [1] < 3000
		// Baixo: 2900 < VR [0] < 3000 | VR[1] > 3000

		if (on_off) {
			if (VR[0] < 20 && 2800 < VR[1] && VR[1] < 3210) // Esquerda
					{
				if (has_return_to_middle) {
					menu_horizontal--;
					if (menu_horizontal < 1) {
						menu_horizontal = 3;
					}

					has_return_to_middle = false;
					has_cleaned_var = false;
				}

				HAL_GPIO_WritePin(xmin_GPIO_Port, xmin_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(xmax_GPIO_Port, xmax_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymin_GPIO_Port, ymin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymax_GPIO_Port, ymax_Pin, GPIO_PIN_RESET);
			}

			else if (2850 < VR[0] && VR[0] < 3210 && VR[1] < 20) // Cima
					{
				if (has_return_to_middle) {
					menu_vertical++;
					menu_horizontal = 1; // vai p/ default
					if (menu_vertical > 3) {
						menu_vertical = 1;
					}

					has_return_to_middle = false;
					has_cleaned_var = false;
				}

				HAL_GPIO_WritePin(xmin_GPIO_Port, xmin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(xmax_GPIO_Port, xmax_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymin_GPIO_Port, ymin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymax_GPIO_Port, ymax_Pin, GPIO_PIN_SET);
			}

			else if (VR[0] > 3125 && 2800 < VR[1] && VR[1] < 3210) // Direita
					{
				if (has_return_to_middle) {
					menu_horizontal++;
					if (menu_horizontal > 3) {
						menu_horizontal = 1;
					}

					has_return_to_middle = false;
					has_cleaned_var = false;
				}

				HAL_GPIO_WritePin(xmin_GPIO_Port, xmin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(xmax_GPIO_Port, xmax_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ymin_GPIO_Port, ymin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymax_GPIO_Port, ymax_Pin, GPIO_PIN_RESET);
			}

			else if (2800 < VR[0] && VR[0] < 3210 && VR[1] > 3100) // Baixo
					{
				if (has_return_to_middle) {
					menu_vertical--;
					menu_horizontal = 1; // vai p/ default
					if (menu_vertical < 1) {
						menu_vertical = 3;
					}

					has_return_to_middle = false;
					has_cleaned_var = false;
				}

				HAL_GPIO_WritePin(xmin_GPIO_Port, xmin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(xmax_GPIO_Port, xmax_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymin_GPIO_Port, ymin_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ymax_GPIO_Port, ymax_Pin, GPIO_PIN_RESET);
			}

			else if (2950 < VR[0] && VR[0] < 3210 && 2950 < VR[1]
					&& VR[1] < 3075) // Meio
							{
				has_return_to_middle = true;

				HAL_GPIO_WritePin(xmin_GPIO_Port, xmin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(xmax_GPIO_Port, xmax_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymin_GPIO_Port, ymin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymax_GPIO_Port, ymax_Pin, GPIO_PIN_RESET);
			}

			else {
				// do nothing
				HAL_GPIO_WritePin(xmin_GPIO_Port, xmin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(xmax_GPIO_Port, xmax_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymin_GPIO_Port, ymin_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ymax_GPIO_Port, ymax_Pin, GPIO_PIN_RESET);
			}
		}

	}
	/*
	 USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, xmin_Pin | ymax_Pin | xmax_Pin | ymin_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PD11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : xmin_Pin ymax_Pin xmax_Pin ymin_Pin */
	GPIO_InitStruct.Pin = xmin_Pin | ymax_Pin | xmax_Pin | ymin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
