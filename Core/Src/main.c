/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"

// Sensor libraries
#include "mpu6500.h"
#include "bmp280.h"
#include "IIRFilter.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	USART2->DR = (uint8_t) ch;
	while (!LL_USART_IsActiveFlag_TXE(USART2))
		;
	return ch;
}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265f
#define G_MPS2 9.81f

#define LED_TOGGLE_RATE_HZ 5
#define MPU_SAMPLE_RATE_HZ 500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct Attitude angles = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void toggle_led()
{
	if ((GPIOC->IDR & (1 << 13)))
		GPIOC->BSRR = (uint32_t) (1 << (13 + 16));
	else
		GPIOC->BSRR = (uint32_t) (1 << 13);
}

void print_mpu_all_regs(struct MPU_Handle *mpu)
{
	uint8_t data;

	printf("START === \r\n");
	for (int i = 0; i < 127; i++) {
		MPU_ReadReg(mpu, i, &data, 1);
		printf("register %x: %x\r\n", i, data);
	}
	printf("STOP ==== \r\n");
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
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
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	LL_mDelay(1000); // wait some time for the sensors to stabilize

	// Setting up MPU and its filters
	struct MPU_Handle mpu;
	MPU_Init(&mpu, I2C1);
	MPU_SetAccelFS(&mpu, MPU6500_ACCEL_FS_2G);
	MPU_SetGyroFS(&mpu, MPU6500_GYRO_FS_250DPS);
	LL_mDelay(100);
	MPU_Calibrate(&mpu);

	struct IIRFilter theta_filter;
	struct IIRFilter phi_filter;
	IIR_Init(&theta_filter, 1.735, -0.766, 0.008, 0.016, 0.008); // 15Hz
	IIR_Init(&phi_filter, 1.735, -0.766, 0.008, 0.016, 0.008); // 15 Hz

	// Setting up BMP280
	struct bmp280_dev bmp;
	struct bmp280_config conf;
	struct bmp280_uncomp_data ucomp_data;
	double pres;

	bmp.dev_id = BMP280_I2C_ADDR_PRIM;
	bmp.intf = BMP280_I2C_INTF;
	bmp280_init(&bmp, I2C1);

	bmp280_get_config(&conf, &bmp);

	conf.filter = BMP280_FILTER_COEFF_2;
	conf.odr = BMP280_ODR_1000_MS;
	bmp280_set_config(&conf, &bmp);
	bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	printf("Program is starting! \r\n");

	uint32_t timerLED = HAL_GetTick();
	uint32_t timerMPU = HAL_GetTick();
	uint32_t timerBMP = HAL_GetTick();

	while (1) {
		if (HAL_GetTick() - timerLED > 200) {
			toggle_led();
			timerLED = HAL_GetTick();
		}

		if (HAL_GetTick() - timerMPU > 1000 / MPU_SAMPLE_RATE_HZ) {
			MPU_GetSensorData(&mpu);

			// calculate pitch and roll from accel
			angles.theta_a = atan2(mpu.a.x, sqrt(mpu.a.y * mpu.a.y + mpu.a.z * mpu.a.z));
			angles.phi_a = atan2(mpu.a.y, sqrt(mpu.a.x * mpu.a.x + mpu.a.z * mpu.a.z));

			// filter accel theta calculation
			angles.theta_a = IIR_Update(&theta_filter, angles.theta_a);
			angles.phi_a = IIR_Update(&phi_filter, angles.phi_a);

			// convert bodyframe gyro rates into euler frame gyro rate
			angles.theta_dot = -(mpu.g.y * cos(angles.phi) - mpu.g.z * sin(angles.phi));
			angles.phi_dot = -(mpu.g.x + mpu.g.y * sin(angles.phi) * tan(angles.theta) + mpu.g.z * cos(angles.phi) * tan(angles.theta));

			// calculate angle from gyro by integrating
			angles.theta_g = angles.theta + ((float) (HAL_GetTick() - timerMPU) / 1000.0f) * angles.theta_dot;
			angles.phi_g = angles.phi + ((float) (HAL_GetTick() - timerMPU) / 1000.0f) * angles.phi_dot;

			// complementary filter
			float alpha = 0.05;
			angles.theta = angles.theta_a * alpha + (1 - alpha) * angles.theta_g;
			angles.phi = angles.phi_a * alpha + (1 - alpha) * angles.phi_g;

//			printf("%f,%f,%f,%f\r\n", angles.theta * 180 / PI, angles.phi * 180 / PI, angles.theta_a * 180 / PI, angles.phi_a * 180 / PI);

			timerMPU = HAL_GetTick();
		}

		if (HAL_GetTick()-timerBMP > 1000) {
			bmp280_get_uncomp_data(&ucomp_data, &bmp);
			bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
			printf("%f\r\n", pres);

			timerBMP = HAL_GetTick();
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
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
