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
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	USART2->DR = (uint8_t) ch;
	while (!LL_USART_IsActiveFlag_TXE(USART2));
	return ch;
}

#include "stdio.h"
#include "math.h"
#include <mpu6500.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265f
#define G_MPS2 9.81f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct Attitude angles = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void toggle_led_and_wait()
{
	if ((GPIOC->IDR & (1 << 13)))
		GPIOC->BSRR = (uint32_t) (1 << (13 + 16));
	else
		GPIOC->BSRR = (uint32_t) (1 << 13);

	LL_mDelay(10);
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

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	printf("Program is starting! \r\n");

	struct MPU_Handle mpu;
	MPU_Init(&mpu, I2C1);
	MPU_SetAccelFS(&mpu, MPU6500_ACCEL_FS_2G);
	MPU_SetGyroFS(&mpu, MPU6500_GYRO_FS_250DPS);
	LL_mDelay(100);

	MPU_Calibrate(&mpu);

	uint32_t timestamp;

	while (1) {
		toggle_led_and_wait();
//		print_mpu_all_regs(&mpu);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		timestamp = HAL_GetTick();
		MPU_GetSensorData(&mpu);

		// calculate pitch and roll from accel
		angles.theta_a = atan2(mpu.a.x, sqrt(mpu.a.y * mpu.a.y + mpu.a.z * mpu.a.z));
		angles.phi_a = atan2(mpu.a.y, sqrt(mpu.a.x * mpu.a.x + mpu.a.z * mpu.a.z));

		// calculate pitch and roll from gyro
		angles.theta_dot = -(mpu.g.y * cos(angles.phi) - mpu.g.z * sin(angles.phi));
		angles.phi_dot = -(mpu.g.x + mpu.g.y * sin(angles.phi) * tan(angles.theta) + mpu.g.z * cos(angles.phi) * tan(angles.theta));

		angles.theta_g = angles.theta + ((float) (HAL_GetTick() - timestamp) / 1000.0f) * angles.theta_dot;
		angles.phi_g = angles.phi + ((float) (HAL_GetTick() - timestamp) / 1000.0f) * angles.phi_dot;

		timestamp = HAL_GetTick();

		// complementary filter
		float alpha = 0.02;
		angles.theta = angles.theta_a * alpha + (1 - alpha) * angles.theta_g;
		angles.phi = angles.phi_a * alpha + (1 - alpha) * angles.phi_g;

		printf("%f,%f,%f,%f\r\n",
			angles.theta * 180 / PI,
			angles.theta_dot * 180 / PI,
			angles.theta_a * 180 / PI,
			angles.theta_g * 180 / PI
		);
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 150;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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
