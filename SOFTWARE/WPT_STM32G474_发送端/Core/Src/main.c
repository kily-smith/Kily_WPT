/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "fdcan.h"
#include "hrtim.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "fonts.h"
#include "stdio.h"
#include "ina226.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void start_module_A(void)
//{



//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//start_module_A();
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
  MX_ADC1_Init();
  MX_FDCAN3_Init();
  MX_HRTIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
		HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);	//务必用|链接，实现同步
 
	
	 HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 );
	 HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA2 );
	
	
	 HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 );
	 HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB2 );
//	HAL_Delay (100);
//	ST7735_Init();
//	HAL_GPIO_WritePin (SPI1_BL_GPIO_Port ,SPI1_BL_Pin ,GPIO_PIN_SET );
//	ST7735_FillScreen(ST7735_BLACK);
//	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		
//		float    Current;
//		float   Voltage;
//		float  Pow;	
//		
//	//INA226 初始化及CAL值适配
//	/*
//	* 设置转换时间8.244ms,求平均值次数16，设置模式为分流和总线连续模式
//	* 总数据转换时间 = 8.244*16 = 131.9ms 
//	*/
//    INA226_SetConfig(0x45FF);
//	/*
//	* 分流电阻最大电压 :电阻0.01R，分辨率0.2mA
//	* 公式1
//	* Current_LSB = 预= 32768 * 0.0000025V = 0.08192V
//	* 设置分流电压转电流转换参数期最大电流 / 2^15
//	* Current_LSB = 5 / 32768 = 0.00152A ,选0.002ma
//	* 公式2
//	* CAL = 0.00512/(Current_LSB*R)
//	* CAL = 0.00512/(0.0002*0.1)=256 = 0x0100			100m对应0xa00  1对应0xa00
//	*/
//    INA226_SetCalibrationReg(0x07D0);
//		
//	//数据获取及转换
//	Voltage = INA226_GetBusV();
//	Current = INA226_GetCurrent();
//	Pow = INA226_GetPower();
//	char str_voltage[10]; // 分配足够的空间存储字符串
//  sprintf(str_voltage, "%.6f", Voltage ); // 将浮点数格式化为整数形式 
//	char str_current[10]; // 分配足够的空间存储字符串       `
//  sprintf(str_current, "%.6f", Current ); // 将浮点数格式化为整数形式 
//	char str_pow[10]; // 分配足够的空间存储字符串
//  sprintf(str_pow, "%.6f", Pow ); // 将浮点数格式化为整数形式 
////	//串口打印
////	printf("Current is %fA ",Current);
////	printf("Voltage is %fV ",Voltage);
////	printf("Pow is %fW\n",Pow);
//	//屏幕显示
//	ST7735_DrawString(0, 0, "[WPT-TX]", ST7735_YELLOW, ST7735_BLACK, &Font_11x18);
//	ST7735_DrawString(100, 0, "CAN:NOT", ST7735_RED, ST7735_BLACK, &Font_7x10);
//	ST7735_DrawString(100, 10, "UART:NOT", ST7735_RED, ST7735_BLACK, &Font_7x10);
//	ST7735_DrawString(0, 20, str_voltage, ST7735_GREEN, ST7735_BLACK, &Font_11x18);
//	ST7735_DrawString(0, 40, str_current, ST7735_BLUE, ST7735_BLACK, &Font_11x18);
//	ST7735_DrawString(0, 60, str_pow, ST7735_YELLOW, ST7735_BLACK, &Font_11x18);
//	ST7735_DrawString(100, 20, "POWER:", ST7735_PUPER, ST7735_BLACK, &Font_7x10);
//	ST7735_DrawString(100, 30, "2000J", ST7735_WHITE, ST7735_BLACK, &Font_7x10);
//	ST7735_DrawString(100, 40, "TIME", ST7735_CYAN, ST7735_BLACK, &Font_7x10);
//	ST7735_DrawString(100, 50, "0:00:00", ST7735_WHITE, ST7735_BLACK, &Font_7x10);
//	ST7735_DrawString(100, 60, "29DU", ST7735_BLUE, ST7735_BLACK, &Font_7x10);
//	ST7735_DrawString(100, 70, "------", ST7735_YELLOW, ST7735_BLACK, &Font_7x10);
//	
//	//指示灯显示
	HAL_GPIO_WritePin (LED_GPIO_Port ,LED_Pin ,GPIO_PIN_RESET);
	HAL_Delay (100);
	HAL_GPIO_WritePin (LED_GPIO_Port ,LED_Pin ,GPIO_PIN_SET);		
	HAL_Delay (100);

	HAL_GPIO_WritePin (RGB_G_GPIO_Port ,RGB_G_Pin ,GPIO_PIN_RESET);
	HAL_Delay (100);
	HAL_GPIO_WritePin (RGB_G_GPIO_Port ,RGB_G_Pin ,GPIO_PIN_SET);		
	HAL_Delay (100);
	HAL_GPIO_WritePin (RGB_B_GPIO_Port ,RGB_B_Pin ,GPIO_PIN_RESET);
	HAL_Delay (100);
	HAL_GPIO_WritePin (RGB_B_GPIO_Port ,RGB_B_Pin ,GPIO_PIN_SET);		
	HAL_Delay (100);
	HAL_GPIO_WritePin (RGB_R_GPIO_Port ,RGB_R_Pin ,GPIO_PIN_RESET);
	HAL_Delay (100);
	HAL_GPIO_WritePin (RGB_R_GPIO_Port ,RGB_R_Pin ,GPIO_PIN_SET);		
	HAL_Delay (100);
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
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
  while (1)
  {
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
