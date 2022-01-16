/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "opamp.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
const int16_t maxsize = 1500;
uint16_t database[1505];
uint8_t base[1505];
uint16_t DACVALUE_BASE[] = {0, 0, 0, 1225, 1575, 1900, 2200, 2450, 2600, 2725, 2775, 2725, 2600, 2450, 2200, 1900, 1575, 1225, 0, 0, 0, 0, 0, 1225, 1575, 1900, 2200, 2450, 2600, 2725, 2775, 2725, 2600, 2450, 2200, 1900, 1575, 1225, 0, 0};
uint16_t flagBase[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uint16_t DACVALUE = 0;
uint16_t flag = 0;
// uint32_t prev = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t sum = 0;
  DACVALUE = DACVALUE_BASE[0];
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
  MX_ADC2_Init();
  MX_DAC3_Init();
  MX_OPAMP3_Init();
  MX_ADC1_Init();
  MX_OPAMP1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(DIAG1_GPIO_Port, DIAG1_Pin, GPIO_PIN_SET);
  HAL_DAC_Start(&hdac3, DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DACVALUE);
  HAL_OPAMP_Start(&hopamp3);

  HAL_GPIO_WritePin(DIAG2_GPIO_Port, DIAG2_Pin, GPIO_PIN_SET);
  HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DACVALUE);
  HAL_OPAMP_Start(&hopamp1);

  // prev = HAL_GetTick();
  HAL_GPIO_WritePin(BRAKE1_GPIO_Port, BRAKE1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BRAKE2_GPIO_Port, BRAKE2_Pin, GPIO_PIN_SET);

  printf("%d\n", DACVALUE);
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  flag = 1;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    DACVALUE = DACVALUE_BASE[sum];
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DACVALUE);
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DACVALUE);
    HAL_GPIO_WritePin(FWD_REV1_GPIO_Port, FWD_REV1_Pin, flagBase[sum]);
    sum++;
    if (sum == 40)
      sum = 0;
    HAL_Delay(1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV64;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
   */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(flag ==0) return ;
  static uint16_t fre = 0;
  static uint16_t sum = 0;
  if ((GPIO_Pin == GPIO_PIN_12) || (GPIO_Pin == GPIO_PIN_11) || (GPIO_Pin == GPIO_PIN_13))
  {
    // 统计该配置下的dac输出与相应转速
    // fre++;
    // if (fre == maxsize)
    // {
    //   uint32_t now = HAL_GetTick();
    // 	float cal = (float)200 / ((float)(now-prev))*1000;
    //   database[sum++] = cal ;
    //   DACVALUE += 25;
    // 	if(DACVALUE %200 == 0)
    // 		printf("%d is done\n",DACVALUE);
    //   if (DACVALUE > 3000)
    //   {
    //     HAL_GPIO_WritePin(BRAKE1_GPIO_Port, BRAKE1_Pin, GPIO_PIN_RESET);
    //     for (int i =0;i<120;i++)
    //     {
    //       printf("%d\t%.0f\n",i*25,database[i]);
    //       HAL_Delay(50);
    //     }
    //     return;
    //   }
    //   HAL_GPIO_WritePin(BRAKE1_GPIO_Port, BRAKE1_Pin, GPIO_PIN_RESET);
    //   HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DACVALUE);
    //   HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DACVALUE);
    //   HAL_GPIO_WritePin(BRAKE1_GPIO_Port, BRAKE1_Pin, GPIO_PIN_SET);
    //   prev = HAL_GetTick();
    //   fre = 0;
    // }

    if (fre < maxsize)
    {
      GPIO_PinState a = HAL_GPIO_ReadPin(FWD_REV1_GPIO_Port, FWD_REV1_Pin);
      base[fre] = (a == GPIO_PIN_RESET) ? 0 : 1;
      database[fre] = HAL_GetTick();
    }
    if (fre == maxsize)
    {
      printf("TIME1 %d\n", HAL_GetTick());
      HAL_GPIO_WritePin(BRAKE1_GPIO_Port, BRAKE1_Pin, GPIO_PIN_RESET);
      for (int16_t i = 0; i < maxsize; i++)
      {
        printf("%d\t%d\t%d\n", i, base[i], database[i]);
        HAL_Delay(5);
      }
    }
    fre++;
    // sum--;
    // if (sum == 0)
    // {
    //   sum = 6;
    //   HAL_GPIO_TogglePin(FWD_REV1_GPIO_Port, FWD_REV1_Pin);
    // }
  }
  if (GPIO_Pin == GPIO_PIN_0)
  {
    fre++;
    if (fre == 100)
    {
      fre = 0;
      printf("TIME2 %d\n", HAL_GetTick());
    }
  }
  return;
}

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

#ifdef USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
