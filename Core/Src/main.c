/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "WHW_IRQN.h"
#include "All_Init.h"
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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t _init_step = 0;  /* 调试用：记录初始化到哪一步 */
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
  _init_step = 1;
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  _init_step = 2;
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* 调试模式：保持所有定时器在 CPU halt 时继续运行，确保中断能触发 */
  _init_step = 3;
  DBGMCU->APB1FZ = 0x00001800;  /* 仅冻结 IWDG(bit12)+WWDG(bit11)，TIM2~TIM7 保持运行 */
  DBGMCU->APB2FZ = 0x00000000;  /* TIM1,TIM8~TIM11 全部保持运行 */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  _init_step = 10; MX_GPIO_Init();
  _init_step = 11; MX_DMA_Init();
  _init_step = 12; MX_CAN1_Init();
  _init_step = 13; MX_CAN2_Init();
  _init_step = 14; MX_SPI1_Init();
  _init_step = 15; MX_USART1_UART_Init();
  _init_step = 16; MX_USART6_UART_Init();
  _init_step = 17; MX_CRC_Init();
  _init_step = 18; MX_TIM10_Init();
  _init_step = 19; MX_USART3_UART_Init();
  _init_step = 20; MX_TIM4_Init();
  _init_step = 21; MX_ADC1_Init();
  _init_step = 22; MX_ADC3_Init();
  _init_step = 23; MX_TIM7_Init();
  _init_step = 24; MX_SPI2_Init();
  _init_step = 25; MX_USART2_UART_Init();
  _init_step = 26; MX_TIM1_Init();
  _init_step = 27; MX_TIM3_Init();
  _init_step = 28; MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  /* 在调度器启动前开启 TIM3/TIM7/TIM9，避免被高优先级阻塞任务卡住导致定时器不启动 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim9);

  /* 直接操作寄存器兜底：确保 TIM9 时钟+计数器+中断一定开启 */
  __HAL_RCC_TIM9_CLK_ENABLE();  /* 强制开时钟 */
  TIM9->CR1  |= TIM_CR1_CEN;    /* 使能计数器 */
  TIM9->DIER |= TIM_DIER_UIE;   /* 使能更新中断 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  * 使用内部 HSI（16MHz）替代 HSE，避免外部晶振起振失败
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;    /* HSI=16MHz / 8 = 2MHz VCO输入 */
  RCC_OscInitStruct.PLL.PLLN = 168;  /* 2MHz * 168 = 336MHz VCO输出 */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  /* 336/2 = 168MHz SYSCLK */
  RCC_OscInitStruct.PLL.PLLQ = 7;    /* 336/7 = 48MHz (USB) */
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */


  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */

  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* 调试期间不关中断，检查 _init_step 变量确定失败位置 */
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
