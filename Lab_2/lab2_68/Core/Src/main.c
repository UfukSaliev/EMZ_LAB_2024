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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for input_task */
osThreadId_t input_taskHandle;
const osThreadAttr_t input_task_attributes = {
  .name = "input_task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for output_task */
osThreadId_t output_taskHandle;
const osThreadAttr_t output_task_attributes = {
  .name = "output_task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void inputTask(void *argument);
void outputTask(void *argument);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of input_task */
  input_taskHandle = osThreadNew(inputTask, NULL, &input_task_attributes);

  /* creation of output_task */
  output_taskHandle = osThreadNew(outputTask, NULL, &output_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, InB2_Pin|InB1_Pin|InA2_Pin|InA1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : InB2_Pin InB1_Pin InA2_Pin InA1_Pin */
  GPIO_InitStruct.Pin = InB2_Pin|InB1_Pin|InA2_Pin|InA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Button2_Pin Button3_Pin Button4_Pin */
  GPIO_InitStruct.Pin = Button2_Pin|Button3_Pin|Button4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button1_Pin */
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

    uint8_t button1State = 0;
    uint8_t button2State = 0;
    uint8_t button3State = 0;
    uint8_t button4State = 0;

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_inputTask */
/**
* @brief Function implementing the input_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_inputTask */
void inputTask(void *argument)
{
  /* USER CODE BEGIN inputTask */
  /* Infinite loop */
  for(;;)
  {
	  button1State = HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin);
	  button3State = HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin);
	  button2State = HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin);
	  button4State = HAL_GPIO_ReadPin(Button4_GPIO_Port, Button4_Pin);

    osDelay(10);
  }
  /* USER CODE END inputTask */
}

/* USER CODE BEGIN Header_outputTask */
/**
* @brief Function implementing the output_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_outputTask */
void outputTask(void *argument)
{
  /* USER CODE BEGIN outputTask */
  /* Infinite loop */
  for(;;)
  {
      if(button1State == GPIO_PIN_RESET)
      {
    	  HAL_GPIO_WritePin(InB1_GPIO_Port, InB1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InB2_GPIO_Port, InB2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InA1_GPIO_Port, InA1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(InA2_GPIO_Port, InA2_Pin, GPIO_PIN_RESET);
      }
      else if(button2State == GPIO_PIN_RESET)
      {
    	  HAL_GPIO_WritePin(InB1_GPIO_Port, InB1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InB2_GPIO_Port, InB2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InA1_GPIO_Port, InA1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InA2_GPIO_Port, InA2_Pin, GPIO_PIN_SET);
      }
      else if(button3State == GPIO_PIN_RESET)
      {
    	  HAL_GPIO_WritePin(InB1_GPIO_Port, InB1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InB2_GPIO_Port, InB2_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(InA1_GPIO_Port, InA1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InA2_GPIO_Port, InA2_Pin, GPIO_PIN_RESET);
      }
      else if(button4State == GPIO_PIN_RESET)
      {
    	  HAL_GPIO_WritePin(InB1_GPIO_Port, InB1_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(InB2_GPIO_Port, InB2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InA1_GPIO_Port, InA1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InA2_GPIO_Port, InA2_Pin, GPIO_PIN_RESET);
      }
      else
      {
    	  HAL_GPIO_WritePin(InB1_GPIO_Port, InB1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InB2_GPIO_Port, InB2_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InA1_GPIO_Port, InA1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(InA2_GPIO_Port, InA2_Pin, GPIO_PIN_RESET);
      }

	  osDelay(100);
  }
  /* USER CODE END outputTask */
}

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
