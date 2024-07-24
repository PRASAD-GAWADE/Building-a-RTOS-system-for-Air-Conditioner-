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
#include "stdio.h"
#include "string.h"

#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int i=0,manual=0;

float voltage_1;
float voltage;
int16_t ADC_Val;
float temp_LM35;
int pseudo_i=0;
int irq_fault=0;

int FanSpeed;
int A;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

osThreadId Task_1Handle;
osThreadId Task_2Handle;
osThreadId Task_3Handle;
osSemaphoreId BinSemHandle;
/* USER CODE BEGIN PV */
void FanSpeedContrl(float);
void SpeedDataOnLCD(int);
void TempDataOnLCD(float);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
void Temp_Sensing(void const * argument);
void IndooreFanSpeedContrl(void const * argument);
void ComprsorFanSpeedContrl(void const * argument);

/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) // Check if EXTI line 0 triggered the interrupt
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0); // Clear the interrupt flag

        // Toggle the variable i
        if (i == 0)
        {
            i++;       //Switch to Manual Mode
        }
        else
        {
            i--;       //Switch to Auto  Mode
            osSemaphoreRelease(BinSemHandle);
            pseudo_i=0;
        }
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	 if(GPIO_Pin==GPIO_PIN_1)
	 {
		 temp_LM35++;
		 FanSpeedContrl(temp_LM35);
		 pseudo_i++;
	 }

	 else if(GPIO_Pin==GPIO_PIN_2)
	 {
		 temp_LM35--;
		 FanSpeedContrl(temp_LM35);
		 pseudo_i--;

	 }

      else
	      irq_fault++;

 }



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float temp_convrt(int16_t ADC_Val)
{
	uint16_t val= ADC_Val;
	float voltage = ((val * (3.3/4095))/0.01); //for 12 Bit ADC

	TempDataOnLCD(voltage);

	return voltage;
}

float ADC_Setting(void)
{
	    //int16_t ADC_Val;
	    HAL_ADC_Start(&hadc1);
	    HAL_ADC_PollForConversion(&hadc1, 100);
	    ADC_Val=HAL_ADC_GetValue(&hadc1);
	    HAL_ADC_Stop(&hadc1);
	    return ADC_Val;

}

/*void Send_UART(float voltage_1)
{
	 char buffer[7]; // Adjust the size according to your needs
	 sprintf(buffer, "%.2f \n", (double)voltage_1);// Convert float to string with 2 decimal places
	  // Transmit the string over UART
	 HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}*/


void FanSpeedContrl(float temp_LM35)
{
	int16_t duty_cycle [6]  = {20, 35, 50, 70, 80, 95} ;

    int speed=0;

    if(temp_LM35<=28)
    {
    	speed = duty_cycle [0];
    	FanSpeed=6;
    }
    else if(temp_LM35<=30)
    {
    	speed=duty_cycle [1];
    	FanSpeed=7;
    }
    else if(temp_LM35<=32)
        {
        	speed=duty_cycle [2];
        	FanSpeed=8;
        }
    else if(temp_LM35<=34)
        {
        	speed=duty_cycle [3];
        	FanSpeed=9;
        }
    else if(temp_LM35<=36)
        {
        	speed=duty_cycle [4];
        	FanSpeed=10;
        }
    else
        {
        	speed=duty_cycle [5];
        	FanSpeed=11;
        }

    TIM1->CCR1=speed;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    SpeedDataOnLCD(FanSpeed);
}

void Manual_Run(void)
{

	if(pseudo_i==0)
	{
	temp_LM35=40;
	}

	FanSpeedContrl(temp_LM35);

    	//SpeedDataOnLCD(temp_LM35);
	manual++;
}

void SpeedDataOnLCD(int A)
{
    HD44780_SetCursor(A,1);       ////setting courser at 1st row and 0th column
    HD44780_Blink();              //blink whole matrix

}


void TempDataOnLCD(float voltage_11)
{
	    char tempStr[6];  // Allocate a buffer to store the temperature string

	    // Convert the temperature float value to a string
	    sprintf(tempStr, "%.2f", voltage_11);  // "%.2f" formats the float with 2 decimal places

	    // Print the temperature string on the LCD
	    HD44780_SetCursor(6,0);
	    HD44780_PrintStr(tempStr);
}

void ParmanentDataOnLCD(void)
{
    HD44780_Clear();
    HD44780_SetCursor(0,0);       ////setting courser at 0st column and 0th row
    HD44780_PrintStr("TEMP :");
    HD44780_SetCursor(0,1);       ////setting courser at 0st column and 1th row
    HD44780_PrintStr("SPEED:");
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HD44780_Init(2);
      HD44780_Clear();
      HD44780_SetCursor(0,0);       ////setting courser at 1st row and 0th column
      HD44780_PrintStr("AC STARTING...");
      HAL_Delay(2000);
      HD44780_NoBacklight();
      HAL_Delay(500);
      HD44780_Backlight();

      ParmanentDataOnLCD();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinSem */
  osSemaphoreDef(BinSem);
  BinSemHandle = osSemaphoreCreate(osSemaphore(BinSem), 1);

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
  /* definition and creation of Task_1 */
  osThreadDef(Task_1, Temp_Sensing, osPriorityNormal, 0, 256);
  Task_1Handle = osThreadCreate(osThread(Task_1), NULL);

  /* definition and creation of Task_2 */
  osThreadDef(Task_2, IndooreFanSpeedContrl, osPriorityNormal, 0, 128);
  Task_2Handle = osThreadCreate(osThread(Task_2), NULL);

  /* definition and creation of Task_3 */
  osThreadDef(Task_3, ComprsorFanSpeedContrl, osPriorityIdle, 0, 128);
  Task_3Handle = osThreadCreate(osThread(Task_3), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Temp_Sensing */
/**
  * @brief  Function implementing the Task_1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Temp_Sensing */
void Temp_Sensing(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(i==0)
	{
		//auto code
		//k++;
		ADC_Val=ADC_Setting();
	    voltage_1=temp_convrt(ADC_Val);
	   // Send_UART(voltage_1);


	}
	else
	{
		//Manual code
		osSemaphoreWait(BinSemHandle, osWaitForever);
		//l++;
		Manual_Run();
	}

osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_IndooreFanSpeedContrl */
/**
* @brief Function implementing the Task_2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IndooreFanSpeedContrl */
void IndooreFanSpeedContrl(void const * argument)
{
  /* USER CODE BEGIN IndooreFanSpeedContrl */
  /* Infinite loop */
  for(;;)
  {  osSemaphoreWait(BinSemHandle, osWaitForever);
	  FanSpeedContrl((float)voltage_1 );
	  osSemaphoreRelease(BinSemHandle);
  osDelay(50);
  }
  /* USER CODE END IndooreFanSpeedContrl */
}

/* USER CODE BEGIN Header_ComprsorFanSpeedContrl */
/**
* @brief Function implementing the Task_3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ComprsorFanSpeedContrl */
void ComprsorFanSpeedContrl(void const * argument)
{
  /* USER CODE BEGIN ComprsorFanSpeedContrl */
  /* Infinite loop */
  for(;;)
  {
	  FanSpeedContrl((float)voltage_1 );
	 	  	  osSemaphoreRelease(BinSemHandle);
  }
  /* USER CODE END ComprsorFanSpeedContrl */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
