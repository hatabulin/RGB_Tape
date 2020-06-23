
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "eeprom.h"
#include "Utils.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

osThreadId defaultTaskHandle;
osThreadId effectsTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char str_rx[21];
bool flag_hardbit = true;
bool flag_rgb_state = true;
uint8_t conf_channel = ZERO;
uint8_t flag_blynk = ZERO;
uint8_t red_value,green_value,blue_value;

uint8_t i = ZERO;
uint8_t sec = ZERO;
uint8_t gsec = ZERO;
uint8_t flag_usb;

TrafficLight data ;
RGBColor red,green,yellow;
effectConfig effectCONF;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
static void MX_NVIC_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void WriteDefaultsConstantToEEP(void);
void ReadConstantsFromEEP(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void ReadEEprom(void) {
	uint32_t data_;
	EE_Read(EEP_CHECK_ADDR,&data_);
	if (data_!= CHECK_DATA) {
		EE_Format();
		WriteDefaultsConstantToEEP();
		EE_Write(EEP_CHECK_ADDR,CHECK_DATA);
		ReadConstantsFromEEP();
	} else {
		ReadConstantsFromEEP();
	}
}

void WriteDefaultsConstantToEEP(void) {
	EE_Write(EEP_red, 0xFF0000); //
	EE_Write(EEP_green, 0x00FF00); //
	EE_Write(EEP_yellow, 0xFFFF00); //
}

void WriteValuesToEEprom() {
	uint32_t data_;
	data_ = red.red << 16 | red.green << 8 | red.blue;
	EE_Write(EEP_red, data_);
	data_ = green.red << 16 | green.green << 8 | green.blue;
	EE_Write(EEP_green, data_);
	data_ = yellow.red << 16 | yellow.green << 8 | yellow.blue;
	EE_Write(EEP_yellow, data_);
}

void ReadConstantsFromEEP(void) {
	uint32_t data_;
	EE_Read(EEP_red, &data_);red.red = (data_ >> 16) & 0xFF;red.green = (data_ >> 8) & 0xFF;red.blue = data_ & 0xFF;
	EE_Read(EEP_green, &data_);green.red = data_ >> 16;green.green = data_ >> 8;green.blue = data_ ;
	EE_Read(EEP_yellow, &data_);yellow.red = data_ >> 16;yellow.green = data_ >> 8;yellow.blue = data_ ;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void) {
  /* USER CODE BEGIN 1 */
    uint32_t i;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  for(i=0;i<=1536;i++) {
	  if(i<256) RED=i;
      else if ((i>255)&&(i<512)) RED=512-i;
      else if ((i>511)&&(i<768)) GREEN= i-768;
      else if ((i>767)&&(i<1024)) GREEN=1024-i;
      else if ((i>1023)&&(i<1280)) BLUE=i-1280;
      else if ((i>1279)&&(i<1536)) BLUE=1536-i;
  	  HAL_Delay(0);
  }

  ReadEEprom();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim2.Instance) {
		sec++;
        if (sec>data.freq-1) {
        	sec=0;
        	switch (data.channel) {
        	case 0: { if (RED == ZERO) RED = 255; else RED = ZERO; } break;
  	  	  	case 1: { if (BLUE == ZERO) BLUE = 255; else BLUE = ZERO; } break;
  	  	  	case 2: { if (GREEN == ZERO) GREEN = 255; else GREEN = ZERO; } break;
        	}
  	  //dhcp_n6sec_tick();
        }
	}
}

void ChangeColors(effectConfig effect_config) {
	while ((effect_config.use_channel_red || effect_config.use_channel_green || effect_config.use_channel_blue )
			& (flag_usb > ZERO)) {
		if (effect_config.use_channel_red) {
			  if (RED < effect_config.red_value) RED = RED + 1;
			  else {
			  	  if (RED > effect_config.red_value) RED = RED - 1;
			  	  else effect_config.use_channel_red = false;
			  }
		}
		if (effect_config.use_channel_green) {
			  if (GREEN < effect_config.green_value) GREEN = GREEN + 1;
			  else {
				  if (GREEN > effect_config.green_value) GREEN = GREEN - 1;
				  else effect_config.use_channel_green = false;
			  }
		}
		if (effect_config.use_channel_blue) {
			  if (BLUE < effect_config.blue_value) BLUE = BLUE + 1;
			  else {
				  if (BLUE > effect_config.blue_value) BLUE = BLUE - 1;
				  else effect_config.use_channel_blue = false;
			  }
		}
		osDelay(effect_config.delay_pwm); // 255-data.freq)*4
	}
}

void StartEffectsTask(void const * argument)
{
	for(;;) {
		if (effectCONF.dirty) {
			effectConfig effect_config;
			switch (effectCONF.effectID) {

			case CHANGE_CHANNELS_COLOR_TO:
				ChangeColors(effectCONF);
				effectCONF.dirty = false;
				break;

			case CHANGE_CHANNELS_COLOR_TO_AND_BACK:
				effect_config = effectCONF;

				effect_config.red_value = RED;
				effect_config.green_value = GREEN;
				effect_config.blue_value = BLUE;

				ChangeColors(effectCONF);
				osDelay(effectCONF.delay_change * 40); // 255-data.freq)*4
				ChangeColors(effect_config);
				effectCONF.dirty = false;
				break;
			}
			  //effectCONF.dirty = false;
		  }
		osDelay(1);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  char str[50];
  /* Infinite loop */

  osThreadDef(effectsTask, StartEffectsTask, osPriorityNormal, 0, 256);
  effectsTaskHandle = osThreadCreate(osThread(effectsTask), NULL);

  for(;;) {
	  if (flag_usb == SAVE_CFG) {
		  snprintf(str,strlen(str),"SAVE_CFG - success !\n\r");
		  CDC_Transmit_FS(str, 25);

		  WriteValuesToEEprom();
		  flag_usb = 0;
	  }

	  if (flag_usb == CHANNEL_TUNNING) {
		  switch (conf_channel) {
		  	  case 0:
		  		  red.red = red_value;
		  		  red.green = green_value;
		  		  red.blue = blue_value;
		  		  break;
		  	  case 1:
		  		  yellow.red = red_value;
		  		  yellow.green = green_value;
		  		  yellow.blue = blue_value;
		  		  break;
		  	  case 2:
		  		  green.red = red_value;
		  		  green.green = green_value;
		  		  green.blue = blue_value;
		  		  break;
		  }
		  data.dirty = true;
		  flag_usb = 0;
	  }

	  if (data.mode == NORMAL_MODE) {
		  if (data.dirty) {
			  TIM1->CCR3 = 0;
			  TIM1->CCR2 = 0;
			  TIM1->CCR1 = 0;
			  data.dirty = false;
			  switch (data.channel) {
	  	  	  	  case 0:
	  	  	  		  RED = red.red;
	  	  	  		  GREEN = red.green;
	  	  	  		  BLUE = red.blue;
	  	  	  	  break;
	  	  	  	  case 1:
	  	  	  		  RED = yellow.red;
	  	  	  		  GREEN = yellow.green;
	  	  	  		  BLUE = yellow.blue;
	  	  	  	  break;
	  	  	  	  case 2:
	  	  	  		  RED = green.red;
	  	  	  		  GREEN = green.green;
	  	  	  		  BLUE = green.blue;
	  	  	  	  break;
			  }
		  }
	  } else
		  if ((data.mode == BLINK_MODE)) { // & (data.dirty) ) // if Mode 2 (Blink light)
			  if (data.dirty) {
				  TIM1->CCR3 = 0;
				  TIM1->CCR2 = 0;
				  TIM1->CCR1 = 0;
				  data.dirty = false;
			  }

			  flag_blynk =  1 - flag_blynk ;
			  switch (data.channel) {
  	  	  	  	  case 0:
  	  	  	  		  RED = red.red * flag_blynk;
  	  	  	  		  GREEN = red.green * flag_blynk;
  	  	  	  		  BLUE = red.blue * flag_blynk;
  	  	  	  		  break;
  	  	  	  	  case 1:
  	  	  	  		  RED = yellow.red * flag_blynk;
  	  	  	  		  GREEN = yellow.green * flag_blynk;
  	  	  	  		  BLUE = yellow.blue * flag_blynk;
  	  	  	  		  break;
  	  	  	  	  case 2:
  	  	  	  		  RED = green.red * flag_blynk;
  	  	  	  		  GREEN = green.green * flag_blynk;
  	  	  	  		  BLUE = green.blue * flag_blynk;
  	  	  	  		  break;
			  }

			  HAL_Delay((255-data.freq)*4);
		  } else {
//		  TIM1->CCR3 = 0;
//		  TIM1->CCR2 = 0;
//		  TIM1->CCR1 = 0;
		  }
	  if (flag_hardbit) {
		  snprintf(str,sizeof(str),"%01u.%01u.%03u\n\r",(data.channel),(data.mode),(data.freq));
		  CDC_Transmit_FS(str, 15);
		  flag_hardbit = false;
	  }

	  if (flag_rgb_state ) {
		  switch (conf_channel) {
		  	  case 0:
		  		  red_value = red.red;
		  		  green_value = red.green;
		  		  blue_value = red.blue;
		  		  break;
		  	  case 1:
		  		  red_value = yellow.red;
		  		  green_value = yellow.green;
		  		  blue_value = yellow.blue;
		  		  break;
		  	  case 2:
		  		  red_value = green.red;
		  		  green_value = green.green;
		  		  blue_value = green.blue;
		  		  break;
		  }
		  flag_rgb_state = 0;
		  snprintf(str,sizeof(str),"CH%01u:%02X%02X%02X\n\r",conf_channel,red_value,green_value,blue_value);
		  CDC_Transmit_FS(str, 15);
	  }
	  osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
