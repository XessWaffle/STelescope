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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "status.h"
#include "compass.h"
#include "accelerometer.h"
#include "display.h"
#include "mobility.h"
#include "camera.h"
#include "rtc.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define USBD_CDC_BUFF_LEN 128
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t tim2_tick_sec;

uint32_t tim3_counter_max;

uint8_t usbd_buff[USBD_CDC_BUFF_LEN];
uint32_t usbd_buff_len;

uint32_t rate;

uint8_t capture_flag = FALSE;
uint8_t sensor_flag = FALSE;

orientation_packet_s orientation;
camera_data_packet_s cam_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void populate_orientation_packet(void);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  if(init_status_leds() != TRUE)
    Error_Handler();

  set_status_led(STATUS_PWR, 0x000001);
  update_leds();
  HAL_Delay(2000);

  set_status_led(STATUS_PWR, 0x010000);
  set_status_led(STATUS_MODE, 0x000100);
  set_status_led(STATUS_SENS, 0x000001);
  update_leds();

  if(init_display() != TRUE)
    Error_Handler();

  if(init_camera() != TRUE)
	  Error_Handler();

  set_status_led(STATUS_SENS, 0x000101);
  update_leds();

  if(init_compass() != TRUE)
	  Error_Handler();

  set_status_led(STATUS_SENS, 0x000202);
  update_leds();

  if(init_accelerometer() != TRUE)
	  Error_Handler();

  set_status_led(STATUS_SENS, 0x000303);
  update_leds();

  if(init_rtc() != TRUE)
	  Error_Handler();

  set_status_led(STATUS_SENS, 0x010000);
  update_leds();


  if(init_stepper() != TRUE)
    Error_Handler();

  set_status_led(STATUS_STAT, 0x010000);
  update_leds();

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /*home(YAW);
  home(PITCH);
  home(ROLL);*/

  rate = 60000;

  HAL_Delay(1000);
  set_stepper_state(TRACKING);

  usbd_buff_len = snprintf((char*) usbd_buff, USBD_CDC_BUFF_LEN, "Homing Complete, Ready to Track Objects\r\n");  
  CDC_Transmit_FS(usbd_buff, usbd_buff_len);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*HAL_Delay(40000);
	  HAL_GPIO_TogglePin(ST_DIR_Y_GPIO_Port, ST_DIR_Y_Pin);
	  HAL_GPIO_TogglePin(ST_DIR_R_GPIO_Port, ST_DIR_R_Pin);
	  HAL_GPIO_TogglePin(ST_DIR_P_GPIO_Port, ST_DIR_P_Pin);*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*dacq_accelerometer();
    dacq_compass();
    HAL_Delay(100);

    usbd_buff_len = snprintf(usbd_buff, USBD_CDC_BUFF_LEN, "%d|%d|%d|%lu\r\n", accel->out_x_a_raw, accel->out_y_a_raw, accel->out_z_a_raw, get_stepper_position(PITCH));  
    CDC_Transmit_FS(usbd_buff, usbd_buff_len);

    HAL_Delay(400);*/

    sensor_update();
    camera_update();

    telescope_command_s *t_cmd;
    if((t_cmd = get_next_command()) == NULL)
      continue;

    switch (t_cmd->cmd) {
      case CMD_R_PLUS: /* Move the rotation axis with a positive rate */
        set_stepper_rate(ROLL, rate);
        break;
      case CMD_R_MINUS: /* Move the rotation axis with a negative rate */
        set_stepper_rate(ROLL, -rate);
        break;
      case CMD_R_STOP: /* Stop the rotation axis */
        set_stepper_rate(ROLL, 0);
        break;
      case CMD_Y_PLUS: /* etc etc... */
        set_stepper_rate(YAW, rate);
        break;
      case CMD_Y_MINUS:
        set_stepper_rate(YAW, -rate);
        break;                   
      case CMD_Y_STOP:
        set_stepper_rate(YAW, 0);
        break;    
      case CMD_P_PLUS:
        set_stepper_rate(PITCH, rate);
        break;
      case CMD_P_MINUS:
        set_stepper_rate(PITCH, -rate);
        break;    
      case CMD_P_STOP:
        set_stepper_rate(PITCH, 0);
        break;
      case CMD_HOME:
        HAL_TIM_Base_Stop_IT(&htim4);
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
        home(YAW);
        home(PITCH);
        home(ROLL);
        HAL_TIM_Base_Start_IT(&htim4);
        HAL_NVIC_EnableIRQ(EXTI2_IRQn);
        break;
      case CMD_MICST_MODE: /* Change the microstep mode (mode given in info)*/
        set_microstep_mode(t_cmd->info);
        break;
      case CMD_CAPTURE:
        break;
      case CMD_INVALID:
      default:
        break;
    }
    
    /*usbd_buff_len = snprintf((char*) usbd_buff, USBD_CDC_BUFF_LEN, "Received CMD:%d, INFO:%d\r\n", t_cmd->cmd, t_cmd->info);  
    CDC_Transmit_FS(usbd_buff, usbd_buff_len);*/
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 23;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  tim2_tick_sec = SystemCoreClock / (htim2.Init.Prescaler + 1) / (htim2.Init.Period + 1);

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 14;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 359;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ST_MS3_Pin|ST_MS2_Pin|ST_MS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ST_DIR_R_Pin|ST_STEP_R_Pin|ST_DIR_Y_Pin|ST_STEP_Y_Pin
                          |DISP_CS1_Pin|ST_SLP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IND_CTRL_Pin|CAM_CS_Pin|DISP_RST_Pin|DISP_DC_Pin
                          |ST_DIR_P_Pin|ST_STEP_P_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ST_MS3_Pin ST_MS2_Pin ST_MS1_Pin */
  GPIO_InitStruct.Pin = ST_MS3_Pin|ST_MS2_Pin|ST_MS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ST_DIR_R_Pin ST_STEP_R_Pin ST_DIR_Y_Pin ST_STEP_Y_Pin
                           DISP_CS1_Pin ST_SLP_Pin */
  GPIO_InitStruct.Pin = ST_DIR_R_Pin|ST_STEP_R_Pin|ST_DIR_Y_Pin|ST_STEP_Y_Pin
                          |DISP_CS1_Pin|ST_SLP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IND_CTRL_Pin CAM_CS_Pin DISP_RST_Pin DISP_DC_Pin
                           ST_DIR_P_Pin ST_STEP_P_Pin */
  GPIO_InitStruct.Pin = IND_CTRL_Pin|CAM_CS_Pin|DISP_RST_Pin|DISP_DC_Pin
                          |ST_DIR_P_Pin|ST_STEP_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RTC_1HZ_Pin */
  GPIO_InitStruct.Pin = RTC_1HZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RTC_1HZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MDL_DRDY_Pin MDL_INT_Pin */
  GPIO_InitStruct.Pin = MDL_DRDY_Pin|MDL_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * Define callbacks for I2C and SPI
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	handle_e handle = hi2c == &hi2c1 ? I2C_1 : I2C_2;
	rtx_i2c_cb(handle);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	handle_e handle = hi2c == &hi2c1 ? I2C_1 : I2C_2;
	rtx_i2c_cb(handle);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	handle_e handle = hspi == &hspi1 ? SPI_1 : SPI_2;
	rtx_spi_cb(handle);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	handle_e handle = hspi == &hspi1 ? SPI_1 : SPI_2;
	rtx_spi_cb(handle);
}

/*
* Interrupt and main loop handlers for data acquisition and stepper motor control
*/

void step()
{

}

void led_update()
{

}

void sensor_update()
{
  if(sensor_flag == TRUE)
  {
    sensor_flag = FALSE;
    dacq_compass(ONBOARD);
    dacq_compass(OFFBOARD);
    dacq_accelerometer();

    populate_orientation_packet();

    CDC_Transmit_FS((uint8_t*) orientation.raw_data, sizeof(orientation_packet_s));
  }
}

void camera_update()
{

  static uint16_t packet = 0;

  capture_step_e step_id = capture_and_pipe(capture_flag, cam_data.buff, CAMERA_PIPE_BUFFER_LENGTH);
  if(capture_flag == TRUE)
    capture_flag = FALSE;

  uint8_t send = (step_id == CAPTURE_DATA) || (step_id == CAPTURE_END);

  cam_data.packet_id = CAMERA_PACKET_ID;
  cam_data.size = CAMERA_PIPE_BUFFER_LENGTH;
  cam_data.last_packet = packet;

  if(step_id == CAPTURE_DATA)
    packet++;
  else if(step_id == CAPTURE_END)
    packet = 0;

  if(send)
  {
    CDC_Transmit_FS((uint8_t*) cam_data.raw_data, sizeof(camera_data_packet_s));
  }
}

void step_isr()
{
  for(uint8_t i = 0; i < AXES; i++)
  {
    if(increment_tick((axes_e) i) == TRUE)
      step_axis((axes_e) i);
  }
}

void led_update_isr()
{
    static uint8_t period = 0;
    static status_led_iterator_s data;

    if(period == 0)
      data = get_next_pulse_cc(FALSE);

    if(data.buff_pos < (WS2812B_WRITE_BITS * STATUS_LED_NUM) && data.ccr_val != 0)
    {
      if (period == 0)
      {
        HAL_GPIO_WritePin(IND_CTRL_GPIO_Port, IND_CTRL_Pin, GPIO_PIN_RESET);
      } 
      else if(period == data.ccr_val)
      {
        HAL_GPIO_WritePin(IND_CTRL_GPIO_Port, IND_CTRL_Pin, GPIO_PIN_SET);
      }
    }
    else
    {
      HAL_GPIO_WritePin(IND_CTRL_GPIO_Port, IND_CTRL_Pin, GPIO_PIN_RESET);
    }
      
    if(data.buff_pos >= (WS2812B_WRITE_BITS * STATUS_LED_NUM + 50))
      HAL_TIM_Base_Stop_IT(&htim3);
    
    period++;
    period %= 3;
    
}

void sensor_update_isr()
{
  sensor_flag = TRUE;
}

void camera_update_isr()
{
  capture_flag = TRUE;
}

void populate_orientation_packet()
{
  accelerometer_s *accel = get_accelerometer_data();
  compass_s *offboard_compass = get_compass_data(OFFBOARD);
  compass_s *onboard_compass = get_compass_data(ONBOARD);
  
  /* Packet ID */
  orientation.packet_id = ORIENTATION_PACKET_ID;

  /* Accelerometer */
  orientation.out_x_g_raw = accel->out_x_g_raw;
  orientation.out_y_g_raw = accel->out_y_g_raw;
  orientation.out_z_g_raw = accel->out_z_g_raw;
  orientation.out_x_a_raw = accel->out_x_a_raw;
  orientation.out_y_a_raw = accel->out_y_a_raw;
  orientation.out_z_a_raw = accel->out_z_a_raw;

  /* Offboard and Onboard compass*/
  orientation.out_x_raw_offboard = offboard_compass->out_x_raw;
  orientation.out_y_raw_offboard = offboard_compass->out_y_raw;
  orientation.out_z_raw_offboard = offboard_compass->out_z_raw;
  orientation.out_x_raw_onboard = onboard_compass->out_x_raw;
  orientation.out_y_raw_onboard = onboard_compass->out_y_raw;
  orientation.out_z_raw_onboard = onboard_compass->out_z_raw;

  /* Stepper position */
  orientation.yaw_position = get_stepper_position(YAW);
  orientation.roll_position = get_stepper_position(ROLL);
  orientation.pitch_position = get_stepper_position(PITCH);
  orientation.yaw_rate = get_stepper_rate(YAW);
  orientation.roll_rate = get_stepper_rate(ROLL);
  orientation.pitch_rate = get_stepper_rate(PITCH);
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
