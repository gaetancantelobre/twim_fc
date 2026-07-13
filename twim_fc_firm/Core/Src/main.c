/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "bmi270_config.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "crsf.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
#define UART_RX_BUF_SIZE 256
uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
uint16_t uart_rx_head = 0;
crsf_t elrs;
uint32_t last_print_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void JumpToDFU(void);
void LPS22DF_Write(uint8_t reg, uint8_t data);
uint8_t LPS22DF_Read(uint8_t reg);
void LPS22DF_ReadBlock(uint8_t reg, uint8_t *data, uint16_t len);
void BMI270_Write(uint8_t reg, uint8_t data);
uint8_t BMI270_Read(uint8_t reg);
void BMI270_ReadBlock(uint8_t reg, uint8_t *data, uint16_t len);
void BMI270_LoadConfig(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t boot_to_dfu = 0;

void JumpToDFU(void) {
    // Write a magic value to end of SRAM2 (survives a soft reset)
    *((volatile uint32_t *)0x200057F0) = 0xDEADBEEF;
    
    // Perform a true hardware reset of the microcontroller
    NVIC_SystemReset();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Catch the Magic RAM flag immediately after reset, before ANY clocks or peripherals are initialized
  if (*((volatile uint32_t *)0x200057F0) == 0xDEADBEEF) {
      *((volatile uint32_t *)0x200057F0) = 0; // Clear the flag
      
      // Remap System Memory to address 0x00000000
      __HAL_RCC_SYSCFG_CLK_ENABLE();
      __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
      
      // Setup jump to ROM bootloader
      void (*SysMemBootJump)(void) = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
      
      // Set the main stack pointer
      __set_MSP(*(uint32_t *) 0x1FFF0000);
      
      // Jump!
      SysMemBootJump();
      while(1);
  }
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
  MX_USB_Device_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  crsf_init(&elrs);
  HAL_UART_Receive_DMA(&huart1, uart_rx_buf, UART_RX_BUF_SIZE);
  // Start the PWM channels
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Start PA9 (TIM1 CH2)

  // Initialize LPS22HB (Baro)
  uint8_t lps_id = LPS22DF_Read(0x0F); // WHO_AM_I (LPS22HB returns 0xB1)
  LPS22DF_Write(0x10, 0x20); // CTRL_REG1: Power on, ODR=10Hz

  // Initialize BMI270 (IMU)
  BMI270_Read(0x00); // Dummy read to wake up SPI
  HAL_Delay(5);
  uint8_t bmi_id = BMI270_Read(0x00); // CHIP_ID (Expect 0x24)
  BMI270_Write(0x7C, 0x00); // PWR_CONF: Disable advanced power save
  HAL_Delay(1);
  BMI270_LoadConfig();
  uint8_t bmi_stat = BMI270_Read(0x21); // INTERNAL_STATUS (Expect 0x01 = init_ok)
  BMI270_Write(0x7D, 0x04); // PWR_CTRL: acc_en=1
  HAL_Delay(10); // Wait for accelerometer to power up
  BMI270_Write(0x40, 0xA8); // ACC_CONF: ODR=100Hz, Normal mode

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (boot_to_dfu) {
        JumpToDFU();
    }

    // Process any new bytes received by DMA
    uint16_t tail = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    while (uart_rx_head != tail) {
        crsf_parse_byte(&elrs, uart_rx_buf[uart_rx_head]);
        uart_rx_head = (uart_rx_head + 1) % UART_RX_BUF_SIZE;
    }

    // Print parsed CRSF data to USB CDC periodically if new data is available
    if (elrs.new_data && (HAL_GetTick() - last_print_time >= 50)) {
        elrs.new_data = false;
        last_print_time = HAL_GetTick();
        
        // Update Servo PWM output on PA0 (TIM2 CH1) from Channel 1
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, elrs.channels[0]);
        
        // Update Servo PWM output on PA9 (assumed TIM1 CH2) from Channel 2
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, elrs.channels[1]);
        
        // Update ESC PWM output on PA1 (TIM2 CH2) from Channel 3
        // Map receiver range [180, 1800] to standard ESC pulse [1000us, 2000us]
        int32_t ch3_val = elrs.channels[2];
        if (ch3_val < 180) ch3_val = 180;
        if (ch3_val > 1800) ch3_val = 1800;
        uint32_t esc_pulse = 1000 + ((ch3_val - 180) * 1000) / (1800 - 180);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, esc_pulse);

        // Toggle MOSFET on PB11 based on Channel 6 (Aux 2)
        if (elrs.channels[5] > 1500) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
        }

        // Read LPS22DF Data
        uint8_t press_data[3];
        LPS22DF_ReadBlock(0x28, press_data, 3);
        int32_t pressure = (press_data[2] << 16) | (press_data[1] << 8) | press_data[0];
        if (pressure & 0x800000) pressure |= 0xFF000000; // Sign extension
        // Convert to hPa: raw / 4096 = hPa
        int32_t press_int = pressure / 4096;
        int32_t press_frac = ((pressure % 4096) * 100) / 4096;
        if (press_frac < 0) press_frac = -press_frac;

        // Read BMI270 Data
        uint8_t acc_data[6];
        BMI270_ReadBlock(0x0C, acc_data, 6);
        int16_t ax = (acc_data[1] << 8) | acc_data[0];
        int16_t ay = (acc_data[3] << 8) | acc_data[2];
        int16_t az = (acc_data[5] << 8) | acc_data[4];

        char msg[512];
        int len = snprintf(msg, sizeof(msg), 
            "CH1:%4u CH2:%4u CH3:%4u CH6:%4u | Baro:%ld.%02ld hPa | BMI_STAT:%02X AX:%6d AY:%6d AZ:%6d\r\n", 
            elrs.channels[0], elrs.channels[1], elrs.channels[2], elrs.channels[5],
            press_int, press_frac, bmi_stat, ax, ay, az);

        CDC_Transmit_FS((uint8_t*)msg, len);
    }
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 420000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS High by default

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // CS High by default
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // MOSFET Low by default

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// LPS22DF Helpers (SPI3, CS=PB9)
void LPS22DF_Write(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), data};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx, 2, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

uint8_t LPS22DF_Read(uint8_t reg) {
    uint8_t tx[2] = {(uint8_t)(reg | 0x80), 0x00};
    uint8_t rx[2] = {0, 0};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 2, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    return rx[1];
}

void LPS22DF_ReadBlock(uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx[32] = {0}; // Ensure large enough
    uint8_t rx[32] = {0};
    tx[0] = reg | 0x80;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, len + 1, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    for (int i = 0; i < len; i++) data[i] = rx[i + 1];
}

// BMI270 Helpers (SPI1, CS=PA4)
void BMI270_Write(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), data};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t BMI270_Read(uint8_t reg) {
    uint8_t tx[3] = {(uint8_t)(reg | 0x80), 0x00, 0x00};
    uint8_t rx[3] = {0, 0, 0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 10); // Dummy byte is rx[1]
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return rx[2];
}

void BMI270_ReadBlock(uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx[32] = {0};
    uint8_t rx[32] = {0};
    tx[0] = reg | 0x80;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + 2, 10); // Dummy byte is rx[1]
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    for (int i = 0; i < len; i++) data[i] = rx[i + 2];
}

void BMI270_LoadConfig(void) {
    BMI270_Write(0x59, 0x00); // INIT_CTRL: Disable initialization
    HAL_Delay(1);
    
    uint16_t config_size = sizeof(bmi270_config_file);
    for (uint16_t i = 0; i < config_size; i += 256) {
        uint16_t len = config_size - i;
        if (len > 256) len = 256;
        
        // Set INIT_ADDR
        uint16_t addr = i / 2;
        BMI270_Write(0x5B, (uint8_t)(addr & 0x0F));
        BMI270_Write(0x5C, (uint8_t)((addr >> 4) & 0xFF));
        
        // Burst write
        uint8_t tx[257];
        tx[0] = 0x5E & 0x7F; // Write to INIT_DATA
        for (int j = 0; j < len; j++) tx[j + 1] = bmi270_config_file[i + j];
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, tx, len + 1, 100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
    
    BMI270_Write(0x59, 0x01); // INIT_CTRL: Start initialization
    HAL_Delay(25); // Wait for init to complete
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
