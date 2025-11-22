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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AX12.h"
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

COM_InitTypeDef BspCOMInit;

UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */
//uint8_t packet[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x19, 0x00, 0xDE};
//uint8_t packet2[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x19, 0x01, 0xDD};
//
//uint8_t torque_enable_packet[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x01, 0xDE};
//uint8_t goal_pos_512[9] = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x1E, 0x00, 0x02, 0xD6};
//
//#define DXL_ID                      0x01
//#define DXL_TIMEOUT_MS              50
//#define DXL_DIR_PORT GPIOA
//#define DXL_DIR_PIN  GPIO_PIN_3
//#define DXL_TX_MODE() HAL_GPIO_WritePin(DXL_DIR_PORT, DXL_DIR_PIN, GPIO_PIN_SET)
//#define DXL_RX_MODE() HAL_GPIO_WritePin(DXL_DIR_PORT, DXL_DIR_PIN, GPIO_PIN_RESET)
//uint8_t g_rx_buffer[8]; // بافر دریافت
//uint8_t ping_packet[6];
//int ping_result = -1;
//volatile uint16_t g_present_position = 0;

AX12_Handle_TypeDef ax12;
uint16_t current_pos = 0;
uint8_t current_temp = 0;
int read_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART7_Init(void);
/* USER CODE BEGIN PFP */
//uint8_t DXL_CalculateChecksum(uint8_t *packet, uint16_t length)
//{
//    uint8_t checksum = 0;
//    for (uint16_t i = 0; i < length; i++)
//    {
//        checksum += packet[i];
//    }
//    return (~checksum) & 0xFF;
//}
//
//int DXL_Ping_Blocking(uint8_t id)
//{
//    // --- ساخت بسته PING ---
//    uint8_t tx_data[] = {id, 0x02, 0x01}; // ID, Length, PING
//
//    ping_packet[0] = 0xFF;
//    ping_packet[1] = 0xFF;
//    memcpy(&ping_packet[2], tx_data, sizeof(tx_data));
//    ping_packet[5] = DXL_CalculateChecksum(tx_data, sizeof(tx_data));
//
//    // --- 1. ارسال ---
//    DXL_TX_MODE(); // فعال کردن TX
//    HAL_UART_Transmit(&huart7, ping_packet, 6, DXL_TIMEOUT_MS);
//
//    // --- 2. انتظار TC (حیاتی برای نیمه‌دوطرفه) ---
//    // مطمئن می‌شویم که آخرین بیت ارسال شده است.
//    while (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_TC) == RESET);
//
//    // --- 3. تغییر جهت و دریافت ---
//    DXL_RX_MODE(); // تغییر فوری به RX
//
//    // انتظار برای پاسخ 6 بایتی وضعیت (Status Packet)
//    HAL_StatusTypeDef rx_status = HAL_UART_Receive(&huart7, g_rx_buffer, 6, DXL_TIMEOUT_MS);
//
//    if (rx_status != HAL_OK)
//    {
//        return -1; // خطا در دریافت (تایم‌اوت)
//    }
//
//    // --- 4. بررسی پاسخ (اختیاری اما مهم) ---
//    // بررسی Header (0xFF 0xFF) و ID
//    if (g_rx_buffer[0] == 0xFF && g_rx_buffer[1] == 0xFF && g_rx_buffer[2] == id)
//    {
//        // ... (می‌توانید Checksum و بایت Error را بررسی کنید)
//        if (g_rx_buffer[4] == 0x00)
//        {
//            return 0; // PING موفقیت آمیز
//        }
//    }
//    return -2; // پاسخ نامعتبر
//}
//
//void DXL_SendPacket(uint8_t *packet, uint16_t length)
//{
//    // 1. Set Direction to TX
//    DXL_TX_MODE();
//
//    // 2. Transmit
//    HAL_UART_Transmit(&huart7, packet, length, DXL_TIMEOUT_MS);
//
//    // 3. Wait for Transmission Complete (Critical for Half-Duplex)
//    while (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_TC) == RESET);
//
//    // 4. Set Direction to RX immediately to listen for Status Packet
//    DXL_RX_MODE();
//
//    // 5. Receive Status Packet (Optional: You can ignore this for now if you just want to move)
//    // The motor WILL send back 6 bytes (0xFF 0xFF ID Len Err CS)
//    HAL_UART_Receive(&huart7, g_rx_buffer, 6, DXL_TIMEOUT_MS);
//}
//void DXL_SetGoalPosition(uint8_t id, uint16_t position)
//{
//    uint8_t packet[9];
//
//    // Limit position to 0-1023 (Safe range)
//    if (position > 1023) position = 1023;
//
//    uint8_t pos_L = position & 0xFF;        // Low Byte
//    uint8_t pos_H = (position >> 8) & 0xFF; // High Byte
//
//    packet[0] = 0xFF;
//    packet[1] = 0xFF;
//    packet[2] = id;
//    packet[3] = 0x05; // Length: Instr(1) + Addr(1) + DataL(1) + DataH(1) + 2(offset) -> actually manual says N+2. N=3. So 5.
//    packet[4] = 0x03; // Instruction: WRITE_DATA
//    packet[5] = 0x1E; // Address: Goal Position
//    packet[6] = pos_L;
//    packet[7] = pos_H;
//
//    // Calculate Checksum: ~(ID + Length + Instruction + Param1 + Param2 + Param3)
//    uint8_t checksum = (id + 0x05 + 0x03 + 0x1E + pos_L + pos_H);
//    packet[8] = (~checksum) & 0xFF;
//
//    DXL_SendPacket(packet, 9);
//}
//
//int DXL_ReadPosition(uint8_t id)
//{
//    uint8_t tx_packet[8];
//
//    // --- 1. Construct Packet ---
//    tx_packet[0] = 0xFF;
//    tx_packet[1] = 0xFF;
//    tx_packet[2] = id;
//    tx_packet[3] = 0x04;
//    tx_packet[4] = 0x02; // Instruction: READ_DATA
//    tx_packet[5] = 0x24; // Address: Present Position (Low Byte)
//    tx_packet[6] = 0x02; // Length to read: 2 Bytes
//
//    // Checksum calculation
//    uint8_t checksum = (id + 0x04 + 0x02 + 0x24 + 0x02);
//    tx_packet[7] = (~checksum) & 0xFF;
//
//    // --- 2. Transmit ---
//    DXL_TX_MODE();
//    if(HAL_UART_Transmit(&huart7, tx_packet, 8, DXL_TIMEOUT_MS) != HAL_OK)
//    {
//        return -1;
//    }
//
//    // --- 3. Switch Direction ---
//    while (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_TC) == RESET);
//    DXL_RX_MODE();
//
//    // =================================================================
//    // NEW STEP: CLEAR FLAGS AND FLUSH BUFFER
//    // This removes the "Echo" of your own transmission and clears Overrun errors
//    // =================================================================
//
//    // Clear Overrun, Noise, Framing, Parity errors
//    __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
//
//    // Flush any data sitting in the RX Data Register (RDR)
//    uint8_t temp;
//    while(__HAL_UART_GET_FLAG(&huart7, UART_FLAG_RXNE))
//    {
//        temp = (uint8_t)(huart7.Instance->RDR & 0x00FF);
//    }
//    // =================================================================
//
//    // --- 4. Receive Response ---
//    // Now the UART is clean and ready for the real Motor Response
//    if(HAL_UART_Receive(&huart7, g_rx_buffer, 8, DXL_TIMEOUT_MS) != HAL_OK)
//    {
//        return -1; // Timeout or HAL Error
//    }
//
//    // --- 5. Parse Data ---
//    if (g_rx_buffer[0] == 0xFF && g_rx_buffer[1] == 0xFF && g_rx_buffer[2] == id)
//    {
//        if (g_rx_buffer[4] != 0) return -2; // Motor Error
//
//        uint8_t pos_low = g_rx_buffer[5];
//        uint8_t pos_high = g_rx_buffer[6];
//
//        g_present_position = (uint16_t)(pos_high << 8) | pos_low;
//
//        return 0; // Success
//    }
//
//    return -3; // Invalid Packet Header
//}
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  // 1. Initialize the library object
  HAL_Delay(1000);

  // Initialize motor handle (UART7, PA3, ID 1)
  AX12_Init(&ax12, &huart7, GPIOA, GPIO_PIN_3, 0x01);

  // Enable Torque (The motor must be enabled to move)
  AX12_TorqueEnable(&ax12, 1);
  HAL_Delay(50);

  // Set Speed (Test a value other than the max/default)
  AX12_SetMovingSpeed(&ax12, 150); // Setting to 150 (out of 1023)
  HAL_Delay(50);
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // --- Phase 1: Move to 0 (CCW Limit) ---
	      AX12_SetGoalPosition(&ax12, 0);
	      HAL_Delay(1000); // Wait for the motor to reach position

	      // Read and verify position
	      read_status = AX12_GetPresentPosition(&ax12, &current_pos);

	      // --- Phase 2: Move to Center (512) ---
	      AX12_SetGoalPosition(&ax12, 512);
	      HAL_Delay(1000);

	      // Read and verify position
	      read_status = AX12_GetPresentPosition(&ax12, &current_pos);

	      // --- Phase 3: Move to 1023 (CW Limit) ---
	      AX12_SetGoalPosition(&ax12, 1023);
	      HAL_Delay(1000);

	      // Read and verify position
	      read_status = AX12_GetPresentPosition(&ax12, &current_pos);

	      // --- Phase 4: Read Temperature (Test single-byte read) ---
	      AX12_GetPresentTemperature(&ax12, &current_temp);

	      HAL_Delay(1000);

//	  	    DXL_SetGoalPosition(DXL_ID, 0);
//	        HAL_Delay(1000);
//	        DXL_ReadPosition(DXL_ID);
//	        HAL_Delay(1000);
//
//	        // Move to Position 512 (150 degrees - Center)
//	        DXL_SetGoalPosition(DXL_ID, 512);
//	        HAL_Delay(1000);
//	        DXL_ReadPosition(DXL_ID);
//	        HAL_Delay(1000);
//	        // Move to Position 1023 (300 degrees)
//	        DXL_SetGoalPosition(DXL_ID, 1023);
//	        HAL_Delay(1000);
//	        DXL_ReadPosition(DXL_ID);
//	        HAL_Delay(1000);
//	  	  ping_result = DXL_Ping_Blocking(DXL_ID);
//	  	  HAL_Delay(500);
//      HAL_UART_Transmit(&huart7, packet,  sizeof(packet), HAL_MAX_DELAY);
//      HAL_Delay(2000);
//      HAL_UART_Transmit(&huart7, packet2,  sizeof(packet), HAL_MAX_DELAY);
//      HAL_Delay(2000);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 1000000;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
