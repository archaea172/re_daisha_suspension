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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "t_semi_can_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint16_t CANID;
	uint8_t motorID;
	volatile float actVel;//rpm
	volatile float actangle;
	volatile float actCurrent;
	float cu;
	float trgVel;
	volatile float p_actVel;//rpm
	volatile float ind;
	float w;
}motor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0

#define ERROR_STATE -1
#define INITIALIZE_STATE 0
#define MOVE_STATE 1

#define STATE_CANID_P_DOWN 0x100
#define STATE_CANID_P_UP 0x101

#define ID 1

#define STATE_CANID_DOWN STATE_CANID_P_DOWN + (ID*16)
#define STATE_CANID_UP STATE_CANID_P_UP + (ID*16)

#define R_F 4
#define L_F 3
#define R_B 1
#define L_B 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const float a0 = M_PI/180*45;
const float a1 = M_PI/180*135;
const float a2 = M_PI/180*225;
const float a3 = M_PI/180*315;
const float r = 0.03;//m
const float R = 0.144;//m

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader_motor;
FDCAN_RxHeaderTypeDef RxHeader_motor;

int8_t state = ERROR_STATE;
int8_t sub_state;

float32_t RxData_f32[16] = {};
uint32_t CANID_R = 0;

motor robomas[4] = {
		{0x201, 1, 0, 0, 0, 0, 0, 0, 0, 0},
		{0x202, 2, 0, 0, 0, 0, 0, 0, 0, 0},
		{0x203, 3, 0, 0, 0, 0, 0, 0, 0, 0},
		{0x204, 4, 0, 0, 0, 0, 0, 0, 0, 0},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
void CAN_SEND_printf(uint32_t, uint8_t*);
void CAN(uint32_t, float32_t*);
void CAN_SEND_printf_robomas(uint32_t, uint8_t*);
void CAN_robomas(motor*);
void interboard_comms_CAN_RxTxSettings_nhk2025_init(void);
void robomas_CAN_RxTxSettings_nhk2025_init(void);

void omni_calc(float theta,float vx,float vy,float omega,float *w0,float *w1,float *w2,float *w3);
float convert_rpm_radps(float rpm);
float convert_radps_rpm(float radps) ;
void omni_control(float Theta, float Vx, float Vy, float Omega, motor *Robomas);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

	        /* Retrieve Rx messages from RX FIFO0 */
		uint8_t RxData_motor[8] = {};
		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader_motor, RxData_motor)) {
			printf("fdcan_getrxmessage_motor is error\r\n");
			Error_Handler();
		}
		/*receive robomas's status*/
		for (int i=0; i < 4; i++){
			if (RxHeader_motor.Identifier == (robomas[i].CANID)) {
				robomas[i].actangle = (int16_t)((RxData_motor[0] << 8) | RxData_motor[1]);
				robomas[i].actVel = (int16_t)((RxData_motor[2] << 8) | RxData_motor[3]);
				robomas[i].actCurrent = (int16_t)((RxData_motor[4] << 8) | RxData_motor[5]);
			}
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if (RESET != (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {

	        /* Retrieve Rx messages from RX FIFO0 */
		uint8_t RxData[64] = {};
		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData)) {
			printf("fdcan_getrxmessage is error\r\n");
			Error_Handler();
		}

		for (int i = 0; i < 16; i++) {
			convert_u8_f32(RxData, RxData_f32);
		}
		CANID_R = RxHeader.Identifier;
	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (&htim6 == htim) {
		float32_t TxData_f32[16] = {};
		TxData_f32[0] = (float32_t)state;
		TxData_f32[1] = (float32_t)sub_state;
		CAN(STATE_CANID_UP, TxData_f32);
	}
	else if (&htim7 == htim) {
		if (STATE_CANID_DOWN == CANID_R) {
			if (INITIALIZE_STATE == RxData_f32[0]) {
				state = RxData_f32[0];
			}
			else if (ERROR_STATE == RxData_f32[0]) {
				state = RxData_f32[0];
			}
		}
	}
	else if (&htim16 == htim) {
		for (int i = 0; i < 4; i++) {
			float kp = 10, kd = 0.009, ki = 0.2;
			float dt = 0.001;
			float hensa = robomas[i].trgVel - robomas[i].actVel;
			float derivative = (robomas[i].p_actVel - robomas[i].actVel)/dt;
			robomas[i].ind += hensa;
			robomas[i].cu = kp*hensa + kd*derivative + ki*robomas[i].ind;
			robomas[i].p_actVel = robomas[i].actVel;
			if (robomas[i].cu > 10000) robomas[i].cu = 10000;
			if (robomas[i].cu < -10000) robomas[i].cu = -10000;
		}
		CAN_robomas(robomas);
	}
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);

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
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_FDCAN3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  printf("start\r\n");
  //FDCAN_RxTxSettings();
  interboard_comms_CAN_RxTxSettings_nhk2025_init();
  robomas_CAN_RxTxSettings_nhk2025_init();
  printf("can start\r\n");

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (ERROR_STATE == state) {

	  }
	  else if (INITIALIZE_STATE == state) {

	  }
	  else if (MOVE_STATE == state) {

	  }
	  HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 2;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 15;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 7999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 9;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 7999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 9;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 7999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Board_LED_Pin */
  GPIO_InitStruct.Pin = Board_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_SEND_printf(uint32_t CANID, uint8_t *txdata) {
	if(CAN_SEND(CANID, txdata, &hfdcan1, &TxHeader) < 0) {
		printf("can_addmessage error\r\n");
		Error_Handler();
	}
}

void CAN(uint32_t CANID, float32_t *txdata_f32) {
	uint8_t txdata[64] = {};
	convert_f32_u8(txdata_f32, txdata);
	CAN_SEND_printf(CANID, txdata);
}

void CAN_SEND_printf_robomas(uint32_t CANID, uint8_t *txdata) {
	if (CAN_SEND(CANID, txdata, &hfdcan3, &TxHeader_motor) < 0) {
		printf("addmessage errror robomas\r\n");
		Error_Handler();
	}
}

void CAN_robomas(motor *Robomas) {
	uint8_t TxData_motor_0x200[8];
	//uint8_t TxData_motor_0x1ff[8];
	for (int i = 0; i < 4; i++) {
		TxData_motor_0x200[i*2] = ((int16_t)Robomas[i].cu) >> 8;
		TxData_motor_0x200[i*2+1] = (uint8_t)(((int16_t)Robomas[i].cu) & 0xff);
		//TxData_motor_0x1ff[i*2] = (Robomas[i+4].cu) >> 8;
		//TxData_motor_0x1ff[i*2+1] = (uint8_t)((Robomas[i+4].cu) & 0xff);
	}
	CAN_SEND_printf_robomas(0x200, TxData_motor_0x200);
	//CAN_SEND_printf_robomas(0x1ff, TxData_motor_0x1ff);
}

void interboard_comms_CAN_RxTxSettings_nhk2025_init(void) {
	  FDCAN_FilterTypeDef FDCAN_Filter_settings;
	  int return_value = interboard_comms_CAN_RxTxSettings_nhk2025(&hfdcan1, &FDCAN_Filter_settings, &TxHeader);
	  switch (return_value) {
	  	  case -1:
	  		  printf("fdcan_configfilter is error\r\n");
	  		  break;
	  	  case -2:
	  		  printf("fdcan_configglobalfilter is error\r\n");
	  		  break;
	  	  case -3:
	  		  printf("fdcan_start is error\r\n");
	  		  break;
	  	  case -4:
	  		  printf("fdcan_activatenotification is error\r\n");
	  		  break;
	  }
	  if (return_value < 0) Error_Handler();
	  printf("can_init success\r\n");
}

void robomas_CAN_RxTxSettings_nhk2025_init(void) {
	FDCAN_FilterTypeDef FDCAN_Filter_settings;
	int return_value = robomas_CAN_RxTxSettings_nhk2025(&hfdcan3, &FDCAN_Filter_settings, &TxHeader_motor);
	switch (return_value) {
	  case -1:
		  printf("fdcan_configfilter is error\r\n");
		  break;
	  case -2:
		  printf("fdcan_configglobalfilter is error\r\n");
		  break;
	  case -3:
		  printf("fdcan_start is error\r\n");
		  break;
	  case -4:
		  printf("fdcan_activatenotification is error\r\n");
		  break;
	}
}

void omni_calc(float theta,float vx,float vy,float omega,float *w0,float *w1,float *w2,float *w3){

	float v[3] = {vx, vy, omega};
	float sint = sin(theta);
	float cost = cos(theta);

	float arr[4][3] =
	{{-cos(a0)*sint-sin(a0)*cost, cos(a0)*cost-sin(a0)*sint, R},
	{-cos(a1)*sint-sin(a1)*cost, cos(a1)*cost-sin(a1)*sint, R},
	{-cos(a2)*sint-sin(a2)*cost, cos(a2)*cost-sin(a2)*sint, R},
	{-cos(a3)*sint-sin(a3)*cost, cos(a3)*cost-sin(a3)*sint, R}};

	*w0 = (arr[0][0] * v[0] + arr[0][1] * v[1] + arr[0][2] * v[2]) / r;
	*w1 = (arr[1][0] * v[0] + arr[1][1] * v[1] + arr[1][2] * v[2]) / r;
	*w2 = (arr[2][0] * v[0] + arr[2][1] * v[1] + arr[2][2] * v[2]) / r;
	*w3 = (arr[3][0] * v[0] + arr[3][1] * v[1] + arr[3][2] * v[2]) / r;
}

float convert_rpm_radps(float rpm) {
	float radps = rpm/60*2*M_PI;
	return radps;
}

float convert_radps_rpm(float radps) {
	float rpm = radps*60/2/M_PI;
	return rpm;
}

void omni_control(float Theta, float Vx, float Vy, float Omega, motor *Robomas) {
	omni_calc(Theta ,Vx, Vy, Omega, &Robomas[R_F-1].w, &Robomas[L_F-1].w, &Robomas[L_B-1].w, &Robomas[R_B-1].w);
	for (int i = 0; i < 4; i++) Robomas[i].trgVel = convert_radps_rpm(Robomas[i].w)*36*(-1);
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
