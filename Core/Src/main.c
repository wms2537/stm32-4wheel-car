/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "jy62.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    READY,
    RUNNING,
    BATTLING,
    FINISHED
} GameStage_edc25;

typedef struct
{
    float posx;
    float posy;
} Position_edc25;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LEN 24
#define RX_BUF_LEN 24
#define MAX_SINGLE_MSG 95 // 可修正
#define MAX_MSG_LEN 150
#define MAX_STATUS_LEN 150
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t k210Receive[66];
uint8_t k210ReceiveByte;
uint8_t k210ReceiveIndex = 0;

uint8_t jy62Receive[JY62_MESSAGE_LENTH];	//实时记录收到的信息
uint8_t jy62ReceiveByte;
uint8_t currentReceiveIndex = 0;
uint8_t jy62Message[JY62_MESSAGE_LENTH];   //确认无误后用于解码的信息

uint8_t zigbeeRaw[MAX_MSG_LEN];         // Raw zigbee data
uint8_t zigbeeMessage[MAX_MSG_LEN * 2]; // Double the size to save a complete message
int32_t memPtr = 0;
uint8_t cutavoid[4];

uint8_t gameStatusMessage[MAX_STATUS_LEN];

Position_edc25 Pos;
Position_edc25 PosOpp;

uint8_t map[8][8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void set_direction(int mfr, int mfl, int mbr, int mbl);
void set_motor_speed(int mfr, int mfl, int mbr, int mbl);
void motor_forward(uint16_t time);
void motor_backward(uint16_t time);
void motor_left(uint16_t time);
void motor_right(uint16_t time);
void motor_right_90deg();
void motor_left_90deg();

void zigbee_Init(UART_HandleTypeDef *huart); // 初始化,开始接收消息
uint8_t zigbeeMessageRecord();                  // 刷新消息
int32_t getGameTime();
GameStage_edc25 getGameStage();
void getHeightOfAllChunks(uint8_t *dest);
uint8_t getHeightOfId(uint8_t id);
bool hasBed();
bool hasBedOpponent();
void getPosition(Position_edc25 *Pos);
void getPositionOpponent(Position_edc25 *Pos);
uint8_t getAgility();
uint8_t getHealth();
uint8_t getMaxHealth();
uint8_t getStrength();
uint8_t getEmeraldCount();
uint8_t getWoolCount();
void attack_id(uint8_t chunk_id);
void place_block_id(uint8_t chunk_id);
void trade_id(uint8_t item_id);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Receive_DMA(&huart1, &u1_RX_ReceiveBit, 1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  set_motor_speed(0,0,0,0);
//  motor_forward();
  HAL_Delay(2000);
  zigbee_Init(&huart1);
  jy62_Init(&huart2);
  HAL_UART_Receive_DMA(&huart2,&jy62ReceiveByte,1);
  HAL_UART_Receive_DMA(&huart3,k210Receive,64);
  SetHorizontal();
  InitAngle();
  Calibrate();
  HAL_Delay(2000);
  uint8_t hp = 0;

	uint8_t Agility = 0;

	uint8_t count = 0;
//  SleepOrAwake();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  hp = getHealth();
	  	  int32_t time = getGameTime();
	  	  Agility = getAgility();
	  	  count = getWoolCount();
	  	  getPosition(&Pos);
		  getPositionOpponent(&PosOpp);

		  char char_buf[200];
		  int char_buf_len = sprintf(char_buf, "t: %ld, ht: %d, ag: %d, wc: %d, x: %f, y: %f, xo: %f, yo: %f\r\n", time, hp, Agility, count, Pos.posx, Pos.posy, PosOpp.posx, PosOpp.posy);
		  HAL_UART_Transmit(&huart3, (uint8_t*)char_buf, char_buf_len, 100);
//	  		  HAL_Delay(1000);

	  	  HAL_Delay(2000);
//	  	  srand(HAL_GetTick());
//	  	  int num_a = rand()%(10+1);
//	  	  int num_b = rand()%(10+1);
//		  int num_c = rand()%(10+1);
//		  int num_d = rand()%(10+1);
//		  char uart_buf[24];
//	  	  int uart_buf_len = sprintf(uart_buf, "A=(%d,%d)B=(%d,%d)\r\n", num_a, num_b, num_c, num_d);
//	  	  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, uart_buf_len, 100);
//		  HAL_Delay(1000);

//	  	  motor_forward(500);
//	  	  HAL_Delay(1000);
//
//	  	  motor_backward(500);
//		  HAL_Delay(1000);
//
//		  motor_right(457);
//		  HAL_Delay(1000);
//
//		  motor_left(481);
//		  HAL_Delay(1000);


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BR_IN4_Pin|BR_IN3_Pin|BL_IN1_Pin|BL_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FR_IN4_Pin|FR_IN3_Pin|FL_IN2_Pin|FL_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BR_IN4_Pin BR_IN3_Pin BL_IN1_Pin BL_IN2_Pin */
  GPIO_InitStruct.Pin = BR_IN4_Pin|BR_IN3_Pin|BL_IN1_Pin|BL_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FR_IN4_Pin FR_IN3_Pin FL_IN2_Pin FL_IN1_Pin */
  GPIO_InitStruct.Pin = FR_IN4_Pin|FR_IN3_Pin|FL_IN2_Pin|FL_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int32_t modularAdd(int32_t a, int32_t b, int32_t max) // In case modular does not work for negative numbers
{
    int32_t c;
    c = a + b;
    if (c >= max)
        c -= max;
    if (c < 0)
        c += max;
    return c;
}

static uint8_t calculateChecksum(uint8_t tempMessage[], int32_t start_idx, int32_t count)
{
    uint8_t checksum = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        checksum ^= tempMessage[modularAdd(i, start_idx, MAX_MSG_LEN * 2)];
    }
    return checksum;
}

static float changeFloatData(uint8_t *dat)
{
    float float_data;
    float_data = *((float *)dat);
    return float_data;
}


void zigbee_Init(UART_HandleTypeDef *huart)
{
    memset(zigbeeMessage, 0x00, MAX_MSG_LEN);
    memset(zigbeeRaw, 0x00, MAX_MSG_LEN);
    memset(gameStatusMessage, 0x00, MAX_STATUS_LEN);
    HAL_UART_Receive_DMA(&huart1, zigbeeRaw, MAX_MSG_LEN);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
//    	char charbuf[] = "DMA_HALF\r\n";
//    	HAL_UART_Transmit(&huart3, (uint8_t*)charbuf, sizeof(charbuf), 100);
        uint8_t *zigbeeMsgPtr = &zigbeeMessage[memPtr];
        uint8_t *rawPtr = &zigbeeRaw[0];
        memcpy(zigbeeMsgPtr, rawPtr, sizeof(uint8_t) * MAX_MSG_LEN / 2);
        memPtr = modularAdd(MAX_MSG_LEN / 2, memPtr, MAX_MSG_LEN * 2);
        zigbeeMessageRecord();
        // zigbeeMessageRecord is completed almost instantly in the callback function.
        // Please don't add u1_printf into the function.
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if(huart == &huart2)//我这里选择的是uart2所以这里用的是&huart2，其实应该是大家选择哪个串口就填写哪个
	{
		jy62Receive[currentReceiveIndex ++] = jy62ReceiveByte;
			if(currentReceiveIndex ==  JY62_MESSAGE_LENTH) {
				currentReceiveIndex = 0;
				if(jy62Receive[0] ==0x55)
					{
						uint8_t sum  = 0x00;
						for (int i = 0; i < JY62_MESSAGE_LENTH-1; i++)
						{
							sum += jy62Receive[i];
						}
						if(sum == jy62Receive[JY62_MESSAGE_LENTH-1])
						{
							for (int i = 0; i < JY62_MESSAGE_LENTH; i++)
							{
								jy62Message[i] = jy62Receive[i];
							}
						    Decode(jy62Message);
						}
					}
			}
		//	HAL_UART_Receive_DMA(jy62_huart,jy62Receive,JY62_MESSAGE_LENTH);
			HAL_UART_Receive_DMA(&huart2,&jy62ReceiveByte,1);
	}
	if(huart == &huart1){
//		char charbuf[] = "DMA\r\n";
//		HAL_UART_Transmit(&huart3, (uint8_t*)charbuf, sizeof(charbuf), 100);
		uint8_t *zigbeeMsgPtr = &zigbeeMessage[memPtr];
		uint8_t *rawPtr = &zigbeeRaw[MAX_MSG_LEN / 2];
		memcpy(zigbeeMsgPtr, rawPtr, sizeof(uint8_t) * MAX_MSG_LEN / 2);
		memPtr = modularAdd(MAX_MSG_LEN / 2, memPtr, MAX_MSG_LEN * 2);
		zigbeeMessageRecord();
	}
	if(huart == &huart3)
		{
		char charbuf[] = "DMA\r\n";
				HAL_UART_Transmit(&huart3, (uint8_t*)charbuf, sizeof(charbuf), 100);
				for(uint8_t i = 0; i< 8; i++) {
									for(uint8_t j = 0; j< 8; j++) {
										map[i][j] = k210Receive[i*8+j];
									}
								}
								for(uint8_t i = 0; i< 8; i++) {
									char char_buf[200];
									int char_buf_len = sprintf(char_buf, "Row %d: \n%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\r\n", i, map[i][0], map[i][1], map[i][2], map[i][3], map[i][4], map[i][5], map[i][6], map[i][7]);
									HAL_UART_Transmit(&huart3, (uint8_t*)char_buf, char_buf_len, 100);
								}

			HAL_UART_Receive_DMA(&huart3,k210Receive,64);
		}
}

void set_direction(int mfr, int mfl, int mbr, int mbl){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, mfl == 1 ? 1 : 0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, mfl == 1 ? 0 : 1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4, mfr == 1 ? 1 : 0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, mfr == 1 ? 0 : 1);

	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, mbr == 1 ? 0 : 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, mbr == 1 ? 1 : 0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, mbl == 1 ? 1 : 0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, mbl == 1 ? 0 : 1);
}

void set_motor_speed(int mfr, int mfl, int mbr, int mbl){
	TIM1->CCR1 = mfr;
	TIM1->CCR4 = mfl;
	TIM2->CCR3 = mbr;
	TIM2->CCR4 = mbl;
}

void motor_forward(uint16_t time){
	set_direction(1,1,1,1);
	set_motor_speed(1000,1000,1000,1000);
	HAL_Delay(time*0.8);
	set_motor_speed(500,500,500,500);
	HAL_Delay(time*0.2);
	set_direction(0,0,0,0);
	set_motor_speed(1000,1000,1000,1000);
	HAL_Delay(80);
	set_motor_speed(0,0,0,0);
}

void motor_backward(uint16_t time){
	set_direction(0,0,0,0);
		set_motor_speed(1000,1000,1000,1000);
		HAL_Delay(time*0.8);
		set_motor_speed(500,500,500,500);
		HAL_Delay(time*0.2);
		set_direction(1,1,1,1);
		set_motor_speed(1000,1000,1000,1000);
		HAL_Delay(80);
		set_motor_speed(0,0,0,0);
}

void motor_left(uint16_t time){
	set_direction(1,0,1,0);
	set_motor_speed(1000,1000,1000,1000);
	HAL_Delay(time);
	set_direction(0,1,0,1);
	HAL_Delay(65);
	set_motor_speed(0,0,0,0);
}

void motor_right(uint16_t time){
	set_direction(0,1,0,1);
	set_motor_speed(1000,1000,1000,1000);
	HAL_Delay(time);
	set_direction(1,0,1,0);
	HAL_Delay(65);
	set_motor_speed(0,0,0,0);
}

void motor_right_90deg(){
	float initialYaw = GetYaw();
	float targetYaw = initialYaw + 80.0;
	uint8_t offset = 0;
	if(targetYaw >= 360) {
		targetYaw = targetYaw - 360.0;
		offset = 1;
	}
    set_direction(0,1,0,1);
    set_motor_speed(1000,1000,1000,1000);
	while(1){
		float currentYaw = GetYaw();
		if(offset == 1) {
			if(currentYaw >= targetYaw && currentYaw < initialYaw) break;
		} else {
			if(currentYaw >= targetYaw) break;
		}
	}
	set_direction(1,0,1,0);
	HAL_Delay(100);
	set_motor_speed(0,0,0,0);
	return;
}

void motor_left_90deg(){
	float initialYaw = GetYaw();
	float targetYaw = initialYaw - 80.0;
	uint8_t offset = 0;
	if(targetYaw < 0) {
		targetYaw = targetYaw + 360.0;
		offset = 1;
	}
	set_direction(1,0,1,0);
	set_motor_speed(1000,1000,1000,1000);
	while(1){
		float currentYaw = GetYaw();
		if(offset == 1) {
			if(currentYaw <= targetYaw && currentYaw > initialYaw) break;
		} else {
			if(currentYaw <= targetYaw) break;
		}
	}
	set_direction(0,1,0,1);
	HAL_Delay(100);
	set_motor_speed(0,0,0,0);
	return;
}

uint8_t zigbeeMessageRecord()
{

    int32_t msgIndex = 0;
    uint8_t tempZigbeeMessage[MAX_MSG_LEN * 2];
    memcpy(tempZigbeeMessage, zigbeeMessage, MAX_MSG_LEN * 2);
    // In case zigbeeMessage updates in the interrupt during the loop

    int32_t prevMemPtr = memPtr; // In case memPtr changes in the interrupt during the loop
    int16_t byteNum;
    // find the first 0x55 of msgType
    for (msgIndex = modularAdd(prevMemPtr, -MAX_SINGLE_MSG, MAX_MSG_LEN * 2); msgIndex != prevMemPtr;)
    // A message is at most 30 bytes long. We find the header of the first full message
    {
        if (tempZigbeeMessage[msgIndex] == 0x55 &&
            tempZigbeeMessage[modularAdd(msgIndex, 1, MAX_MSG_LEN * 2)] == 0xAA)
        {


            cutavoid[0] = tempZigbeeMessage[modularAdd(msgIndex, 2, MAX_MSG_LEN * 2)];
            cutavoid[1] = tempZigbeeMessage[modularAdd(msgIndex, 3, MAX_MSG_LEN * 2)];
            byteNum = *((int16_t*)(cutavoid));

            uint8_t tmpchecksum;
            tmpchecksum = calculateChecksum(tempZigbeeMessage, modularAdd(msgIndex, 5, MAX_MSG_LEN * 2), byteNum);
            if (tmpchecksum == tempZigbeeMessage[modularAdd(msgIndex, 4, MAX_MSG_LEN * 2)])
            {
                break;
            }
        }
        msgIndex = modularAdd(msgIndex, -1, MAX_MSG_LEN * 2);
    }
    if (msgIndex == prevMemPtr)
    {
        return 1;
    }

    int32_t prevTime, newTime;
    prevTime = getGameTime();
    for(int32_t i = 0;i < 4;i++)
    {
        cutavoid[i] =  tempZigbeeMessage[modularAdd(msgIndex, 5 + 1 + i, MAX_MSG_LEN * 2)];
    }
    newTime = *((int32_t *)(cutavoid));
    if (newTime >= prevTime && newTime <= prevTime + 1000)
    {
        memset(gameStatusMessage, 0x00, MAX_STATUS_LEN);
        for (int32_t i = 0; i < byteNum; i++)
        {
            gameStatusMessage[i] = tempZigbeeMessage[modularAdd(msgIndex, 5 + i, MAX_MSG_LEN * 2)];
        }
    }
    return 0;
}

int32_t getGameTime()
{
    int32_t time;
    time = *((int32_t *)(&gameStatusMessage[1]));
    return time;
}

GameStage_edc25 getGameStage()
{
    uint8_t stage;
    stage = gameStatusMessage[0];
    return (GameStage_edc25)stage;
}

void getHeightOfAllChunks(uint8_t *dest)
{
    memcpy(dest, gameStatusMessage[5], 64);
}

uint8_t getHeightOfId(uint8_t id)
{
    return gameStatusMessage[5 + id];
}

bool hasBed()
{
    return (bool)gameStatusMessage[69];
}

bool hasBedOpponent()
{
    return (bool)gameStatusMessage[70];
}

void getPosition(Position_edc25 *Pos)
{
    Pos->posx = changeFloatData(gameStatusMessage + 71);
    Pos->posy = changeFloatData(gameStatusMessage + 75);
}

void getPositionOpponent(Position_edc25 *Pos)
{
    Pos->posx = changeFloatData(gameStatusMessage + 79);
    Pos->posy = changeFloatData(gameStatusMessage + 83);
}

uint8_t getAgility()
{
    return gameStatusMessage[87];
}

uint8_t getHealth()
{
    return gameStatusMessage[88];
}

uint8_t getMaxHealth()
{
    return gameStatusMessage[89];
}

uint8_t getStrength()
{
    return gameStatusMessage[90];
}

uint8_t getEmeraldCount()
{
    return gameStatusMessage[91];
}

uint8_t getWoolCount()
{
    return gameStatusMessage[92];
}

void attack_id(uint8_t chunk_id)
{
    uint8_t slaver_msg[7] = {0x55, 0xAA, 0x02, 0x00, (uint8_t)(0^chunk_id), 0, chunk_id};
    HAL_UART_Transmit(&huart1, slaver_msg, 7, HAL_MAX_DELAY);
}

void place_block_id(uint8_t chunk_id)
{
    uint8_t slaver_msg[7] = {0x55, 0xAA, 0x02, 0x00, (uint8_t)(1^chunk_id), 1, chunk_id};
    HAL_UART_Transmit(&huart1, slaver_msg, 7, HAL_MAX_DELAY);
}

void trade_id(uint8_t item_id)
{
    uint8_t slaver_msg[7] = {0x55, 0xAA, 0x02, 0x00, (uint8_t)(2^item_id), 2, item_id};
    HAL_UART_Transmit(&huart1, slaver_msg, 7, HAL_MAX_DELAY);
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
