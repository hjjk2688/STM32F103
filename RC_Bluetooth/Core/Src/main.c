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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t transmit[256];

uint8_t rx3_data;
uint8_t rx2_data;

volatile uint8_t new_data_flag = 0; // 새 데이터 수신 여부를 알리는 플래그
volatile uint32_t last_rx_time = 0; // 마지막으로 데이터를 수신한 시간

int HIGH = 1;
int LOW = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROT0TYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
	if (ch == '\n')
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

//	HAL_UART_Transmit(&huart3, (uint8_t*) "\r", 1, 0xFFFF);
//	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void smartcar_forward(){
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 1); // PB8  뒤앞   21 22  RR_F /Right_RearForward
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 0); // PB9  뒤백	RR_B Right_RearBack

	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 1); // PB4 앞앞  17 16 RF_F
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 0); // PB5 앞백 RF_B

	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 1); // PA9  뒤앞 LR_F 12  27 left_RearForward
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 0); // PB10 뒤백 LR_B left_RearBack

	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 1); // 앞앞 PB3 25 26 LF_F left_ForwardForward
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 0); // 앞백 PA10 LF_B left_ForwardBack
}

void smartcar_back(){
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 0); // PB8  뒤앞   21 22  RR_F /Right_RearForward
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 1); // PB9  뒤백	RR_B Right_RearBack

	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 0); // PB4 앞앞  17 16 RF_F
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 1); // PB5 앞백 RF_B

	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 0); // PA9  뒤앞 LR_F 12  27 left_RearForward
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 1); // PB10 뒤백 LR_B left_RearBack

	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 0); // 앞앞 PB3 25 26 LF_F left_ForwardForward
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 1); // 앞백 PA10 LF_B left_ForwardBack
}

void smartcar_left(){
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 1); // PB8  뒤앞   21 22  RR_F /Right_RearForward
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 0); // PB9  뒤백	RR_B Right_RearBack

	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 1); // PB4 앞앞  17 16 RF_F
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 0); // PB5 앞백 RF_B

	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 0); // PA9  뒤앞 LR_F 12  27 left_RearForward
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 1); // PB10 뒤백 LR_B left_RearBack

	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 0); // 앞앞 PB3 25 26 LF_F left_ForwardForward
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 1); // 앞백 PA10 LF_B left_ForwardBack
}

void smartcar_right(){
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 0); // PB8  뒤앞   21 22  RR_F /Right_RearForward
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 1); // PB9  뒤백	RR_B Right_RearBack

	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 0); // PB4 앞앞  17 16 RF_F
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 1); // PB5 앞백 RF_B

	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 1); // PA9  뒤앞 LR_F 12  27 left_RearForward
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 0); // PB10 뒤백 LR_B left_RearBack

	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 1); // 앞앞 PB3 25 26 LF_F left_ForwardForward
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 0); // 앞백 PA10 LF_B left_ForwardBack
}

void smartcar_stop(){
	HAL_GPIO_WritePin(RR_F_GPIO_Port, RR_F_Pin, 0); // PB8  뒤앞   21 22  RR_F /Right_RearForward
	HAL_GPIO_WritePin(RR_B_GPIO_Port, RR_B_Pin, 0); // PB9  뒤백	RR_B Right_RearBack

	HAL_GPIO_WritePin(RF_F_GPIO_Port, RF_F_Pin, 0); // PB4 앞앞  17 16 RF_F
	HAL_GPIO_WritePin(RF_B_GPIO_Port, RF_B_Pin, 0); // PB5 앞백 RF_B

	HAL_GPIO_WritePin(LR_F_GPIO_Port, LR_F_Pin, 0); // PA9  뒤앞 LR_F 12  27 left_RearForward
	HAL_GPIO_WritePin(LR_B_GPIO_Port, LR_B_Pin, 0); // PB10 뒤백 LR_B left_RearBack

	HAL_GPIO_WritePin(LF_F_GPIO_Port, LF_F_Pin, 0); // 앞앞 PB3 25 26 LF_F left_ForwardForward
	HAL_GPIO_WritePin(LF_B_GPIO_Port, LF_B_Pin, 0); // 앞백 PA10 LF_B left_ForwardBack
}

void timer_start(TIM_HandleTypeDef *htim) { //초기화
	HAL_TIM_Base_Start(htim);
}

void delay_us(uint16_t us, TIM_HandleTypeDef *htim) { // us 딜레이
	__HAL_TIM_SET_COUNTER(htim, 0); // initislize counter to start from 0
	while ((__HAL_TIM_GET_COUNTER(htim)) < us)
		; // wait count until us
}

void trig(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, TIM_HandleTypeDef *htim) { // 트리거
	HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, HIGH);

	delay_us(10, htim);
	HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, LOW);
}

long unsigned int echo(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin,
		TIM_HandleTypeDef *htim) { // 에코 (타임아웃 추가)
	long unsigned int echo = 0;
	uint32_t start_time;
	uint32_t timeout = 30000; // us, 최대 유효 측정시간보다 길게 설정

	start_time = __HAL_TIM_GET_COUNTER(htim);
	while (HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin) == LOW) {
		if (__HAL_TIM_GET_COUNTER(htim) - start_time > timeout) {
			return 0;
		}
	}
	__HAL_TIM_SET_COUNTER(htim, 0);
	while (HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin) == HIGH) {
		if (__HAL_TIM_GET_COUNTER(htim) > timeout) {
			return 0;
		}
	}
	echo = __HAL_TIM_GET_COUNTER(htim);
	if (echo >= 240 && echo <= 23000)
		return echo;
	else
		return 0;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//2- 컴퓨터 - mcu   recive - 컴퓨터 -> mcu / tran - mcu -> 컴퓨터
	//3 - mcu - 핸드폰 / recive - 핸드폰 -> mcu / tran - mcu -> 핸드폰

	if (huart->Instance == USART3) {

//		HAL_UART_Transmit(&huart2, &rx3_data, sizeof(rx3_data), 10);
//		HAL_UART_Receive_IT(&huart3, &rx3_data, sizeof(rx3_data));
//		new_data_flag = 1;              // 새 데이터가 있다고 플래그 설정
//		last_rx_time = HAL_GetTick();

		HAL_UART_Transmit(&huart2, &rx3_data, 1, 10);
		HAL_UART_Receive_IT(&huart3, &rx3_data, 1);

	} else if (huart->Instance == USART2) {

//		HAL_UART_Transmit(&huart3, &rx2_data, sizeof(rx2_data), 10);
//		HAL_UART_Receive_IT(&huart2, &rx2_data, sizeof(rx2_data));

		HAL_UART_Transmit(&huart3, &rx2_data, 1, 10);
		HAL_UART_Receive_IT(&huart2, &rx2_data, 1);

	}
}
#define MEDIAN_FILTER_SIZE 5
int right_history[MEDIAN_FILTER_SIZE];
int left_history[MEDIAN_FILTER_SIZE];
int history_index = 0;

int compare_int(const void *a, const void *b) {
	return (*(int*) a - *(int*) b);
}

int get_median(int history[]) {
	int temp_history[MEDIAN_FILTER_SIZE];
	memcpy(temp_history, history, sizeof(temp_history));
	qsort(temp_history, MEDIAN_FILTER_SIZE, sizeof(int), compare_int);
	return temp_history[MEDIAN_FILTER_SIZE] / 2;

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	//printf("UART3 Receive Test\n");
//	HAL_UART_Receive_IT(&huart3, &rx3_data, sizeof(rx3_data));
//	HAL_UART_Receive_IT(&huart2, &rx2_data, sizeof(rx2_data));
	HAL_UART_Receive_IT(&huart3, &rx3_data, 1);
	HAL_UART_Receive_IT(&huart2, &rx2_data, 1);

	timer_start(&htim1);
	timer_start(&htim2);
	long unsigned int echo_time_right;
	long unsigned int echo_time_left;
	int dist_right;
	int dist_left;

	uint32_t last_sensor_read_time = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (HAL_GetTick() - last_sensor_read_time > 500) {
			last_sensor_read_time = HAL_GetTick(); // 현재 시간 기록

			trig(Trigger_GPIO_Port, Trigger_Pin, &htim1);
			echo_time_right = echo(Echo_GPIO_Port, Echo_Pin, &htim1);

			trig(Trigger2_GPIO_Port, Trigger2_Pin, &htim2);
			echo_time_left = echo(Echo2_GPIO_Port, Echo2_Pin, &htim2);

			if (echo_time_right != 0 || echo_time_left != 0) {
				dist_right = (int) (17 * echo_time_right / 100);
				dist_left = (int) (17 * echo_time_left / 100);
				printf("1: Distance = %d(mm) | 2: Distance = %d(mm)\n",
						dist_right, dist_left);
				if (dist_right <= 100 && dist_left <= 100) {
					smartcar_stop();
				}

			} else {
				printf("Out of Range!\n");
			}




		}
		// 1. 블루투스 제어 확인 (매 루프마다 즉시 확인)
		// 거리 계산 (mm 단위)                                                           │
//		dist_right = (echo_time_right > 0) ?(int) (17 * echo_time_right / 100) : 9999;
//		dist_left =	 (echo_time_left > 0) ? (int) (17 * echo_time_left / 100) : 9999;

		// 필터에 현재 값 추가
		if (echo_time_right > 0) {
				dist_right = (int) (17 * echo_time_right / 100);
			} else {
				dist_right = 9999;
			}
			if (echo_time_left > 0) {
				dist_left = (int) (17 * echo_time_left / 100);
			} else {
				dist_left = 9999;
			}

		right_history[history_index] = dist_right;
		left_history[history_index] = dist_left;
		history_index = (history_index + 1) % MEDIAN_FILTER_SIZE; // 인덱스 순환




		// 안정화된 거리 가져오기
		int final_dist_right = get_median(right_history);
		int final_dist_left = get_median(left_history);
		int obstacle_detected = 0;

		// 자동 정지 로직: 전진 중일 때만 장애물 감지
		if (final_dist_right < 250 || final_dist_left < 250) {
			obstacle_detected = 1;
		}

		if (rx3_data == 'w' && obstacle_detected == 0) {
			smartcar_forward();
		} else if (rx3_data == 'w' && obstacle_detected == 1) {
			smartcar_stop();
		} else if (rx3_data == 'a') {
			smartcar_left();
		} else if (rx3_data == 'd') {
			smartcar_right();
		} else if (rx3_data == 's') {

			smartcar_back();
		} else if (rx3_data == 'p') {
			smartcar_stop();
		}
		// 2. 500ms 마다 초음파 센서 확인 (논블로킹 방식)


	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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

  /* USER CODE END TIM2_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Trigger2_Pin|LR_F_Pin|LF_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LR_B_Pin|LF_F_Pin|RF_F_Pin|RF_B_Pin
                          |RR_F_Pin|RR_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo2_Pin */
  GPIO_InitStruct.Pin = Echo2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Trigger2_Pin LR_F_Pin LF_B_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Trigger2_Pin|LR_F_Pin|LF_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Pin */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LR_B_Pin LF_F_Pin RF_F_Pin RF_B_Pin
                           RR_F_Pin RR_B_Pin */
  GPIO_InitStruct.Pin = LR_B_Pin|LF_F_Pin|RF_F_Pin|RF_B_Pin
                          |RR_F_Pin|RR_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	//2- 컴퓨터 - mcu   recive - 컴퓨터 -> mcu / tran - mcu -> 컴퓨터
//	//3 - mcu - 핸드폰 / recive - 핸드폰 -> mcu / tran - mcu -> 핸드폰
//
//	if (huart->Instance == USART3) {
//
////		HAL_UART_Transmit(&huart2, &rx3_data, sizeof(rx3_data), 10);
////		HAL_UART_Receive_IT(&huart3, &rx3_data, sizeof(rx3_data));
//		new_data_flag = 1;              // 새 데이터가 있다고 플래그 설정
//		last_rx_time = HAL_GetTick();
//
//		HAL_UART_Transmit(&huart2, &rx3_data, 1, 10);
//		HAL_UART_Receive_IT(&huart3, &rx3_data, 1);
//
//	} else if (huart->Instance == USART2) {
//
////		HAL_UART_Transmit(&huart3, &rx2_data, sizeof(rx2_data), 10);
////		HAL_UART_Receive_IT(&huart2, &rx2_data, sizeof(rx2_data));
//
//
//		new_data_flag = 1;              // 새 데이터가 있다고 플래그 설정
//		last_rx_time = HAL_GetTick();
//		HAL_UART_Transmit(&huart3, &rx2_data, 1, 10);
//		HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
//
//	}
//}
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
	while (1) {
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
