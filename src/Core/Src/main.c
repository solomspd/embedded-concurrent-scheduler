/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdlib.h>
#include "../../scheduler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTERNAL 0
#define DEMO1 1
#define DEMO2 2
#define TASK_SET DEMO1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);
void wrap_around(int *x,int wrap_val);
struct task* get_tail(struct queue *que);
struct task* get_head(struct queue *que);
struct task* que_pop(struct queue *que);
struct task* que_pop_back(struct queue *que);
void swap_task(struct task **a, struct task **b);
void queue_push_back(struct queue *que, struct task *new_task);
void QueTask(void (*func_in)(void), uint8_t priority_in);
void ReRunMe(unsigned int delay_in);
void Deque(void);
void init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// TODO propper boudary and error checking when queue overflows

//// variable declarations

struct queue rdy_que, delay_que;
struct task *cur_task;

void wrap_around(int *x, int wrap_val) { // boiler plate to make queue circular
	if (*x < 0) {
		*x = wrap_val - 1;
	} else if (*x == wrap_val) {
		*x = 0;
	}
}

struct task* get_tail(struct queue *que) {
	return que->que[que->tail];
}

struct task* get_head(struct queue *que) {
	return que->que[que->head];
}

struct task* queue_pop(struct queue *que) {
	struct task *ret = que->que[que->head++];
	ret->ref_count--;
	wrap_around(&(que->head), que->max);
	que->len--;
	return ret;
}

struct task* que_pop_back(struct queue *que) {
	struct task *ret = que->que[que->tail--];
	ret->ref_count--;
	wrap_around(&(que->head), que->max);
	que->len--;
	return ret;
}

void swap_task(struct task **a, struct task **b) {
	struct task *temp = *a;
	*a = *b;
	*b = temp;
}

void queue_push_back(struct queue *que, struct task *new_task) {
	new_task->ref_count++;
	que->que[que->tail] = new_task;
	que->len++;
	//if (que->tail == que->head) return; // Queue overflow. should never reach this state.
	int i, ii;
	for (i = que->tail; i != que->head; --i) {
		wrap_around(&i, que->max);
		ii = i - 1;
		wrap_around(&ii, que->max);
		if (que->que[i]->prio < que->que[ii]->prio) {
			swap_task(&que->que[i], &que->que[ii]);
		} else {
			break;
		}
	}
	que->tail++;
	wrap_around(&que->tail, que->max);
}

void QueTask(void (*func_in)(void), uint8_t priority_in) {
	struct task *new_task = malloc(sizeof(struct task));
	new_task->func = func_in;
	new_task->prio = priority_in;
	new_task->ref_prio = priority_in;
	new_task->ref_count = 0;
	queue_push_back(&rdy_que, new_task);
}

void ReRunMe(unsigned int delay_in) {
	if (delay_in ==  0) {
		cur_task->prio = cur_task->ref_prio;
		queue_push_back(&rdy_que, cur_task);
	} else {
		cur_task->prio = delay_in;
		queue_push_back(&delay_que, cur_task);
	}
}

void Deque() {
	if (rdy_que.len > 0) {
		cur_task = queue_pop(&rdy_que);
		cur_task->func();
		if (cur_task->ref_count == 0) {
			free(cur_task);
		}
	}
}

void init() {
	rdy_que.head = 0;
	rdy_que.tail = 0;
	rdy_que.max = MAX_N_TASKS;
	rdy_que.len = 0;
	delay_que.head = 0;
	delay_que.tail = 0;
	delay_que.max = MAX_N_TASKS;
	delay_que.len = 0;
	HAL_SYSTICK_Config(SystemCoreClock/20);
}


//// TASKS FOR INTERNAL TESTING ////
void taskA() {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	ReRunMe(10);
}

int taskb_cnt = 0;
void taskB() {
	uint8_t buf[10] = {'0','0','0','0','0','0','0','0','0','0'};
	int temp = taskb_cnt;
	int i;
	for (i = 9; temp > 0 && i >= 0; i--) {
		buf[i] = temp%10+'0';
		temp /= 10;
	}
	HAL_UART_Transmit(&huart2, buf, sizeof(buf), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, "\n\r", 3, HAL_MAX_DELAY);
	taskb_cnt++;
	ReRunMe(6);
}
////////////////////////////////////

void 

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
  /* USER CODE BEGIN 2 */
	init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	#if TASK_SET == INTERNAL
	QueTask(taskA, 1);
	QueTask(taskB, 4);
	#elif TASK_SET == DEMO1
	QueTask();
	#elif TASK_SET == DEMO2
	
	#endif
  while (1)
  {
		Deque();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
