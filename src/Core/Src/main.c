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
#include <assert.h>
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
#define TASK_SET DEMO2
#define DEBUG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
void wrap_around(int *x,int wrap_val);
struct task* get_tail(struct queue *que);
struct task* get_head(struct queue *que);
struct task* que_pop(struct queue *que);
struct task* que_pop_back(struct queue *que);
void swap_task(struct task **a, struct task **b);
void queue_push_back(struct queue *que, struct task *new_task);
void eq_tasks(struct task *a, struct task *b);
void init_que(struct queue *que);

void QueTask(void (*func_in)(void), uint8_t priority_in);
void ReRunMe(unsigned int delay_in);
void Deque(void);
void init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// TODO propper boudary and error checking when queue overflows

//// Queue variable declarations

struct queue rdy_que, delay_que;
struct task *cur_task;

struct task* get_tail(struct queue *que) {
	int tmp = que->tail - 1;
	wrap_around(&tmp, que->max);
	return que->que[tmp];
}

struct task* get_head(struct queue *que) {
	return que->que[que->head];
}

void wrap_around(int *x, int wrap_val) { // boiler plate to make queue circular
	if (*x < 0) {
		*x = wrap_val - 1;
	} else if (*x == wrap_val) {
		*x = 0;
	}
}

struct task* queue_pop(struct queue *que) { // pop front of queue and return it
	struct task *ret = que->que[que->head++];
	ret->ref_count--;
	wrap_around(&(que->head), que->max);
	que->len--;
	return ret;
}

struct task* que_pop_back(struct queue *que) { // pop back of queueu and return it
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

void init_que(struct queue *que) {
	que->head = 0;
	que->tail = 0;
	que->max = MAX_N_TASKS;
	rdy_que.len = 0;
}

void init() {
	init_que(&rdy_que);
	init_que(&delay_que);
	HAL_SYSTICK_Config(SystemCoreClock/20);
}

void eq_tasks(struct task *a, struct task *b) {
	uint8_t ret = 0xFF;
	ret &= a->prio == b->prio;
	ret &= a->ref_prio == b->ref_prio;
	ret &= a->func == b->func;
	ret &= a->ref_count == b->ref_count;
	return ret;
}

//// TASKS FOR INTERNAL TESTING ////

void tgl_led() {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}

void led_off() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

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

void unit_tests() {
	
	int unit_int = -1;
	wrap_around(&unit_int, MAX_N_TASKS);
	assert(unit_int == MAX_N_TASKS-1);
	unit_int = MAX_N_TASKS;
	assert(unit_int == 0);
	
	struct queue unit_que;
	init_que(&unit_que);
	assert(unit_que.head == 0);
	assert(unit_que.tail == 0);
	assert(unit_que.max == MAX_N_TASKS);
	assert(unit_que.len == 0);
	
	struct task unit_task;
	unit_task.prio = 1;
	unit_task.ref_prio = 1;
	unit_task.func = tgl_led;
	unit_task.ref_count = 1;
	
	assert(eq_tasks(&unit_task, &unit_task));
	
	queue_push_back(&unit_que, &unit_task);
	assert(eq_tasks(&unit_task, get_tail(unit_que)));
	assert(eq_tasks(&unit_task, get_head(unit_que)));
	
	assert(eq(tasks(&unit_task, queue_pop(&unit_que));
	
	LL_SYSTICK_DisableIT();	// disable systick interrupt so as delay que does not interfere
	QueTask(tgl_led, 1);
	assert(eq_tasks(&unit_task, get_tail(rdy_que)));
	assert(eq_tasks(&unit_task, get_head(rdy_que)));
	QueTask(led_off, 2);
	assert(eq_tasks(&unit_task, get_head(rdy_que));
	unit_task.prio = 2;
	unit_task.ref_prio = 2;
	unit_task.func = led_off;
	unit_task.ref_count = 1;
	assert(eq_tasks(&unit_task, get_head(rdy_que));
	LL_SYSTICK_EnableIT(); // enable systick interrupt again
	
	
}

////////////////////////////////////

//// DEMO 1 (temperature sensor) TASKS ////
uint8_t thresh = 0;
uint16_t cur_temp = 0;
void read_temp() {
	uint8_t buf[7] = {0,0,'.',0,0,'\n','\r'};
	uint8_t tmp;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, (uint8_t[]){0x0E, 0x3C}, 2, 10); // refresh temperature
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, (uint8_t[]){0x11}, 1, 10); // request current temperature
	HAL_I2C_Master_Receive(&hi2c1, 0xD1, &tmp, 1, 10); // read current temperature
	cur_temp = tmp << 2;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, (uint8_t[]){0x12}, 1, 10); // request current temperature fraction
	HAL_I2C_Master_Receive(&hi2c1, 0xD1, &tmp, 1, 10); // read current temperature fraction
	cur_temp |= tmp >> 6;
	
	#ifdef DEBUG
	tmp = cur_temp;
	cur_temp = cur_temp >> 2;
	buf[0] = cur_temp/10 + '0';
	buf[1] = cur_temp%10 + '0';
	cur_temp = tmp & 0x3;
	buf[3] = (cur_temp*25)/10 + '0';
	buf[4] = (cur_temp*25)%10 + '0';
	HAL_UART_Transmit(&huart2, buf, sizeof(buf), HAL_MAX_DELAY);
	#endif
	
	ReRunMe(600); // 30/0.05
}

void trig_alarm() {
	if (cur_temp > thresh) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	}
	ReRunMe(5);
}

uint8_t temp_buf [8];
int temp_loc = 0;
void set_temp_thresh() {
	HAL_UART_Receive(&huart2, temp_buf+temp_loc, 1, 10);
	HAL_UART_Transmit(&huart2, temp_buf+temp_loc, 1, 10);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	if (temp_buf[temp_loc] == '\n') {
		int tmp = 0;
		int sig = 1;
		while (temp_buf[temp_loc] != '.') {
			tmp = sig * (temp_buf[temp_loc--] - '0');
			sig *= 10;
		}
		thresh = tmp/25 + 1;
		tmp = 0;
		sig = 1;
		temp_loc--; // skip decimal point
		while (temp_loc > 0) {
			tmp = sig * (temp_buf[temp_loc--] - '0');
			sig *= 10;
		}
		thresh |= tmp << 2;
		temp_loc = 0;
	} else {
		temp_loc++;
	}
}
	
///////////////////////////////////////////////////////////

//// DEMO 2 (distance) TASKS ////

int dist = 0;
void read_dist() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	int time = 0;
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {}
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) { time++; }
	dist = time/100;
	
	#ifdef DEBUG
	char buf[10] = {0,0,0,0,0,0,0,0,0,0};
	int i;
	for (i = 9; time > 0 && i > 0; i--) {
		buf[i] = time%10 + '0';
		time /= 10;
	}
	HAL_UART_Transmit(&huart2, buf, 10, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, "\n\r", 3, HAL_MAX_DELAY);
	#endif
	
	ReRunMe(5);
}

void beep() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	if (dist <= 0) {
		dist = 1;
	} else if (dist >= 20) {
		dist = 20;
	}
	ReRunMe(dist);
}

///////////////////////////////////////////////////////////

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	#if TASK_SET == INTERNAL
	QueTask(taskA, 1);
	QueTask(taskB, 4);
	#elif TASK_SET == DEMO1
	QueTask(read_temp, 2);
	QueTask(trig_alarm, 3);
	//QueTask(set_temp_thresh, 2);
	#elif TASK_SET == DEMO2
	QueTask(read_dist, 1);
	QueTask(beep, 2);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
