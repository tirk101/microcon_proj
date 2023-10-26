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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "max_matrix_stm32.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	GPIO_TypeDef *type;
	uint16_t pin;
	uint8_t value;
}
digitalpin;

typedef struct
{
	digitalpin near;
	digitalpin far;
}
irsensor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t _counter_1ms_tim1 = 0;
uint32_t _counter_1ms_tim3 = 0;
irsensor _ir_sensors[4] =
{
    { .near = { GPIOG, GPIO_PIN_0, 0 }, .far = { GPIOG, GPIO_PIN_1, 0 } },
    { .near = { GPIOD, GPIO_PIN_1, 0 }, .far = { GPIOF, GPIO_PIN_9, 0 } },
	{ .near = { GPIOD, GPIO_PIN_0, 0 }, .far = { GPIOF, GPIO_PIN_7, 0 } },
	{ .near = { GPIOF, GPIO_PIN_0, 0 }, .far = { GPIOF, GPIO_PIN_8, 0 } }
}; // !Direction Note! index 0 = North, index 1 = East, index 2 = South, index 3 = West
digitalpin _common_7segment[4] =
{
	{ GPIOA, GPIO_PIN_15, 0 },
	{ GPIOB, GPIO_PIN_12, 0 },
	{ GPIOB, GPIO_PIN_13, 1 },
	{ GPIOB, GPIO_PIN_15, 1 }
}; // !Direction Note! index 0 = North, index 1 = East, index 2 = South, index 3 = West
   // !Data Note! value 0 = active 1 (Cathode) value 1 = active 0 (Anode)
digitalpin _light_led = { GPIOE, GPIO_PIN_3, 0 };
digitalpin _walk_btn[4][2]=
{

		{ { GPIOB, GPIO_PIN_11, 1 }, { GPIOB, GPIO_PIN_10, 1 } },
		{ { GPIOE, GPIO_PIN_15, 1 }, { GPIOE, GPIO_PIN_14, 1 } },
		{ { GPIOE, GPIO_PIN_12, 1 }, { GPIOE, GPIO_PIN_10, 1 } },
		{ { GPIOD, GPIO_PIN_11, 1 }, { GPIOD, GPIO_PIN_12, 1 } }


}; // !Direction Note! index 0 = North, index 1 = East, index 2 = South, index 3 = West
   // !Data Note! when read pressed = value
digitalpin _walk_buzzer[4] =
{
	{ GPIOF, GPIO_PIN_14, 0 },
	{ GPIOE, GPIO_PIN_9 , 0 },
	{ GPIOE, GPIO_PIN_11, 0 },
	{ GPIOF, GPIO_PIN_14, 0 }
};
uint32_t _7segment_nums[16][2] =
{
	{ 0xF03F0FC0, 0x0FC0F03F }, // 0
	{ 0xFE7F0180, 0x0180FE7F }, // 1
	{ 0xE93F16C0, 0x16C0E93F }, // 2
	{ 0xEC3F13C0, 0x13C0EC3F }, // 3
	{ 0xE67F1980, 0x1980E67F }, // 4
	{ 0xE4BF1B40, 0x1B40E4BF }, // 5
	{ 0xE0BF1F40, 0x1F40E0BF }, // 6
	{ 0xFE3F01C0, 0x01C0FE3F }, // 7
	{ 0xE03F1FC0, 0x1FC0E03F }, // 8
	{ 0xE43F1BC0, 0x1BC0E43F }, // 9
	{ 0xE23F1DC0, 0x1DC0E23F }, // A
	{ 0xE0FF1F00, 0x1F00E0FF }, // b
	{ 0xF1BF0E40, 0x0E40F1BF }, // C
	{ 0xE87F1780, 0x1780E87F }, // d
	{ 0xE1BF1E40, 0x1E40E1BF }, // E
	{ 0xE3BF1C40, 0x1C40E3BF }  // F
};
char _queue_direction[5];
char _walk_direction[5];
char _buzzer_direction[5];
char _current_direction = '\0';
uint8_t _walk_index = 0;
uint8_t _queue_index = 0;
int _check_add_queue[4] = { 0, 0, 0, 0 };
int _check_display_tim[4] = { 0, 0, 0, 0 };
int _walk_display_tim[4] = { 0, 0, 0, 0 };
volatile uint32_t _ldr_val = 0;
int state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ReadIrSensors(void);
void ReadLdrSensor(void);
void AddQueueDirections(void);
void AddQueueWalkDirections(void);
void DisplayIntersection(void);
void DisplayWalkIntersection(void);
void DisplayRoadsidesLight(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void ResetChangeState(void);

//Sub function
int CheckConditionAddQueue(int);
int CheckConditionAddWalkQueue(int);
char ReverseDirection(char);
void Display7Segment(int, int);
void WorkingIntersection(int);
void DoneIntersection(void);
void WorkingWalkIntersection(int);
void DoneWalkIntersection(int);
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
  char temp;
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_ADC_Start(&hadc1);

  HAL_UART_Receive_IT(&huart3, (uint8_t*) &temp, 1);
  // Reset common 7 segment
  for(int i=0; i<4; i++) HAL_GPIO_WritePin(_common_7segment[i].type, _common_7segment[i].pin, (_common_7segment[i].value == 1)? 0: 1);
  for(int i=0; i<4; i++) HAL_GPIO_WritePin(_walk_buzzer[i].type, _walk_buzzer[i].pin, (_walk_buzzer[i].value == 1)?0:1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(state == 0){
		AddQueueWalkDirections();
		DisplayWalkIntersection();

		ReadIrSensors();
		AddQueueDirections();
		DisplayIntersection();
		while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET){}
		HAL_UART_Transmit(&huart3, (uint8_t*) "This is state 0\r\n", strlen("This is state 0\r\n!"), 1000);
	}else if(state == 1){
		for(int i = 0; i< 4; i++) {
			Display7Segment(0, i);
		}
		htim4.Instance -> CCR1 = (10000-1) * 1;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		htim4.Instance -> CCR2 = (10000-1) * 1;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		htim4.Instance -> CCR3 = (10000-1) * 1;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		htim4.Instance -> CCR4 = (10000-1) * 1;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

		while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET){}
		HAL_UART_Transmit(&huart3, (uint8_t*) "This is state 1\r\n", strlen("This is state 1\r\n"), 1000);
	}

	ReadLdrSensor();
	DisplayRoadsidesLight();
//	HAL_UART_Receive(&huart3, (uint8_t*) &temp, 1, 1000);


//	write_char(26, 1);
//	write_char(24, 2);
//	write_char(27, 3);
//
//	write_char(26, 4);
//	write_char(24, 5);
//	write_char(27, 6);
//
//	write_char(26, 7);
//	write_char(24, 8);
//	write_char(27, 9);
//
//	write_char(26, 10);
//	write_char(24, 11);
//	write_char(27, 12);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ReadIrSensors(void){
	for(int i=0; i<4; i++)
	{
		_ir_sensors[i].near.value = (HAL_GPIO_ReadPin(_ir_sensors[i].near.type, _ir_sensors[i].near.pin) == 1) ? 0 : 1;
		_ir_sensors[i].far.value = (HAL_GPIO_ReadPin(_ir_sensors[i].far.type, _ir_sensors[i].far.pin) == 1) ? 0 : 1;
	}
}

void ReadLdrSensor(void){
	while(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); }
	_ldr_val = HAL_ADC_GetValue(&hadc1);
}

void AddQueueDirections(void){
	if (_queue_index == 0) _counter_1ms_tim1 = 0;

	for(int i=0; i<4; i++){
		if (CheckConditionAddQueue(i) == 1) {
			if (_check_add_queue[i] == 0) _check_add_queue[i] = _counter_1ms_tim3;
			if (_counter_1ms_tim3 - _check_add_queue[i] >= 1500)  {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
				_queue_direction[_queue_index] = "NESW"[i];
				_queue_index++;
				_check_add_queue[i] = 0;
			}
		}else _check_add_queue[i] = 0;
	}
}

void AddQueueWalkDirections(void){
	for(int i=0; i<4; i++){
		if(CheckConditionAddWalkQueue(i)){
			_walk_direction[_walk_index] = "NESW"[i];
			_walk_index++;
		}
	}
}

void DisplayIntersection(void){
	if (_queue_index == 0) return;

	for(int i=0 ; i<4; i++){
		if(_queue_direction[0] == "NESW"[i]){
			if( ( strchr(_walk_direction, _queue_direction[0]) == NULL &&
				  strchr(_walk_direction, ReverseDirection(_queue_direction[0])) == NULL
				) ||
				_current_direction == _queue_direction[0] )
			{
				if(_counter_1ms_tim1 < 17000){
					if (_counter_1ms_tim1 < 16000){
						if (_ir_sensors[i].far.value == 1)_check_display_tim[i] = _counter_1ms_tim1;
						else if(_counter_1ms_tim1 - _check_display_tim[i] >= 5000) {
							_counter_1ms_tim1 = 16000;
							_check_display_tim[i] = 0;
						}
						WorkingIntersection(i);
					}
					else HAL_GPIO_WritePin(_common_7segment[i].type, _common_7segment[i].pin, (_common_7segment[i].value == 1)? 0: 1);
				}
				else DoneIntersection();
			}else _counter_1ms_tim1 = 0;
		}
	}
}

void DisplayWalkIntersection(void){
	if (_walk_index == 0) return;
	char *pos;
	for(int i=0 ; i<4; i++){
		pos = strchr(_walk_direction, "NESW"[i]);
		if(pos != NULL){
			if(_walk_direction[pos-_walk_direction] != ReverseDirection(_current_direction) && _walk_direction[pos-_walk_direction] != _current_direction) {
				if(_walk_display_tim[i] == 0) _walk_display_tim[i] = _counter_1ms_tim3;
				if(_counter_1ms_tim3 - _walk_display_tim[i] < 5000) {
					if(_counter_1ms_tim3 - _walk_display_tim[i] > 3000) {
						if(i == 0) {
							htim4.Instance -> CCR1 = (10000-1) * 0.5;
							HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
						}else if(i == 1) {
							htim4.Instance -> CCR2 = (10000-1) * 0.5;
							HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
						}else if(i == 2) {
							htim4.Instance -> CCR3 = (10000-1) * 0.5;
							HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
						}else if(i == 3) {
							htim4.Instance -> CCR4 = (10000-1) * 0.5;
							HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
						}
					} else WorkingWalkIntersection(i);
				} else {
					DoneWalkIntersection(i);
					_walk_display_tim[i] = 0;
				}
			} else _walk_display_tim[i] = _counter_1ms_tim3;
		}
	}
}

void DisplayRoadsidesLight(void){
	HAL_GPIO_WritePin(_light_led.type, _light_led.pin, (_ldr_val < 3000) ? 1 : 0);
}

void ResetChangeState(void){
	_queue_direction[5] = '\0';
	_walk_direction[5] = '\0';
	_buzzer_direction[5] = '\0';
	_current_direction = '\0';
	_walk_index = 0;
	_queue_index = 0;
	for (int i=0; i<4; i++){
		_check_add_queue[i] = 0;
		_check_display_tim[i] = 0;
		_walk_display_tim[i] = 0;
	}


	while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET){}
	HAL_UART_Transmit(&huart3, (uint8_t*) "RESET!\r\n", strlen("RESET!\r\n"), 1000);
}

//Sub function
int CheckConditionAddQueue(int index){
	// !Direction Note! Use upper case
	return (_ir_sensors[index].far.value == 1 &&
			_ir_sensors[index].near.value != 1 &&
			_current_direction == '\0' &&
			strchr(_queue_direction, "NESW"[index]) == NULL) ? 1 : 0;
}

int CheckConditionAddWalkQueue(int index){
	// !Direction Note! Use upper case
	return ((HAL_GPIO_ReadPin(_walk_btn[index][0].type, _walk_btn[index][0].pin) == _walk_btn[index][0].value  ||
			 HAL_GPIO_ReadPin(_walk_btn[index][1].type, _walk_btn[index][1].pin) == _walk_btn[index][1].value) &&
			 strchr(_walk_direction, "NESW"[index]) == NULL) ? 1 : 0;
}

char ReverseDirection(char c){
	for(int i=0; i<4; i++){
		if(c == "NESW"[i]) return "SWNE"[i];
	}
	return '\0';
}

void Display7Segment(int num, int com){
	// !PIN Note! GPIOC PIN 6 - PIN 12 -> a-g
	HAL_GPIO_WritePin(_common_7segment[com].type, _common_7segment[com].pin, _common_7segment[com].value);

	GPIOC -> BSRR = _7segment_nums[num][_common_7segment[com].value];
}

void WorkingIntersection(int direction){
	// !Direction Note! index 0 = North, index 1 = East, index 2 = South, index 3 = West
	_current_direction = "NESW"[direction];
	Display7Segment(15 - _counter_1ms_tim1/1000, direction);
}

void DoneIntersection(void){
	_counter_1ms_tim1 = 0;
	char *new_direction = _queue_direction + 1;
	strcpy(_queue_direction, new_direction);
	_queue_index--;
	_current_direction = '\0';
}

void WorkingWalkIntersection(int direction){
	if(direction == 0) {
		htim4.Instance -> CCR1 = (10000-1) * 1;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	}else if(direction == 1) {
		htim4.Instance -> CCR2 = (10000-1) * 1;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	}else if(direction == 2) {
		htim4.Instance -> CCR3 = (10000-1) * 1;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	}else if(direction == 3) {
		htim4.Instance -> CCR4 = (10000-1) * 1;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	}
	HAL_GPIO_WritePin(_walk_buzzer[direction].type, _walk_buzzer[direction].pin, _walk_buzzer[direction].value);
}

void DoneWalkIntersection(int direction){
	if(direction == 0) {
		htim4.Instance -> CCR1 = (10000-1) * 0;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	}else if(direction == 1) {
		htim4.Instance -> CCR2 = (10000-1) * 0;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	}else if(direction == 2) {
		htim4.Instance -> CCR3 = (10000-1) * 0;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	}else if(direction == 3) {
		htim4.Instance -> CCR4 = (10000-1) * 0;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	}

	HAL_GPIO_WritePin(_walk_buzzer[direction].type, _walk_buzzer[direction].pin, (_walk_buzzer[direction].value == 1)?0:1);
	char *pos = strchr(_walk_direction, "NESW"[direction]);
	memmove(pos, pos + 1, strlen(pos));
	_walk_index--;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart3) {
//		char temp;
//		HAL_UART_Receive_IT(&huart3, (uint8_t*) &temp, 1);
//		if (state == 0 && temp == '1') {
//			ResetChangeState();
//			state = 1;
//		}
//		else if (state == 1 && temp == '0') {
//			ResetChangeState();
//			state = 0;
//		}

	}
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
