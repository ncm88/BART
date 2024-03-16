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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "pid.h"
#include "filter.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM_7_MAX_TICKS
#define SCALE_FACTOR 3.3/4096
#define ON 1
#define OFF 0
#define ADC_BUF_LEN 4096
#define MOTOR_VEL_REFERENCE 60
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ABS_DIFF(A, B)((B > A)? (B - A) : (TIM_7_MAX_TICKS - (A - B)))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
encoder_instance enc_instance_mot1, enc_instance_mot2;
static pid_instance_int16 pid_instance_mot1, pid_instance_mot2;
moving_avg_obj filter_instance1, filter_instance2, sampleFilter;

ema_obj m2_ema;


char msg[150];
volatile bool uart_flag;

uint16_t adc_buf[ADC_BUF_LEN];

float motor1_vel, motor2_vel;
float m2_vel_ema;

int32_t encoder_position;
uint16_t timer_counter;
uint16_t latest_adc_value;
uint16_t filtered_adc_value;

uint16_t isrStart = 1;
uint16_t isrEnd = 1;
uint16_t mlStart = 1;
uint16_t mlEnd = 1;
uint16_t ISR_Utilization;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  HAL_UART_MspInit(&huart2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  TIM3_Init();




  /* USER CODE END 2 */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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

/* USER CODE BEGIN 4 */

void DMATransferComplete(DMA_HandleTypeDef* hdma){
  huart2.Instance->CR3 &= ~USART_CR3_DMAT; 
}


//motor 1: encoder-htim2, output-htim16
//motor 2: encoder-htim3, output-htim17
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){   //predefined function override

  if(htim->Instance == TIM6){
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //Blink test wrapper

    //get position delta
	  update_encoder(&enc_instance_mot1, &htim2);
    update_encoder(&enc_instance_mot2, &htim3);

	  //apply average velocity filter
	  apply_average_filter(&filter_instance1, enc_instance_mot1.velocity, &motor1_vel);
    apply_average_filter(&filter_instance2, enc_instance_mot2.velocity, &motor2_vel);


	  if(pid_instance_mot1.d_gain != 0 || pid_instance_mot1.p_gain != 0 || pid_instance_mot1.i_gain != 0){
		  // PID apply
		  apply_pid(&pid_instance_mot1, MOTOR_VEL_REFERENCE - motor1_vel, SAMPLE_RATE);

		  // PWM
		  if(pid_instance_mot1.output > 0){
			  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pid_instance_mot1.output);
			  HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);
		  }
		
      else{
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, (-1) * pid_instance_mot1.output);
		    HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_RESET);
		  }
	  }

    if(pid_instance_mot2.d_gain != 0 || pid_instance_mot2.p_gain != 0 || pid_instance_mot2.i_gain != 0){
		  // PID apply
		  apply_pid(&pid_instance_mot2, MOTOR_VEL_REFERENCE - motor2_vel, SAMPLE_RATE);

		  // PWM
		  if(pid_instance_mot2.output > 0){
			  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pid_instance_mot2.output);
			  HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);
		  }
		
      else{
        __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, (-1) * pid_instance_mot2.output);
		    HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_RESET);
		  }
	  }

    else __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 2000);
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //End of test wrapper
    
    isrEnd = TIM7->CNT; //get end time
  }
}



void TIM3_Init(void){
  TIM3->CR1 |= 1;
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
