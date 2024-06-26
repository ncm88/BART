/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 Nolan McCleary.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pd.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KP 18.5
#define KD 0.6
#define SAMPLE_RATE 1000
#define MOTOR1_DIR_Pin GPIO_PIN_10
#define MOTOR1_DIR_GPIO_Port GPIOA
#define MOTOR2_DIR_Pin GPIO_PIN_11
#define MOTOR2_DIR_GPIO_Port GPIOA

#define LIMIT_SWITCH_1_PIN GPIO_PIN_1
#define LIMIT_SWITCH_2_PIN GPIO_PIN_3
#define LIMIT_SWITCH_ANGLE_DEG 90
#define HOMING_SET_VALUE (LIMIT_SWITCH_ANGLE_DEG / 360 * ENCODER_RESOLUTION)
#define HOMING_POWER_RATIO 0.2

#define ENCODER_RESOLUTION 48960
#define CHUNK_SIZE 8
#define UART_BUFFSIZE CHUNK_SIZE * 3

#define ON 1
#define OFF 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SIGNED_ANGLE(A, RES) ((A > (RES / 2)) ? (A - RES) : (A)) 
#define ARC_VECTOR(TARGET, CURRENT) \
    ((((TARGET - CURRENT) + ENCODER_RESOLUTION) % ENCODER_RESOLUTION <= ENCODER_RESOLUTION / 2) ? \
        ((TARGET - CURRENT) + ENCODER_RESOLUTION) % ENCODER_RESOLUTION : \
        -((ENCODER_RESOLUTION - ((TARGET - CURRENT) + ENCODER_RESOLUTION) % ENCODER_RESOLUTION) % ENCODER_RESOLUTION))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum{
  HOMING,
  RUNNING,
  ERROR_STATE
}BART_STATE;

pd_instance_int16 pd_instance_mot1, pd_instance_mot2;
moving_avg_obj filter_instance1, filter_instance2;

int32_t xPos;
int32_t xOutput;

int32_t yPos;
int32_t yOutput;

float motor1_error_derivative, motor2_error_derivative;

uint16_t xTarg, yTarg; //angular representation of target x and y cartesian coordinates
uint8_t laser;
uint8_t visited;
int32_t deltaX, deltaY;
int32_t xError, yError;
int32_t last_x_error, last_y_error;

uint8_t rx_buff[UART_BUFFSIZE] = {0};

int16_t buffer_index = -8;
uint16_t point_index;

uint8_t x_homed;
uint8_t y_homed;

int16_t xTargTracker;
int16_t yTargTracker;

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  
  Decoder_Init();
  PWM_Init();
  set_pd_gain(&pd_instance_mot1, KP, KD);
  set_pd_gain(&pd_instance_mot2, KP, KD);
  TIM6_manual_init();
  TIM7_manual_init();
  HAL_UART_Receive_DMA(&huart2, rx_buff, UART_BUFFSIZE); //Establish continuous UART DMA

  //INSERT CALIBRATION CODE HERE----------------------------------------
  x_homed = 0;
  y_homed = 0;
  BART_STATE = HOMING;
  //----------------------------------------------------------------------

  homer_subroutine();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(BART_STATE == HOMING){ //redundancy for any interrupt-triggered homing that needs to be added in the future
      x_homed = 0;
      y_homed = 0;
      homer_subroutine();
    }

    xPos = SIGNED_ANGLE(__HAL_TIM_GET_COUNTER(&htim2), ENCODER_RESOLUTION);
    yPos = SIGNED_ANGLE(__HAL_TIM_GET_COUNTER(&htim3), ENCODER_RESOLUTION);

    xTargTracker = SIGNED_ANGLE(xTarg, ENCODER_RESOLUTION);
    yTargTracker = SIGNED_ANGLE(yTarg, ENCODER_RESOLUTION);

    xOutput = pd_instance_mot1.output;
    yOutput = pd_instance_mot2.output;
    
    
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

/**
 * @brief Timer overflow interrupt callback
 * @details Applies PD on timer 6 overflow, updates UART buffer entry on timer 7 overflow
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){   
  
  if(BART_STATE == RUNNING){
    if(htim->Instance == TIM6){
    
      if(laser == ON){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); 
      }
      
      else{
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); 
      }

      xError = ARC_VECTOR(xTarg, __HAL_TIM_GET_COUNTER(&htim2)); 
      yError = ARC_VECTOR(yTarg, __HAL_TIM_GET_COUNTER(&htim3));
      
      //get error delta
      deltaX = xError - last_x_error;
      deltaY = yError - last_y_error;

      //filter on error delta
      apply_average_filter(&filter_instance1, deltaX, &motor1_error_derivative);
      apply_average_filter(&filter_instance2, deltaY, &motor2_error_derivative);

      // apply PD
      apply_pd(&pd_instance_mot1, xError, motor1_error_derivative, SAMPLE_RATE);
      apply_pd(&pd_instance_mot2, yError, motor2_error_derivative, SAMPLE_RATE);

      // PWM and direction control
      if(pd_instance_mot1.output > 0){
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pd_instance_mot1.output);
        HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);
      }
      else{
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, (-1) * pd_instance_mot1.output);
        HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_RESET);
      }

      if(pd_instance_mot2.output > 0){
        __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pd_instance_mot2.output);    
        HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_SET);
      }
      else{
        __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, (-1) * pd_instance_mot2.output);
        HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_RESET);
      }
      
      last_x_error = xError;
      last_y_error = yError;
    }

    else if(htim->Instance == TIM7){
      buffer_index += 8;
      if (buffer_index >= UART_BUFFSIZE) {
        buffer_index -= UART_BUFFSIZE;  // Wrap around explicitly without waiting for the next interrupt
      }
      visited = rx_buff[buffer_index + 5];
      if((buffer_index < UART_BUFFSIZE - 7) && !visited){
        xTarg = rx_buff[buffer_index] | (rx_buff[buffer_index + 1] << 8);
        yTarg = rx_buff[buffer_index + 2] | (rx_buff[buffer_index + 3] << 8);
        laser = rx_buff[buffer_index + 4];
        rx_buff[buffer_index + 5] |= 0x1; //mark this byte as visited
        point_index = rx_buff[buffer_index + 6] | (rx_buff[buffer_index + 7] << 8);
      }
    }
  }
}


//Turn mirrors until limit switch is hit
void homer_subroutine(void){ 
    laser = OFF;
    
    if(!x_homed){
      __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, HOMING_POWER_RATIO * PD_MAX);
    }
    
    if(!y_homed){
      __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, HOMING_POWER_RATIO * PD_MAX);
    }
    
    while(!x_homed || !y_homed)
    {
    }

    BART_STATE = RUNNING;
}


//Limit switch interrupt which triggers when limit switch pins are pulled low
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  
  if(GPIO_Pin == LIMIT_SWITCH_1_PIN){
    TIM2->CNT &= 0x0;
    TIM2->CNT |= HOMING_SET_VALUE;
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
    x_homed = 1;
  }

  else if(GPIO_Pin == LIMIT_SWITCH_2_PIN){
    TIM3->CNT &= 0x0;
    TIM3->CNT |= HOMING_SET_VALUE;
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
    y_homed = 1;
  }

  __set_PRIMASK(primask);
}



//Low-level initializatiion to bypass HAL unreliability
void Decoder_Init(void){
  TIM2->CR1 |= TIM_CR1_CEN;
  TIM2->CR1 |= TIM_CR1_ARPE;
  TIM2->ARR = ENCODER_RESOLUTION - 1;

  TIM3->CR1 |= TIM_CR1_CEN;
  TIM3->CR1 |= TIM_CR1_ARPE;
  TIM3->ARR = ENCODER_RESOLUTION - 1;
}


void PWM_Init(void){
  TIM16->CR1 |= 1;
  TIM16->CCER |= TIM_CCER_CC1E;
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); //PWM pin map function

  TIM17->CR1 |= 1;
  TIM17->CCER |= TIM_CCER_CC1E;
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); 
}


void TIM6_manual_init(){
  TIM6->CR1 |= 1;
  TIM6->DIER |= TIM_DIER_UIE;
}


void TIM7_manual_init(){
  TIM7->CR1 |= 1;
  TIM7->DIER |= TIM_DIER_UIE;
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
  BART_STATE = ERROR_STATE;
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
