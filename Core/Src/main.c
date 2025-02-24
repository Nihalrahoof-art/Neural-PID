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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_index=0;
uint8_t rx_data[32];
uint8_t rx_buffer[32];
uint8_t transfer_cplt;
uint8_t data_received = 0;


#define PWM_PERIOD_MS 20.0f   // 20ms period (50Hz)
#define PWM_MIN 20.0f         // 20% duty cycle minimum
#define PWM_MAX 100.0f        // 100% duty cycle maximum

#define LEARNING_RATE 0.01f


float cmd_vel = 0.0f;           // Desired velocity (from Jetson)
float imu_vel = 0.0f;           // Current velocity (from Jetson)
float pwm_duty = 0.0f;         // Start with 50% duty cycle
float step_size = 1.0f;         // PWM adjustment step size
float tolerance = 0.5f;         // Acceptable velocity error (km/h)
float error=0.0f;
float more=0.0f;
float less=0.0f;
float equal=0.0f;

static char adjust[10]; // Global/static variable

// PID Constants (TUNE THESE)
float Kp = 1.5f;   // Proportional gain
float Ki = 0.01f;   // Integral gain
float Kd = 0.2f;   // Derivative gain

// PID variables
float prev_error = 0.0f;
float integral = 0.0f;

// NN Weights for Adaptive PID
float w1 = 0.5f, w2 = 0.3f, w3 = 0.2f;
float w4 = 0.4f, w5 = 0.3f, w6 = 0.2f;
float w7 = 0.6f, w8 = 0.4f, w9 = 0.2f;


float h1 = 0;
    float h2 = 0;
    float h3 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);


/* USER CODE BEGIN PFP */
float compute_pid(float error);
void update_pid_gains(float error, float derivative, float integral);

void process_uart_data();
void set_pwm(float duty_cycle);

void set_direction(int direction); // Set motor direction (1 = forward, 0 = reverse)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ======= Fuzzy Membership Functions ======= */
// Neural Activation Function (Sigmoid)
float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

/* ======= PID Controller Function ======= */

// Compute PID Output
float compute_pid(float error) {
    float derivative = error - prev_error;
    integral += error;

    // Update PID gains dynamically using Neural Network
    update_pid_gains(error, derivative, integral);

    // Compute output
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    prev_error = error;

    return output;
}

// Neural Network to Adjust PID Gains
void update_pid_gains(float error, float derivative, float integral) {
    // NN Inputs (Normalized)
    float e_norm = error / 10.0f;
    float de_norm = derivative / 10.0f;
    float Ie_norm = integral / 100.0f;

    // Hidden Layer Neuron Outputs
     h1 = sigmoid(w1 * e_norm + w2 * de_norm + w3 * Ie_norm);
     h2 = sigmoid(w4 * e_norm + w5 * de_norm + w6 * Ie_norm);
     h3 = sigmoid(w7 * e_norm + w8 * de_norm + w9 * Ie_norm);

    // Output Layer (PID Gains)
    Kp = 2.0f * h1;  // Dynamic Kp
    Ki = 0.05f * h2; // Dynamic Ki
    Kd = 0.1f * h3;  // Dynamic Kd

    // Update Weights Using Gradient Descent
    float learning_rate = LEARNING_RATE;
    w1 += learning_rate * error * h1 * (1 - h1);
    w2 += learning_rate * derivative * h2 * (1 - h2);
    w3 += learning_rate * integral * h3 * (1 - h3);
    w4 += learning_rate * error * h1 * (1 - h1);
    w5 += learning_rate * derivative * h2 * (1 - h2);
    w6 += learning_rate * integral * h3 * (1 - h3);
    w7 += learning_rate * error * h1 * (1 - h1);
    w8 += learning_rate * derivative * h2 * (1 - h2);
    w9 += learning_rate * integral * h3 * (1 - h3);
}

void process_uart_data() {
    char *cmd_token = strtok((char *)rx_buffer, ",");
    char *imu_token = strtok(NULL, ",");

    if (cmd_token && imu_token) {
        cmd_vel = atof(cmd_token);
        imu_vel = atof(imu_token);
    }
    memset(rx_buffer, 0, 32);
}


void set_pwm(float duty_cycle) {
	if (duty_cycle < PWM_MIN) duty_cycle = PWM_MIN;
	    if (duty_cycle > PWM_MAX) duty_cycle = PWM_MAX;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (htim3.Init.Period * duty_cycle) / 100);
}

void set_direction(int direction) {
    // Direction control: 0 = reverse, 1 = forward
    if (direction == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // DIR pin LOW for forward
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // DIR pin HIGH for reverse
    }
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

//HAL_UART_Receive_IT(&huart1, rx_data, 32);
HAL_UART_Receive_IT(&huart1, &rx_data[rx_index], 1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Start PWM
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
	  if (data_received) {
	              process_uart_data();
	              data_received = 0;
	          }    /* USER CODE END WHILE */
	  //float error = cmd_vel - imu_vel;

	  	  	  error = fabs(cmd_vel - imu_vel);

	          // Compute PWM using Fuzzy Logic
	          //pwm_duty = fuzzy_pwm(error);
	  	  	  pwm_duty = fabs(compute_pid(error));  // Take absolute value of PID output

	          // Set direction (accelerate or decelerate)
	          if (cmd_vel > imu_vel) {
	              set_direction(0);
	              strcpy(adjust, "0.2\n");
	              	      HAL_Delay(100);
	              	      HAL_UART_Transmit_IT(&huart1, (uint8_t *)adjust, strlen(adjust));
	              	      HAL_Delay(100);
	          } else if (cmd_vel < imu_vel) {
	              set_direction(1);
	              strcpy(adjust, "-0.2\n");
	              	      HAL_Delay(100);
	              	      HAL_UART_Transmit_IT(&huart1, (uint8_t *)adjust, strlen(adjust));
	              	      HAL_Delay(100);
	          }
	          else {  // When cmd_vel == imu_vel
	          	      pwm_duty = 0.0f;  // Stop actuator
	          	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (htim3.Init.Period * pwm_duty) / 100);

	          	      // Send 0 to stop imu_vel changes
	          	      strcpy(adjust, "0\n");
	          	      HAL_Delay(10);
	          	      HAL_UART_Transmit_IT(&huart1, (uint8_t *)adjust, strlen(adjust));
	          	      HAL_Delay(10);
	          	  }

	          // Apply PWM
	          set_pwm(pwm_duty);


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_data[0] == '\n') {
            rx_buffer[rx_index] = '\0';
            data_received = 1;
            rx_index = 0;
        } else {
            rx_buffer[rx_index++] = rx_data[0];
            if (rx_index >= 32) rx_index = 0;
        }
        HAL_UART_Receive_IT(&huart1, &rx_data[0], 1);
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
