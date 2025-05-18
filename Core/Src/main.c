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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*task_cb)(void); // Function pointer for tasks

typedef struct {
    uint32_t period;       // Time between runs (in ticks)
    uint8_t suspended;     // 0 = not suspended, 1 = suspended
    task_cb task_func;     // Pointer to the task function
    uint32_t last_run;     // Last time this task ran
    uint32_t run_count;    // Number of times task has run
} task_control_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_COUNT 4 // D1, D4, UART, Idle
#define D1_TASK 0
#define D4_TASK 1
#define UART_TASK 2
#define IDLE_TASK 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
task_control_t task_control[TASK_COUNT]; // Task queue array
uint8_t left_display = 0;      // 7-segment left counter (0-9)
uint8_t right_display = 0;     // 7-segment right counter (0-5)
volatile uint8_t both_suspended = 0; // Flag for both tasks suspended
char rx_buffer[20]; // UART receive buffer
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void D1_Task(void);
void D4_Task(void);
void UART_Task(void);
void Idle_Task(void);
void Scheduler_Dispatch(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize task queue
  task_control[D1_TASK].period = 700;     // D1 every 700ms
  task_control[D1_TASK].suspended = 0;
  task_control[D1_TASK].task_func = D1_Task;
  task_control[D1_TASK].last_run = 0;
  task_control[D1_TASK].run_count = 0;

  task_control[D4_TASK].period = 1500;    // D4 every 1500ms
  task_control[D4_TASK].suspended = 0;
  task_control[D4_TASK].task_func = D4_Task;
  task_control[D4_TASK].last_run = 0;
  task_control[D4_TASK].run_count = 0;

  task_control[UART_TASK].period = 2000;  // UART every 2000ms
  task_control[UART_TASK].suspended = 0;
  task_control[UART_TASK].task_func = UART_Task;
  task_control[UART_TASK].last_run = 0;
  task_control[UART_TASK].run_count = 0;

  task_control[IDLE_TASK].period = 100;   // Idle runs every 100ms
  task_control[IDLE_TASK].suspended = 0;
  task_control[IDLE_TASK].task_func = Idle_Task;
  task_control[IDLE_TASK].last_run = 0;
  task_control[IDLE_TASK].run_count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Scheduler_Dispatch(); // Run the scheduler
    HAL_Delay(1);        // Small delay to allow SysTick to update
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  /* USER CODE BEGIN USART2_Init_0 */

  /* USER CODE END USART2_Init_0 */

  /* USER CODE BEGIN USART2_Init_1 */

  /* USER CODE END USART2_Init_1 */
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
  /* USER CODE BEGIN USART2_Init_2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 10); // Start UART receive
  /* USER CODE END USART2_Init_2 */
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

  /*Configure GPIO pin Output Level for D4 (PB3 assumed) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin (S1 on PC13) */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 (S2 assumed) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin (D1 on PA5) */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 (D4 assumed) */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_14; // S3 on PC14
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        char task_id;
        uint32_t period;
        if (sscanf(rx_buffer, "%c %lu", &task_id, &period) == 2) {
            if (task_id == 'T' && period >= 500 && period <= 3000) {
                if (rx_buffer[1] == '1') task_control[D1_TASK].period = period;
                else if (rx_buffer[1] == '4') task_control[D4_TASK].period = period;
                else if (rx_buffer[1] == 'U') task_control[UART_TASK].period = period;
            }
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 10);
    }
}

void D1_Task(void) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Toggle D1 (LD2)
}

void D4_Task(void) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); // Toggle D4 (PB3)
}

void UART_Task(void) {
    if (!both_suspended) {
        char msg[50];
        sprintf(msg, "uwTick: %lu\r\n", uwTick);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
    }
}

void Idle_Task(void) {
    if (!both_suspended) {
        left_display = (left_display + 1) % 10; // 0-9
        right_display = (right_display + 1) % 6; // 0-5
        char msg[50];
        sprintf(msg, "Idle - Display: %d%d, uwTick: %lu\r\n", left_display, right_display, uwTick);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
    }
}

void Scheduler_Dispatch(void)
{
    task_cb task = NULL;
    int task_found = 0;

    for (int i = 0; i < TASK_COUNT - 1; i++) { // Skip idle task
        uint32_t elapsed = uwTick - task_control[i].last_run;
        if (elapsed >= task_control[i].period && task_control[i].suspended == 0) {
            task = task_control[i].task_func;
            task_control[i].last_run = uwTick;
            task_control[i].run_count++;
            if (i == UART_TASK && task_control[i].run_count >= 10) {
                task_control[i].suspended = 1; // Suspend UART after 10 runs
            }
            task_found = 1;
            break;
        }
    }

    if (!task_found) {
        uint32_t elapsed = uwTick - task_control[IDLE_TASK].last_run;
        if (elapsed >= task_control[IDLE_TASK].period) {
            task = task_control[IDLE_TASK].task_func;
            task_control[IDLE_TASK].last_run = uwTick;
        }
    }

    if (task != NULL) {
        task(); // Execute the task
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin) { // S1 (PC13)
        task_control[D1_TASK].suspended = !task_control[D1_TASK].suspended;
        if (!both_suspended) {
            char msg[50];
            sprintf(msg, "D1 %s\r\n", task_control[D1_TASK].suspended ? "suspended" : "resumed");
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
        }
    }
    else if (GPIO_Pin == GPIO_PIN_0) { // S2 (PA0)
        task_control[D4_TASK].suspended = !task_control[D4_TASK].suspended;
        if (!both_suspended) {
            char msg[50];
            sprintf(msg, "D4 %s\r\n", task_control[D4_TASK].suspended ? "suspended" : "resumed");
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
        }
    }
    else if (GPIO_Pin == GPIO_PIN_14) { // S3 (PC14)
        task_control[D1_TASK].period += 300;
        if (task_control[D1_TASK].period > 3000)
            task_control[D1_TASK].period = 500;
    }
    if (task_control[D1_TASK].suspended && task_control[D4_TASK].suspended) {
        both_suspended = 1;
    } else {
        both_suspended = 0;
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
