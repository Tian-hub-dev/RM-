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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	speed_0=0,
	speed_1,
	speed_2,
	speed_3
	
}SPEED;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define time_ms_s 100.0f   
#define pid_time 0.01f

//#define speed_0 0.0f
//#define speed_1 1.0f
//#define speed_2 2.0f
//#define speed_3 3.0f

#define KP 80.0f
#define KI 5.0f
#define KD 2.0f

#define encoder 13.0f
#define gear_ratio   20.0f   //电机参数


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float target_speed = speed_0;  
float current_speed = 0.0f;         
float pwm_output = 0.0f;

uint32_t count_encoder=0;
uint32_t count_last_encoder=0;
int32_t count_diff=0;

float pid_e=0;
float pid_i=0;
float pid_d=0;
float last_pid_e=0;

SPEED speed = speed_0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void calculate_speed (void )
{
	    count_encoder = __HAL_TIM_GET_COUNTER(&htim1);
        count_diff =count_encoder -count_last_encoder ;
		count_last_encoder =count_encoder ;
		
		float pulse_per_rev =encoder*gear_ratio;
		float pulse_per_sec = count_diff  * time_ms_s;
		current_speed =pulse_per_sec / pulse_per_rev; 
}
void Position_PID(void )
{
	pid_e=target_speed -current_speed ;
	if(pid_i<500&&pid_i>-300)
	{
		pid_i+=pid_e*pid_time;
	}
	else if(pid_i>500)pid_i=500;
	else if(pid_i<-300)pid_i=-300;
	
	pid_d = (pid_e - last_pid_e);
	pwm_output = KP * pid_e + KI * pid_i + KD * pid_d;
		
	last_pid_e=pid_e;
	if (pwm_output < 0.0f) pwm_output = 0.0f;
    if (pwm_output > 999.0f) pwm_output = 999.0f;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
HAL_GPIO_WritePin (STBY_GPIO_Port ,STBY_Pin ,GPIO_PIN_SET );
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   if(HAL_GPIO_ReadPin (KEY1_GPIO_Port ,KEY1_Pin )==GPIO_PIN_SET )
	  {
		  HAL_Delay (10);
		   if(HAL_GPIO_ReadPin (KEY1_GPIO_Port ,KEY1_Pin )==GPIO_PIN_SET )
	   {
		   HAL_GPIO_WritePin (AIN1_GPIO_Port ,AIN1_Pin ,GPIO_PIN_SET );
	       HAL_GPIO_WritePin (AIN2_GPIO_Port ,AIN2_Pin ,GPIO_PIN_RESET );
		   
		   speed = (speed + 1) % 4;

           switch(speed)
			   {
              case 0: target_speed = 0.0f; break;
              case 1: target_speed = 1.0f ; break;
              case 2: target_speed = 2.0f ; break;
              case 3: target_speed = 3.0f ; break;
}
		    
		    while(HAL_GPIO_ReadPin (KEY1_GPIO_Port ,KEY1_Pin )==GPIO_PIN_SET );
	   }
	  }
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim2 )
    {
      
		calculate_speed ();
		Position_PID();
		
		if (pwm_output < 0) pwm_output = 0;
        if (pwm_output > 1000) pwm_output = 1000;
		
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)pwm_output);

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
#ifdef USE_FULL_ASSERT
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
