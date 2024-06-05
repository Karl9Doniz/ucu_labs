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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lcd5110.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

LCD5110_display lcd1;
volatile uint32_t tim10_overflows = 0;
void TIM10_reinit()
{
 HAL_TIM_Base_Stop(&htim10);
 __HAL_TIM_SET_PRESCALER( &htim10, (95) );
 __HAL_TIM_SET_COUNTER( &htim10, 0 );
 tim10_overflows = 0;
 HAL_TIM_Base_Start_IT(&htim10);
}

void init_timing()
{


 TIM10_reinit();

}

uint32_t get_tim10_us()
{
 __HAL_TIM_DISABLE_IT(&htim10, TIM_IT_UPDATE);
 uint32_t res = tim10_overflows * 10000 + __HAL_TIM_GET_COUNTER(&htim10);
 __HAL_TIM_ENABLE_IT(&htim10, TIM_IT_UPDATE);
 return res;
}

void udelay_TIM10(uint32_t useconds) {
 uint32_t before = get_tim10_us();
 while( get_tim10_us() < before+useconds){}
}

#define USE_HAL_DELAY_AND_ASM 1
#define USE_DWT_TIMING 1
#define __HAL_TIM_SET_PRESCALER(__HANDLE__, __PRESC__)((__HANDLE__)->Instance->PSC = (__PRESC__))


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM10 )
  {
   ++tim10_overflows;
  }
}





#define USE_TIM10_TIMING 1



uint32_t get_us()
{

 return get_tim10_us();

}

void udelay(uint32_t useconds)
{

 udelay_TIM10(useconds);

}

typedef enum state_t {
 IDLE_S,
 TRIGGERING_S,
 WAITING_FOR_ECHO_START_S,
 WAITING_FOR_ECHO_STOP_S,
 TRIG_NOT_WENT_LOW_S,
 ECHO_TIMEOUT_S,
 ECHO_NOT_WENT_LOW_S,
 READING_DATA_S,
 ERROR_S
} state_t;

volatile state_t state = IDLE_S;

volatile uint32_t echo_start;
volatile uint32_t echo_finish;
volatile uint32_t measured_time;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if (GPIO_Pin == ECHOI_Pin)
 {
  switch (state) {
  case WAITING_FOR_ECHO_START_S: {
   echo_start =  get_us();
   state = WAITING_FOR_ECHO_STOP_S;
   break;
  }
  case WAITING_FOR_ECHO_STOP_S: {
   echo_finish = get_us();
   measured_time = echo_finish - echo_start;
   state = READING_DATA_S;
   break;
  }
  default:
   state = ERROR_S;
  }
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
  MX_SPI1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  lcd1.hw_conf.spi_handle = &hspi1;
    lcd1.hw_conf.spi_cs_pin =  LCD1_CS_Pin;
    lcd1.hw_conf.spi_cs_port = LCD1_CS_GPIO_Port;
    lcd1.hw_conf.rst_pin =  LCD1_RST_Pin;
    lcd1.hw_conf.rst_port = LCD1_RST_GPIO_Port;
    lcd1.hw_conf.dc_pin =  LCD1_DC_Pin;
    lcd1.hw_conf.dc_port = LCD1_DC_GPIO_Port;
    lcd1.def_scr = lcd5110_def_scr;
    LCD5110_init(&lcd1.hw_conf, LCD5110_NORMAL_MODE, 0x40, 2, 3);
//
//    LCD5110_print("Hello world!\n", BLACK, &lcd1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  init_timing();
  while (1)
  {
	 char result[80];

	 HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	 udelay_TIM10(16);
	 HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	 state = WAITING_FOR_ECHO_START_S;

	 while( state == WAITING_FOR_ECHO_START_S && state != ERROR_S )
	    {}
	 if ( state == ERROR_S )
	 {
		 LCD5110_print("Error on start", BLACK, &lcd1);
		 LCD5110_refresh(&lcd1.hw_conf);
		 LCD5110_clear_scr(&lcd1.hw_conf);
	     continue;
	 }
	 while( state == WAITING_FOR_ECHO_STOP_S && state != ERROR_S )
	    {}
	 if ( state == ERROR_S )
	 {
		 LCD5110_print("Error on finish", BLACK, &lcd1);
		 LCD5110_refresh(&lcd1.hw_conf);
		 LCD5110_clear_scr(&lcd1.hw_conf);
	     continue;
	 }
	 if (measured_time > 4000) {
	 		 LCD5110_print("Too far\n", BLACK, &lcd1);
	 		 HAL_Delay(1000);
	 		 LCD5110_refresh(&lcd1.hw_conf);
	 		 LCD5110_clear_scr(&lcd1.hw_conf);
	 		 continue;
	 }
	 uint32_t distance = measured_time/58;


	 sprintf(result, "Time:\n %lu us\nDistance:\n %lu cm\n", measured_time, distance);
	 LCD5110_print(result, BLACK, &lcd1);
	 HAL_Delay(1000);
	 LCD5110_refresh(&lcd1.hw_conf);
	 LCD5110_clear_scr(&lcd1.hw_conf);


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
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
