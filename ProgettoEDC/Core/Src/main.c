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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PSX_Controller.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "logging.h"
#include "encoder.h"
#include "pid.h"

extern uint8_t PSX_Rx_Buffer[9];
controller_t PSX;

circular_buffer_t myBuff;
record_t r;

double Ts = 0.005;
int cycleduration;
uint32_t ticControlStep;
uint32_t tocControlStep;
uint32_t controlComputationDuration;
uint32_t k_controller = -1;
int samplingPrescaler = 4;
int samplingPrescalerCounter = 0;


encoder_t encoder_RF;
encoder_t encoder_LF;
encoder_t encoder_RR;
encoder_t encoder_LR;


PID_t pid_RF;
PID_t pid_LF;
PID_t pid_RR;
PID_t pid_LR;
PID_t pid_FAx;
PID_t pid_RAx;


double lastTicks = 0;
double currentTicks = 0;

int8_t barrier;

uint8_t prec_MOD=0;
uint8_t prec_X=0;
double last_setpoint=0.0;
uint8_t ASC =0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WAITING 2

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

  size_t bufferSize = (size_t)ceil(2 / (Ts * samplingPrescaler));
  cb_init(&myBuff, bufferSize, sizeof(record_t));

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM11_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  PSX_Init();

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim9);


  /*
   * PI's structs initialization
   */
  PID_Init(&pid_RF,0.054164,0.0071, 0);
  PID_Init(&pid_LF,0.059952,0.0067, 0);
  PID_Init(&pid_RR,0.066368,0.0075, 0);
  PID_Init(&pid_LR,0.056354,0.0069, 0);

  PID_Init(&pid_FAx,0.059952,0.0067, 0);
  PID_Init(&pid_RAx,0.056354,0.0069, 0);

  /*
   * Encoder's structs initialization
   */
  encoder_init(&encoder_RF,&htim1);
  encoder_init(&encoder_LF,&htim4);
  encoder_init(&encoder_RR,&htim3);
  encoder_init(&encoder_LR,&htim5);


  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1975);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1975);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 1975);
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 1975);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Set up the barrier
	  barrier = 0;

	  /*
	   * Save prec states of various variables
	   */
	  prec_MOD = PSX.PSX_MOD;
	  prec_X = PSX.PSX_X;
	  last_setpoint = PSX.speed;

	  /*
	   * Read the PS2 Controller's output
	   */
	  DATA_DIGEST(PSX_Rx_Buffer);

	  /*
	   * Set references for all the PI controllers
	   */
	  pid_RF.setpoint = PSX.speed;
      pid_LF.setpoint = PSX.speed;
	  pid_RR.setpoint = PSX.speed;
	  pid_LR.setpoint = PSX.speed;
	  pid_FAx.setpoint = 0.0;
  	  pid_RAx.setpoint = 0.0;

  	  /*
  	   * Check for MOD change or ASC enable/disable command from the PS2 Controller
  	   */
  	  if(prec_MOD != PSX.PSX_MOD){
  		switchKcontrol (&pid_RF, &pid_LF, &pid_RR, &pid_LR, &pid_FAx, &pid_RAx);
  	  }

  	  /*
  	   * Stop here until the CALLBACK function does not change the value of barrier
  	   */
	  while(!barrier);

	  /*
	   * Print the collected data
	   */
	  size_t nEntriesToSend = myBuff.count; 					//number of samples not read yet
	  record_t retrieved; 										//buffer entry

	  for (size_t count = 0; count < nEntriesToSend; count++) {
	 		  cb_pop_front(&myBuff, &retrieved); 				//take entry from the buffer

	 		  printf("RF: %f, %f LF: %f, %f RR: %f, %f LR: %f, %f FAx: %f, %f RAx: %f, %f\n\r",
	 				  retrieved.current_u_rf, retrieved.current_y_rf, retrieved.current_u_lf, retrieved.current_y_lf,
	 				  retrieved.current_u_rr, retrieved.current_y_rr, retrieved.current_u_lr, retrieved.current_y_lr,
	 				  retrieved.current_u_rax, retrieved.current_y_rax, retrieved.current_u_fax, retrieved.current_y_fax); // Comment this line out to avoid printing the axle values

	 		  /*printf("RF: %f  LF: %f  RR: %f  LR: %f   Ref: %f\n\r",
	 				  retrieved.current_y_rf, retrieved.current_y_lf,
	 				  retrieved.current_y_rr, retrieved.current_y_lr, PSX.speed);*/
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance== TIM9) {
		/*
		 * Collect first timestamp (in ticks)
		 */
		k_controller = k_controller + 1;
		if (k_controller == 0) {
			ticControlStep = HAL_GetTick();
		}
		tocControlStep = HAL_GetTick();

		/* READING PHASE */
		//Encoder Right Front
		encoder_RF.current_tick = (double) __HAL_TIM_GET_COUNTER(&htim1);
		encoder_RF.speed = getSpeedByDelta(getTicksDelta(encoder_RF.current_tick, encoder_RF.last_tick, Ts), Ts);
		encoder_RF.last_tick = encoder_RF.current_tick;

		//Encoder Left Front
		encoder_LF.current_tick = (double) __HAL_TIM_GET_COUNTER(&htim4);
		encoder_LF.speed = getSpeedByDelta(getTicksDelta(encoder_LF.current_tick, encoder_LF.last_tick, Ts), Ts);
		encoder_LF.last_tick = encoder_LF.current_tick;

		//Encoder Right Rear
		encoder_RR.current_tick = (double) __HAL_TIM_GET_COUNTER(&htim3); //take current value of ticks counting the encoder edges
		encoder_RR.speed = getSpeedByDelta(getTicksDelta(encoder_RR.current_tick, encoder_RR.last_tick, Ts), Ts);
		encoder_RR.last_tick = encoder_RR.current_tick;

		//Encoder Left Rear
		encoder_LR.current_tick = (double) __HAL_TIM_GET_COUNTER(&htim5); //take current value of ticks counting the encoder edges
		encoder_LR.speed = getSpeedByDelta(getTicksDelta(encoder_LR.current_tick, encoder_LR.last_tick, Ts), Ts);
		encoder_LR.last_tick = encoder_LR.current_tick;

		/* COMPUTING PHASE */
		PID_Compute(&pid_RF, encoder_RF.speed);									// Compute Front Right PI output
		PID_Compute(&pid_LF, encoder_LF.speed);									// Compute Front Left PI output

		PID_Compute(&pid_RR, encoder_RR.speed);									// Compute Rear Right PI output
		PID_Compute(&pid_LR, encoder_LR.speed);									// Compute Rear Left PI output

		PID_Compute(&pid_FAx, encoder_LF.speed-encoder_RF.speed);				// Compute Front Axle PI output
		PID_Compute(&pid_RAx, encoder_LR.speed-encoder_RR.speed);				// Compute Rear Axle PI output

		/* SENDING PHASE */

		PWM_generate(pid_RF.u, &htim2, TIM_CHANNEL_1);							// Send Front Right PWM wave
		PWM_generate(pid_LF.u+pid_FAx.u, &htim2, TIM_CHANNEL_2);				// Send Front Left PWM wave

		PWM_generate(pid_RR.u, &htim11, TIM_CHANNEL_1);							// Send Rear Right PWM wave
		PWM_generate(pid_LR.u+pid_RAx.u, &htim10, TIM_CHANNEL_1);				// Send Rear Left PWM wave


		/* SENDING PHASE WITHOUT PI CONTROLLERS */
/*
		PWM_generate(rpm_to_voltage(PSX.speed), &htim11, TIM_CHANNEL_1);
		PWM_generate(rpm_to_voltage(PSX.speed), &htim10, TIM_CHANNEL_1);
		PWM_generate(rpm_to_voltage(PSX.speed), &htim2, TIM_CHANNEL_2);
		PWM_generate(rpm_to_voltage(PSX.speed), &htim2, TIM_CHANNEL_1);
*/

		/*
		 * Collect second timestamp (in ticks)
		 */
		controlComputationDuration = HAL_GetTick() - tocControlStep;
		/*
		 * Collect data
		 */
		r.current_u_rf = pid_RF.u;
		r.current_y_rf = encoder_RF.speed;
		r.current_u_lf = pid_LF.u;
		r.current_y_lf = encoder_LF.speed;
		r.current_u_rr = pid_RR.u;
		r.current_y_rr = encoder_RR.speed;
		r.current_u_lr = pid_LR.u;
		r.current_y_lr = encoder_LR.speed;
		r.current_u_fax = pid_FAx.u;
		r.current_y_fax = encoder_RF.speed-encoder_LF.speed;
		r.current_u_rax = pid_RAx.u;
		r.current_y_rax = encoder_RR.speed-encoder_LR.speed;
		r.cycleCoreDuration = controlComputationDuration;
		r.cycleBeginDelay = tocControlStep - ticControlStep - (k_controller * Ts * 1000);
		r.currentTimestamp = HAL_GetTick();

		/*
		 * Insert data into the buffer
		 */
		if (samplingPrescalerCounter == (samplingPrescaler - 1)) {
			cb_push_back(&myBuff, &r);
			samplingPrescalerCounter = -1;
		}
		samplingPrescalerCounter++;

		/*
		 * Release the barrier
		 */
		barrier = 1;
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
