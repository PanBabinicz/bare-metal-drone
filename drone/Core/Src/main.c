/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "mpu6050.h"
#include "fst6.h"
#include "pid.h"
#include "math.h"

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
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *DWT_LAR      = (volatile unsigned int *)0xE0001FB0; //address of the register
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC; //address of the register

uint8_t start;
int16_t throttle, cal_int;
int16_t esc_1, esc_2, esc_3, esc_4;
int32_t acc_total_vector;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float roll_level_adjust, pitch_level_adjust;
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
  int32_t start1;
  int32_t end1;
  int32_t end2;
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(1000);

  mpu6050_init(&mpu6050_1byte_data, &mpu6050_i2c_result);
  mpu6050_gyro_calibrate();
  mpu6050_accel_calibrate();

  if (mpu6050_i2c_result == HAL_OK) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  }

  *SCB_DEMCR |= 0x01000000;
  *DWT_LAR = 0xC5ACCE55; // unlock
  *DWT_CYCCNT = 0; // reset the counter
  *DWT_CONTROL |= 1 ; // enable the counter

  // TIM5->CCR1 = 1900;
  // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  TIM5->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
  TIM5->CR2 = 0;
  TIM5->SMCR = 0;
  TIM5->DIER = 0;
  TIM5->EGR = 0;
  TIM5->CCMR1 = (0b110 << 4) | TIM_CCMR1_OC1PE |(0b110 << 12) | TIM_CCMR1_OC2PE;
  TIM5->CCMR2 = (0b110 << 4) | TIM_CCMR2_OC3PE |(0b110 << 12) | TIM_CCMR2_OC4PE;
  TIM5->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM5->PSC = 95;
  TIM5->ARR = 5000;
  TIM5->DCR = 0;
  TIM5->CCR1 = 1000;

  start = 0;

  // pid_p_gain_roll = 0.3;
  // pid_i_gain_roll = 0.00005;
  // pid_d_gain_roll = 2.5;

  //pid_p_gain_roll = 0.6;
  //pid_i_gain_roll = 0.0001;
  //pid_d_gain_roll = 6.5;
  //pid_max_roll = 400;

  //pid_p_gain_pitch = pid_p_gain_roll;
  //pid_i_gain_pitch = pid_i_gain_roll;
  //pid_d_gain_pitch = pid_d_gain_roll;
  //pid_max_pitch = 400;

  // pid_p_gain_yaw = 1.0;
  // pid_i_gain_yaw = 0.0005;
  // pid_d_gain_yaw = 0.0;

  //pid_p_gain_yaw = 2.5;
  //pid_i_gain_yaw = 0.000006;
  //pid_d_gain_yaw = 0.0;
  //pid_max_yaw = 400;



  pid_p_gain_roll = 1.1;
  pid_i_gain_roll = 0.0004;
  pid_d_gain_roll = 14.5;
  pid_max_roll = 400;

  pid_p_gain_pitch = pid_p_gain_roll;
  pid_i_gain_pitch = pid_i_gain_roll;
  pid_d_gain_pitch = pid_d_gain_roll;
  pid_max_pitch = 400;

  // pid_p_gain_yaw = 1.0;
  // pid_i_gain_yaw = 0.0005;
  // pid_d_gain_yaw = 0.0;
  pid_p_gain_yaw = 3.5;
  pid_i_gain_yaw = 0.0002;
  pid_d_gain_yaw = 0.0;
  pid_max_yaw = 400;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE END WHILE */
    *DWT_CYCCNT = 0;
    start1 = *DWT_CYCCNT;

    mpu6050_gyro_read(mpu6050_callback_read, mpu6050_6byte_data, &mpu6050_i2c_result);
    mpu6050_accel_read(mpu6050_callback_read, mpu6050_6byte_data, &mpu6050_i2c_result);

    /* 65.5 = 1 deg/s */
    gyro_input_roll = (gyro_input_roll * 0.7) + (((float)gyro_x_raw / 65.5) * 0.3);     /**< Gyro input for pid is in deg/sec. */
    gyro_input_pitch = (gyro_input_pitch * 0.7) + (((float)gyro_y_raw / 65.5) * 0.3);   /**< Gyro input for pid is in deg/sec. */
    gyro_input_yaw = (gyro_input_yaw * 0.7) + (((float)gyro_z_raw / 65.5) * 0.3);       /**< Gyro input for pid is in deg/sec. */

    /* Gyro angle calculations --> 0.0000611 = 1 / (250Hz / 65.5) */
    angle_roll  += (float)gyro_x_raw * 0.0000611;
    angle_pitch += (float)gyro_y_raw * 0.0000611;

    /* 0.000001066 = 0.0000611 * (pi / 180deg) */
    angle_roll  -= angle_pitch * sin((float)gyro_z_raw * 0.000001066);
    angle_pitch += angle_roll * sin((float)gyro_z_raw * 0.000001066);

    /* Accelerometer angle calculations */
    acc_total_vector = sqrt((accel_x_raw * accel_x_raw) + (accel_y_raw * accel_y_raw) + (accel_z_raw * accel_z_raw));

    if (abs(accel_x_raw) < acc_total_vector) {
      angle_roll_acc = asin((float)accel_x_raw / acc_total_vector) * 57.296;
    }
    if (abs(accel_y_raw) < acc_total_vector) {
      angle_pitch_acc = asin((float)accel_y_raw / acc_total_vector) * 57.296;
    }

    angle_roll  = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;

    roll_level_adjust   = angle_roll * 15;
    pitch_level_adjust  = angle_pitch * 15; 

    if (channel_3 < 1050 && channel_4 < 1050) {
      start = 1;
    }

    if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {
      start = 2;

      angle_roll  = angle_roll_acc;
      angle_pitch = angle_pitch_acc;

      roll_level_adjust = 0;
      pitch_level_adjust = 0;

      pid_last_error_roll = 0;
      pid_last_error_pitch = 0;
      pid_last_error_yaw = 0;

      pid_current_error_roll = 0;
      pid_current_error_pitch = 0;
      pid_current_error_yaw = 0;
    }

    if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
      start = 0;
    }

    pid_setpoint_roll = 0;
    if (channel_1 > 1508) {
      pid_setpoint_roll = channel_1 - 1508;
    } else if (channel_1 < 1492) {
      pid_setpoint_roll = channel_1 - 1492;
    }

    pid_setpoint_roll -= roll_level_adjust;
    pid_setpoint_roll /= 3.0;

    pid_setpoint_pitch = 0;
    if (channel_2 > 1508) {
      pid_setpoint_pitch = channel_2 - 1508;
    } else if (channel_2 < 1492) {
      pid_setpoint_pitch = channel_2 - 1492;
    }

    pid_setpoint_pitch -= pitch_level_adjust;
    pid_setpoint_pitch /= 3.0;

    pid_setpoint_yaw = 0;
    if (channel_3 > 1050) {
      if (channel_4 > 1508) {
        pid_setpoint_yaw = channel_4 - 1508;
      } else if (channel_4 < 1492) {
        pid_setpoint_yaw = channel_4 - 1492;
      }
    }

    pid_setpoint_yaw /= 3.0;
    
    pid_calculate();
    throttle = channel_3;

    if (start == 2) {
      if (throttle > 1800) {
        throttle = 1800;
      }

      esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
      esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
      esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
      esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

      //esc_1 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
      //esc_2 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
      //esc_3 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
      //esc_4 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;

      if (esc_1 < 1100) {
        esc_1 = 1100;
      }
      if (esc_2 < 1100) {
        esc_2 = 1100;
      }
      if (esc_3 < 1100) {
        esc_3 = 1100;
      }
      if (esc_4 < 1100) {
        esc_4 = 1100;
      }

      if (esc_1 > 2000) {
        esc_1 = 2000;
      }
      if (esc_2 > 2000) {
        esc_2 = 2000;
      }
      if (esc_3 > 2000) {
        esc_3 = 2000;
      }
      if (esc_4 > 2000) {
        esc_4 = 2000;
      }
    } else {
      esc_1 = 1000;
      esc_2 = 1000;
      esc_3 = 1000;
      esc_4 = 1000;
    }

    TIM5->CCR1 = esc_1;
    TIM5->CCR2 = esc_2;
    TIM5->CCR3 = esc_3;
    TIM5->CCR4 = esc_4;

    TIM5->CNT = 5000;

    end1 = *DWT_CYCCNT - start1;

    while (*DWT_CYCCNT - start1 < 400000);
    end2 = *DWT_CYCCNT - start1;
    /* USER CODE BEGIN 3 */
    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
