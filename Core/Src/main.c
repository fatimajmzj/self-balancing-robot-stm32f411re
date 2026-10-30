/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu9250.h"
#include "kalman.h"
#include <stdio.h>
#include <math.h>
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
static uint16_t prev_contador_u16 = 0;
static uint32_t prev_contador_u32_2 = 0;
float vel=0.0;
float vel_2=0.0;
float setpoint = 0.0;
float error= 0.0;
float sumae = 0.0;
float control=0.0;

volatile float setpoint_2 = 0.0;
float error_2= 0.0;
float sumae_2 = 0.0;
float control_2=0.0;

volatile float setpoint_ang=0.0;
float error_ang= 0.0;
float sumae_ang = 0.0;
float control_ang=0.0;
float error_ang_pas=0.0;
float derivada=0.0;

//Control angulo
float Kp_ang=6.0;
float Kd_ang=0.1;

//Control vel
float Kp=50.0;//50
float Ki=300.0; //500 300 jala bien
float Kd=0.01; //mete mas oscilaciones
volatile float angulo = 0.0f;
float sat = 0.0;

//Control Pos
float pos_1 = 0.0f;   // posición integrada motor 1
float pos_2 = 0.0f;   // posición integrada motor 2
float Kp_pos = 0.1f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* --- Arrancar encoders (TIM1 = Motor1, TIM2 = Motor2) --- */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  /* --- Arrancar PWM --- */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* Motores apagados al inicio */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

  HAL_Delay(200);
  /* Inicialización bloqueante del MPU */
  mpu9250_init();

  accel_data accel;
  gyro_data  gyro;

  /* Leer ángulo inicial para Kalman */
  mpu9250_parse(&accel, &gyro);

  KalmanFilter kf_pitch;
  kalman_init(&kf_pitch, accel.pitch_angle);

  angulo = accel.pitch_angle;

  uint32_t t_anterior   = HAL_GetTick();
  uint32_t muestra      = 0;
  float    pitch_kalman = accel.pitch_angle;

  /* Dispara la primera lectura DMA ANTES de arrancar TIM5.
   * Cuando TIM5 empiece, ya habrá un dato listo o en camino. */
  mpu9250_request_dma();

  /* Arrancar TIM5  */
  HAL_TIM_Base_Start_IT(&htim5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      /*
       * Patrón DMA no-bloqueante:
       *
       * 1. Comprobamos el flag (puesto por HAL_I2C_MemRxCpltCallback).
       * 2. Si hay dato nuevo: parsear + Kalman + actualizar angulo.
       * 3. Inmediatamente disparamos la siguiente lectura DMA.
       * 4. La CPU queda libre para el printf y cualquier otra tarea.
       *
       * La ISR del TIM5 solo lee 'angulo' — nunca espera ni dispara I2C.
       */
      if (mpu_dma_ready)
      {
          mpu_dma_ready = 0;   /* limpiar flag*/

          uint32_t t_ahora = HAL_GetTick();
          float dt = (float)(t_ahora - t_anterior) / 1000.0f;
          t_anterior = t_ahora;

          if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

          /* Convertir buffer raw a structs */
          mpu9250_parse(&accel, &gyro);

          /* Actualizar Kalman */
          pitch_kalman = kalman_update(&kf_pitch,
                                        accel.pitch_angle,
                                        gyro.y_val,
                                        dt);

          /* Actualizar variable global*/
          __disable_irq();
          angulo = pitch_kalman;
          __enable_irq();

          muestra++;

          /* Se dispara siguiente lectura DMA */
          mpu9250_request_dma();
      }

      /*
       * mientras UART transmite, DMA sigue llenando mpu_rx_buf */
      printf("%5lu | Kalman:%7.2f | controlA:%6.1f  controlB:%6.1f\r\n", muestra, pitch_kalman, control, control_2);

  } /* fin while */
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        float ang_local = angulo;

        //============
        // Lectura de encoders
        //============

        // --- Encoder 1 ---
        uint16_t contador_actual = __HAL_TIM_GET_COUNTER(&htim1);
        int16_t delta_cuenta = (int16_t)(contador_actual - prev_contador_u16);
        prev_contador_u16 = contador_actual;
        float deltapos = 0.00261799f * (float)delta_cuenta;
        vel = deltapos / 0.01f;

        // --- Encoder 2 ---
        uint32_t contador_actual_2 = __HAL_TIM_GET_COUNTER(&htim2);
        int32_t delta_cuenta_2 = (int32_t)(contador_actual_2 - prev_contador_u32_2);
        prev_contador_u32_2 = contador_actual_2;
        float deltapos_2 = 0.00261799f * (float)delta_cuenta_2;
        vel_2 = deltapos_2 / 0.01f;

        //===============
        // Acumular posición
        //================
        pos_1 += deltapos;
        pos_2 += deltapos_2;

        // Promedio de posición (Motor2 gira al revés, por eso se resta)
        float pos_promedio = (pos_1 - pos_2) / 2.0f;

        //==========
        // setpoint_ang dinámico
        //============
        setpoint_ang = -Kp_pos * pos_promedio;

        // Saturación
        if (setpoint_ang >  3.0f) setpoint_ang =  3.0f;
        if (setpoint_ang < -3.0f) setpoint_ang = -3.0f;

        //==============
        // Control de angulo (PD)
        //=============
        error_ang = setpoint_ang - ang_local;
        if (fabs(error_ang) <= 0.5f) error_ang = 0.0f;

        derivada = (error_ang - error_ang_pas) / 0.01f;
        error_ang_pas = error_ang;
        control_ang = Kp_ang * error_ang + Kd_ang * derivada;

        // Setpoints de velocidad para cada motor
        if (control_ang > 0.0f) {
            setpoint   = -(control_ang >  8.0f ?  8.0f : control_ang);
            setpoint_2 =  (control_ang >  8.0f ?  8.0f : control_ang);
        } else {
            setpoint   = -(control_ang < -8.0f ? -8.0f : control_ang);
            setpoint_2 =  (control_ang < -8.0f ? -8.0f : control_ang);
        }

        //================
        // Control de velocidad Motor 1
        //=================
        error = setpoint - vel;
        sat = 1000.0f / Ki;
        sumae += error * 0.01f;
        if (sumae >  sat) sumae =  sat;
        if (sumae < -sat) sumae = -sat;
        control = Kp * error + Ki * sumae;

        if (control > 1000.0f)  control =  1000.0f;
        if (control < -1000.0f) control = -1000.0f;

        if (control >= 0.0f) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, control);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0.0f);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0.0f);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, fabs(control));
        }

        //================
        // Control de velocidad Motor 2
        //================
        error_2 = setpoint_2 - vel_2;
        sumae_2 += error_2 * 0.01f;
        if (sumae_2 >  sat) sumae_2 =  sat;
        if (sumae_2 < -sat) sumae_2 = -sat;
        control_2 = Kp * error_2 + Ki * sumae_2;

        if (control_2 > 1000.0f)  control_2 =  1000.0f;
        if (control_2 < -1000.0f) control_2 = -1000.0f;

        if (control_2 >= 0.0f) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, control_2);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0.0f);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0.0f);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, fabs(control_2));
        }
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
