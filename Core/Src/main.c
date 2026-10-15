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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "ins.h"
#include "cmsis_gcc.h"
#include "mpu6050.h"
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
uint8_t rx_buf[16] = {0};
uint8_t rx_idx = 0;
//VOFA串口接收
uint8_t uart_rx_buf[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "mpu6050.h"
#include <math.h>

#define MAX_SPEED 999

typedef enum
{
  CONTROL_MODE_SPEED = 0,
  CONTROL_MODE_ANGLE = 1
} control_mode_t;

// 控制模式与角度外环
volatile control_mode_t control_mode = CONTROL_MODE_SPEED;
volatile float target_angle = 0.0f;
volatile float control_angle = 0.0f;
volatile float control_angle_rate = 0.0f;
volatile float angle_target_rpm = 0.0f;
volatile float speed_setpoint = 0.0f;
volatile uint8_t control_reset_requested = 1;

#define ANGLE_KP 4.0f
#define ANGLE_KI 0.4f
#define ANGLE_KD 0.3f
#define ANGLE_LOOP_DT 0.020f
#define ANGLE_INTEGRAL_LIMIT 40.0f
#define ANGLE_SPEED_LIMIT 150.0f
#define ANGLE_TARGET_LIMIT 45.0f
#define ANGLE_SAFETY_LIMIT 75.0f
#define ANGLE_CONTROL_DIRECTION 1.0f // 若角度误差越来越大，改为-1.0f

float angle_err = 0.0f;
float angle_err_sum = 0.0f;

//速度环相关变量
volatile int16_t pulse_delta;
volatile float speed_rpm_raw = 0.0f;
volatile float speed_rpm = 0.0f;
#define MOTOR_PPR 13.0f      // 电机轴单通道每圈脉冲数，请按实际编码器修改
#define REDUCE_RATIO 48.0f
#define ENCODER_MULTIPLIER 4.0f // TIM3使用TI12模式，对AB相进行4倍频计数
#define ENCODER_DIRECTION 1.0f  // 若正转时测速为负，改为-1.0f
#define SPEED_LOOP_DT 0.010f    // TIM4控制周期：10 ms
#define SPEED_FILTER_ALPHA 0.30f

//速度PID
volatile float target_rpm = 0.0f;
#define SPEED_TARGET_LIMIT 200.0f
float sp_kp = 1.8f;
float sp_ki = 0.25f;
float sp_kd = 0.03f;
float sp_err = 0;
float sp_err_last = 0;
float sp_err_sum = 0;
volatile float speed_pid_out = 0;
#define PWM_LIMIT 999.0f
#define INTEGRAL_LIMIT 2000.0f






// 角度外环：输出速度内环的目标转速
float angle_pid_update(float angle, float angle_rate)
{
  angle_err = target_angle - angle;
  angle_err_sum += angle_err * ANGLE_LOOP_DT;

  if(angle_err_sum > ANGLE_INTEGRAL_LIMIT) angle_err_sum = ANGLE_INTEGRAL_LIMIT;
  if(angle_err_sum < -ANGLE_INTEGRAL_LIMIT) angle_err_sum = -ANGLE_INTEGRAL_LIMIT;

  float output = ANGLE_CONTROL_DIRECTION *
                 (ANGLE_KP * angle_err + ANGLE_KI * angle_err_sum - ANGLE_KD * angle_rate);

  if(output > ANGLE_SPEED_LIMIT) output = ANGLE_SPEED_LIMIT;
  if(output < -ANGLE_SPEED_LIMIT) output = -ANGLE_SPEED_LIMIT;
  return output;
}

// TB6612 单电机驱动函数
void motor_set(int16_t speed)
{
  //限幅，防止PWM溢出
  if(speed > MAX_SPEED)
  {
    speed = MAX_SPEED;
  }
  if(speed < -MAX_SPEED)
  {
    speed = -MAX_SPEED;
  }
  if(speed > 0)
  {
    //电机正向
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
  }
  else if(speed < 0)
  {
    //电机反向
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -speed);
  }
  else
  {
    //停止
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
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
  //启动编码器
  //HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  //HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  //启动1ms定时中断跑PID
  //HAL_TIM_Base_Start_IT(&htim4);
  //开启串口中断接收VOFA浮点数
  //HAL_UART_Receive_IT(&huart1, uart_rx_buf,4);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // 上电先保证PWM为0，再使能TB6612，防止电机意外启动
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim4);//
  HAL_UART_Receive_IT(&huart1,&rx_buf[rx_idx],1);//



  ins_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // 发送缓冲区
  char buff[256];
  uint32_t vofa_last_tick = 0;
  // 获取陀螺仪数据的指针
  ins_data_t *ins = ins_data();
  while (1)
  {// 更新 MPU6050 数据（陀螺仪 + 加速度计 + 姿态解算）
    ins_update();

    // I2C读取留在主循环；控制中断只使用最近一次姿态结果
    control_angle = ins->pitch;
    control_angle_rate = ins->gyro[1];

    /*
     * 速度模式：I10直接给定speed_setpoint。
     * 角度模式：I9经角度PID生成speed_setpoint。
     * 两种模式最终都只允许速度PID写PWM。
     */

    // VOFA使用FireWater：50 Hz发送，避免阻塞串口破坏控制时序
    if ((HAL_GetTick() - vofa_last_tick) >= 20U)
    {
      vofa_last_tick = HAL_GetTick();
      sprintf(buff, "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.0f,%.3f,%.3f\n",
              ins->gyro[0], ins->gyro[1], ins->gyro[2],
              ins->gyro_accel[0], ins->gyro_accel[1], ins->gyro_accel[2],
              ins->roll, ins->pitch, ins->yaw, target_angle,
              target_rpm, speed_rpm, speed_pid_out, (int)pulse_delta,
              (float)control_mode, speed_setpoint, angle_target_rpm);
      HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 100);
    }

    HAL_Delay(1);
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
#include <stdio.h>
int fputc(int ch,FILE *f)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,10);
  return ch;
}

int str_prefix_to_float_strict(const char *str, float *out_val)
{
  if (str == NULL || out_val == NULL)
    return -1;

  char *p_colon = strchr(str, ':');
  if (p_colon == NULL)
  {
    *out_val = 0;
    return -1;
  }
  const char *num_start = p_colon + 1;
  char *endptr;

  double tmp = strtod(num_start, &endptr);

  // endptr == num_start 代表没有解析到数字
  if (endptr == num_start)
  {
    *out_val = 0;
    return -1;
  }
  *out_val = (float)tmp;
  return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  if(huart->Instance == USART1)
  {
    uint8_t received_char = rx_buf[rx_idx];

    if(received_char == '\n')
    {
      float received_value;

      /* 用字符串结束符覆盖换行符 */
      rx_buf[rx_idx] = '\0';

      if(str_prefix_to_float_strict(
            (const char *)rx_buf,
            &received_value) == 0)
      {
        /*
         * 判断命令前缀：
         * i9  修改目标角度
         * i10 修改目标转速
         */
        if(strstr((const char *)rx_buf, "I9:") != NULL)
        {
          if(received_value > ANGLE_TARGET_LIMIT) received_value = ANGLE_TARGET_LIMIT;
          if(received_value < -ANGLE_TARGET_LIMIT) received_value = -ANGLE_TARGET_LIMIT;
          target_angle = received_value;
          control_mode = CONTROL_MODE_ANGLE;
          control_reset_requested = 1;
        }
        else if(strstr((const char *)rx_buf, "I10:") != NULL)
        {
          if(received_value > SPEED_TARGET_LIMIT) received_value = SPEED_TARGET_LIMIT;
          if(received_value < -SPEED_TARGET_LIMIT) received_value = -SPEED_TARGET_LIMIT;
          target_rpm = received_value;
          control_mode = CONTROL_MODE_SPEED;
          control_reset_requested = 1;
        }
      }

      /* 清空缓冲区，准备接收下一条命令 */
      memset(rx_buf, 0, sizeof(rx_buf));
      rx_idx = 0;
    }
    else if(received_char == '\r')
    {
      /*
       * 忽略回车。
       * rx_idx 不增加，下一字节会覆盖当前位置。
       */
    }
    else
    {
      rx_idx++;

      /* 防止缓冲区越界 */
      if(rx_idx >= sizeof(rx_buf) - 1)
      {
        memset(rx_buf, 0, sizeof(rx_buf));
        rx_idx = 0;
      }
    }

    /* 始终一次只接收一个字节 */
    HAL_UART_Receive_IT(&huart1, &rx_buf[rx_idx], 1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t angle_loop_divider = 0;

  if(htim->Instance == TIM4)
  {
    //读取脉冲，立刻清零计数器
    pulse_delta = (int16_t) __HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3,0);

    // 根据10 ms采样窗口和AB相4倍频计数计算转速，并低通滤波
    speed_rpm_raw = ENCODER_DIRECTION * (float)pulse_delta * 60.0f /
                    (MOTOR_PPR * ENCODER_MULTIPLIER * REDUCE_RATIO * SPEED_LOOP_DT);
    speed_rpm += SPEED_FILTER_ALPHA * (speed_rpm_raw - speed_rpm);

    if (control_reset_requested)
    {
      sp_err = 0.0f;
      sp_err_last = 0.0f;
      sp_err_sum = 0.0f;
      angle_err = 0.0f;
      angle_err_sum = 0.0f;
      angle_target_rpm = 0.0f;
      speed_setpoint = 0.0f;
      speed_pid_out = 0.0f;
      angle_loop_divider = 0;
      control_reset_requested = 0;
      motor_set(0);
      return;
    }

    if (control_mode == CONTROL_MODE_ANGLE)
    {
      // 姿态超出安全范围时立即停止，避免机构继续失控
      if (fabsf(control_angle) > ANGLE_SAFETY_LIMIT)
      {
        angle_target_rpm = 0.0f;
        speed_setpoint = 0.0f;
        speed_pid_out = 0.0f;
        sp_err_sum = 0.0f;
        motor_set(0);
        return;
      }

      // 速度内环100 Hz，角度外环每两次运行一次，即50 Hz
      angle_loop_divider++;
      if (angle_loop_divider >= 2U)
      {
        angle_loop_divider = 0;
        angle_target_rpm = angle_pid_update(control_angle, control_angle_rate);
      }
      speed_setpoint = angle_target_rpm;
    }
    else
    {
      angle_loop_divider = 0;
      angle_target_rpm = 0.0f;
      speed_setpoint = target_rpm;
    }

    // 速度模式下0 RPM作为明确停车命令；角度模式下仍保留速度环以抵抗扰动
    if (control_mode == CONTROL_MODE_SPEED && fabsf(speed_setpoint) < 0.01f)
    {
      sp_err_sum = 0.0f;
      sp_err_last = 0.0f;
      speed_pid_out = 0.0f;
      motor_set(0);
      return;
    }

    // ----- PID计算 -----
    sp_err = speed_setpoint - speed_rpm;

    // 先计算候选积分；若输出已沿误差方向饱和，则停止继续积分
    float sp_err_sum_candidate = sp_err_sum + sp_err;
    if(sp_err_sum_candidate > INTEGRAL_LIMIT) sp_err_sum_candidate = INTEGRAL_LIMIT;
    if(sp_err_sum_candidate < -INTEGRAL_LIMIT) sp_err_sum_candidate = -INTEGRAL_LIMIT;

    float sp_p = sp_kp * sp_err;
    float sp_d = sp_kd * (sp_err - sp_err_last);
    float output_candidate = sp_p + sp_ki * sp_err_sum_candidate + sp_d;
    if (!((output_candidate > PWM_LIMIT && sp_err > 0.0f) ||
          (output_candidate < -PWM_LIMIT && sp_err < 0.0f)))
    {
      sp_err_sum = sp_err_sum_candidate;
    }

    speed_pid_out = sp_p + sp_ki * sp_err_sum + sp_d;
    sp_err_last = sp_err;

    //输出限幅
    if(speed_pid_out > PWM_LIMIT) speed_pid_out = PWM_LIMIT;
    if(speed_pid_out < -PWM_LIMIT) speed_pid_out = -PWM_LIMIT;

    motor_set((int16_t)speed_pid_out);
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
