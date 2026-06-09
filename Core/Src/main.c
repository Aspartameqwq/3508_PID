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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_kinematics.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 控制任务调度周期（ms）�?? */
#define CONTROL_PERIOD_MS 10U

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
static void CAN_Filter_Init(void);

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  DR16_Init();
  CAN_Filter_Init();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  Control_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_control_tick = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    debug_main_loop_count++;

    /* 转圈触发：Ozone 设 debug_run_turns 非零→所有在线电机执行一次→自动清零。
     * Debug_QuickRunTurns 内部已按电机逐台检查 feedback_valid，无需外层检查。 */
    if (debug_run_turns != 0.0f)
    {
      float turns = debug_run_turns;
      debug_run_turns = 0.0f;
      Debug_QuickRunTurns(turns);
    }

    /* ====== 快捷调试模式调度 ====== */
    if (debug_quick_mode == DEBUG_QUICK_MODE_REMOTE)
    {
      /* ====== 遥控器模式（全向轮 X 型底盘运动学解算） ======
       *
       * 摇杆映射：
       *   左摇杆 ch2（左右）→ X 平移    右摇杆 ch0（左右）→ 旋转
       *   左摇杆 ch3（上下）→ Y 平移    右摇杆 ch1（上下）→ 预留
       *
       * 信号丢失保护：
       *   DR16_Init() 已将 RC_CtrlData 初始化为中位值（各通道=1024），
       *   因此上电后、未收到第一帧时，摇杆读数为中位 → 车速为零。
       *   同时检查 last_frame_tick：若从未收到帧或超时未更新 → 强制归零。
       *   超时阈值 debug_dr16_signal_timeout_ms（默认 500 ms）。 */
      {
        uint32_t now_tick = HAL_GetTick();
        uint8_t  signal_lost = 0U;
        uint8_t  i;

        /* 信号丢失判断：从未收到帧，或距最后一帧超过超时阈值 */
        if (debug_dr16_last_frame_tick == 0U ||
            (now_tick - debug_dr16_last_frame_tick) > debug_dr16_signal_timeout_ms)
        {
          signal_lost = 1U;
        }

        if (signal_lost)
        {
          /* 遥控器信号丢失 → 所有电机目标转速归零，确保安全 */
          for (i = 0; i < MOTOR_COUNT; i++)
          {
            MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_SPEED);
            MotorControl_SetTargetSpeed(i, 0.0f);
          }
        }
        else
        {
          float chassis_vx, chassis_vy, chassis_omega;
          float motor_speeds[4];

          /* 步骤 1：遥控器通道 → 底盘目标运动（m/s 和 rad/s） */
          ChassisKinematics_RemoteToChassis(
              RC_CtrlData.rc.ch0, RC_CtrlData.rc.ch1,
              RC_CtrlData.rc.ch2, RC_CtrlData.rc.ch3,
              &chassis_vx, &chassis_vy, &chassis_omega);

          /* 步骤 2：底盘目标运动 → 4 电机目标转速（rad/s，电机轴侧） */
          ChassisKinematics_ChassisToMotors(
              chassis_vx, chassis_vy, chassis_omega, motor_speeds);

          /* 步骤 3：写入每个电机的目标转速，同步 debug_set_speed 供 Ozone 观察 */
          for (i = 0; i < MOTOR_COUNT; i++)
          {
            debug_set_speed[i] = motor_speeds[i];
            MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_SPEED);
            MotorControl_SetTargetSpeed(i, motor_speeds[i]);
          }
        }
      }
    }
    else if (debug_quick_mode == DEBUG_QUICK_MODE_SPEED)
    {
      /* 速度环模式：每个电机独立读取 debug_set_speed[i]。 */
      uint8_t i;
      for (i = 0; i < MOTOR_COUNT; i++)
      {
        MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_SPEED);
        MotorControl_SetTargetSpeed(i, debug_set_speed[i]);
      }
    }
    else
    {
      /* 位置环模式（默认回退）：每个电机独立读取 debug_set_position[i]。 */
      uint8_t i;
      for (i = 0; i < MOTOR_COUNT; i++)
      {
        MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_POSITION);
        MotorControl_SetTargetAngle(i, debug_set_position[i]);
      }
    }

    uint32_t now = HAL_GetTick();
    if ((now - last_control_tick) >= CONTROL_PERIOD_MS)
    {
      float dt_s = (float)(now - last_control_tick) * 0.001f;
      last_control_tick = now;
      Control_Task(dt_s);
    }
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef can_filter = {0};

  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hcan->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    return;
  }

  debug_can_rx_raw_count++;
  debug_last_rx_stdid = rx_header.StdId;

  Control_OnCanRxMessage(&rx_header, rx_data);
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
