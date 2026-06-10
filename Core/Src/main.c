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
#include <math.h>    /* sqrtf — 底盘级反向制动中计算速度矢量模 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 控制任务调度周期（ms） */
#define CONTROL_PERIOD_MS 10U

/*
 * 制动硬停阶段时长（控制任务迭代次数），宏定义见 debug.h。
 *
 * 时间换算：CONTROL_PERIOD_MS=10ms → BRAKE_HS_COUNT=300ms/10ms=30 次迭代。
 * 修改时长只需改 debug.h 中的 BRAKE_HS_COUNT，例如：
 *   150ms → 150/10 = 15    500ms → 500/10 = 50
 *
 * 每电机独立状态见下方 static 变量。
 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/** @brief 进入制动区时的 debug_control_task_count 快照（底盘级，非逐电机）。
 *   用于反向制动阶段的计时：当前控制迭代次数与入口计数之差
 *   小于动态制动次数时，施加底盘级反向制动。 */
static uint32_t brake_entry_tick = 0U;

/** @brief 上一轮迭代是否处于制动区（底盘级，边沿检测用）。
 *   用于在进入制动区的上升沿记录制动起始时刻。 */
static uint8_t  brake_in_zone = 0U;

/** @brief 进入制动区时的底盘实际平移速度大小（m/s）。
 *   在制动上升沿由前向运动学反算，用于计算与速度成正比的制动时长。 */
static float    brake_entry_speed_ms = 0.0f;

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

          /* 步骤 1：遥控器通道 → 底盘目标运动（m/s 和 rad/s）。
           * RemoteToChassis() 仅做纯摇杆→速度映射（含死区），不做硬件补偿。 */
          ChassisKinematics_RemoteToChassis(
              RC_CtrlData.rc.ch0, RC_CtrlData.rc.ch1,
              RC_CtrlData.rc.ch2, RC_CtrlData.rc.ch3,
              &chassis_vx, &chassis_vy, &chassis_omega);

          /* 步骤 2：硬件补偿（仅作用于遥控器解算出的速度矢量，制动等其他路径不受影响）。
           *
           * 补偿分两项，按先后顺序执行：
           *   2a. 速度矢量偏向角修正 — 将 (vx, vy) 旋转 debug_kinematics_bias_angle_deg 度。
           *       用于矫正因轮系安装不对称导致的方向偏移（如推正前车往左前偏）。
           *       正值=CCW 旋转，负值=CW 旋转，设 0 禁用。
           *   2b. CCW 偏航修正 — 根据平移速度大小，叠加与平移成正比的旋转角速度。
           *       用于矫正因机械不对称（四电机出力不均、重心偏移）导致的直行偏航。
           *       omega += gain × sqrt(vx² + vy²)，正值=CCW，设 0 禁用。
           *
           *       对角死区：当速度矢量方向接近纯对角线方向（45°、135°、225°、315°），
           *       即与一三象限或二四象限角平分线的夹角 < debug_kinematics_ccw_dead_angle_deg
           *       （默认 15°）时，跳过 CCW 修正，避免对角线推杆时过旋转。
           *
           * 两项补偿独立可调，Ozone 中修改后立即生效。 */
          if (debug_kinematics_bias_angle_deg != 0.0f)
          {
            float bias_rad = debug_kinematics_bias_angle_deg * (3.14159265f / 180.0f);
            float cos_b = cosf(bias_rad);
            float sin_b = sinf(bias_rad);
            float vx_rot = chassis_vx * cos_b - chassis_vy * sin_b;
            float vy_rot = chassis_vx * sin_b + chassis_vy * cos_b;
            chassis_vx = vx_rot;
            chassis_vy = vy_rot;
          }

          if (debug_kinematics_ccw_correction_gain != 0.0f)
          {
            float speed_mag = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
            uint8_t apply_ccw = 1U;

            /* 对角死区：速度矢量接近 45° 对角线方向时跳过 CCW 修正 */
            if (debug_kinematics_ccw_dead_angle_deg > 0.0f && speed_mag > 0.005f)
            {
              float angle = atan2f(chassis_vy, chassis_vx);
              if (angle < 0.0f) angle += 2.0f * 3.14159265f;
              /* 将所有方向映射到 [0, π/2)，对角线位于 π/4 (45°) */
              float mod = angle;
              while (mod >= 3.14159265f / 2.0f) mod -= 3.14159265f / 2.0f;
              float dist = (mod > 3.14159265f / 4.0f)
                  ? (mod - 3.14159265f / 4.0f)
                  : (3.14159265f / 4.0f - mod);
              if (dist < debug_kinematics_ccw_dead_angle_deg * (3.14159265f / 180.0f))
              {
                apply_ccw = 0U;
              }
            }

            if (apply_ccw)
            {
              chassis_omega += debug_kinematics_ccw_correction_gain * speed_mag;
            }
          }

          /* 步骤 3：底盘目标运动（已补偿）→ 4 电机目标转速（rad/s，电机轴侧） */
          ChassisKinematics_ChassisToMotors(
              chassis_vx, chassis_vy, chassis_omega, motor_speeds);

          /* 步骤 4：写入各电机目标，同步 debug_set_speed 供 Ozone 观察。
           *
           * 制动策略（底盘级两段式，无位置环）：
           *   摇杆中位 → 所有电机解算转速低于 debug_brake_threshold_rad_s → 进入制动区。
           *
           *   段 0 — 底盘级反向制动（动态时长，与入口速度成正比）：
           *          上升沿：前向运动学反算底盘实际 (vx, vy) → 记录入口速度大小
           *          brake_entry_speed_ms，制动次数 = 入口速度 × debug_brake_duration_per_ms
           *          （默认 25 次每 m/s，1 m/s → 250ms，2 m/s → 500ms），
           *          上限由 debug_brake_max_duration 截断（默认 80 次 = 800ms）。
           *
           *          每迭代：前向运动学从实际电机转速反算底盘 (vx, vy, omega) →
           *          施加与运动方向相反的小速度（线速度+角速度）→
           *          逆运动学分解为一致的四电机目标转速。
           *
           *          线速度强度：debug_brake_reverse_speed_ms（默认 0.1 m/s）
           *          角速度强度：debug_brake_reverse_omega_rad_s（默认 0.5 rad/s）
           *          — 若两者均为 0，跳过此阶段直接进入段 1。
           *          — 若 debug_brake_duration_per_ms == 0，跳过此阶段直接进入段 1。
           *
           *   段 1 — 强制零电流：反向制动时间到（或已禁用）后，强制 iq=0，
           *          电机纯靠地面摩擦力滑行至自然停止。只要摇杆保持中位，
           *          电机驱动电流恒为零。
           *          （速度环 + 目标=0 → control.c hard stop 无条件零电流）
           *
           *   离开制动区（摇杆再次推动）→ 正常速度环驱动。 */
                    for (i = 0; i < MOTOR_COUNT; i++)
          {
            float abs_spd = motor_speeds[i];
            if (abs_spd < 0.0f) abs_spd = -abs_spd;

            if (abs_spd < debug_brake_threshold_rad_s)
            {
              /* ====== 制动区：摇杆中位 ====== */

              /* 边沿检测：刚进入制动区 → 记录时间戳 + 入口速度 */
              if (!brake_in_zone)
              {
                brake_entry_tick = debug_control_task_count;
                /* 前向运动学反算底盘实际平移速度，作为制动时长的比例基准 */
                {
                  float entry_vx, entry_vy, entry_omega;
                  ChassisKinematics_MotorsToChassis(
                      debug_actual_speed_rad_s,
                      &entry_vx, &entry_vy, &entry_omega);
                  brake_entry_speed_ms = sqrtf(entry_vx * entry_vx + entry_vy * entry_vy);
                }
                brake_in_zone = 1U;
              }

              /* 动态制动次数：与入口速度成正比 + 最大上限截断 */
              {
                uint32_t brake_dur = (uint32_t)(brake_entry_speed_ms * debug_brake_duration_per_ms);
                if (debug_brake_max_duration > 0.0f &&
                    brake_dur > (uint32_t)debug_brake_max_duration)
                {
                  brake_dur = (uint32_t)debug_brake_max_duration;
                }

                if ((debug_brake_reverse_speed_ms > 0.0f ||
                      debug_brake_reverse_omega_rad_s > 0.0f) &&
                    brake_dur > 0U &&
                    (debug_control_task_count - brake_entry_tick) < brake_dur)
              {
                /* 段 0 — 底盘级反向制动：
                 *
                 * 1. 前向运动学：从实际电机转速反算底盘 (vx, vy, omega)
                 * 2. 施加与运动方向相反的小线速度 + 角速度
                 * 3. 逆运动学分解为四电机一致的目标转速
                 *
                 * 关键：底盘级制动保证四电机协同运动，避免逐电机制动时
                 * 各电机反向速度不一致导致的底盘解体运动。 */

                /* 1. 前向运动学 */
                float actual_vx, actual_vy, actual_omega;
                ChassisKinematics_MotorsToChassis(
                    debug_actual_speed_rad_s,
                    &actual_vx, &actual_vy, &actual_omega);

                /* 2. 计算反向制动底盘速度 */
                float brake_vx = 0.0f, brake_vy = 0.0f, brake_omega = 0.0f;

                /* 线速度反向：归一化方向 × 制动强度 */
                if (debug_brake_reverse_speed_ms > 0.0f)
                {
                  float v_mag = sqrtf(actual_vx * actual_vx + actual_vy * actual_vy);
                  if (v_mag > 0.005f)  /* 0.5 cm/s 以上才触发，避免静止时数值噪声 */
                  {
                    float inv_mag = 1.0f / v_mag;
                    brake_vx = -(actual_vx * inv_mag) * debug_brake_reverse_speed_ms;
                    brake_vy = -(actual_vy * inv_mag) * debug_brake_reverse_speed_ms;
                  }
                }

                /* 角速度反向：方向与当前旋转相反 */
                if (debug_brake_reverse_omega_rad_s > 0.0f)
                {
                  float abs_omega = (actual_omega < 0.0f) ? -actual_omega : actual_omega;
                  if (abs_omega > 0.01f)  /* 0.01 rad/s 死区 */
                  {
                    brake_omega = (actual_omega > 0.0f)
                        ? -debug_brake_reverse_omega_rad_s
                        :  debug_brake_reverse_omega_rad_s;
                  }
                }

                /* 3. 逆运动学：底盘速度 → 四电机目标转速 */
                float brake_motor_speeds[4];
                ChassisKinematics_ChassisToMotors(
                    brake_vx, brake_vy, brake_omega, brake_motor_speeds);

                for (i = 0; i < MOTOR_COUNT; i++)
                {
                  debug_set_speed[i] = brake_motor_speeds[i];
                  MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_SPEED);
                  MotorControl_SetTargetSpeed(i, brake_motor_speeds[i]);
                }
                break;  /* 已在循环内设置了全部 4 电机，跳出外层 for */
              }
              else
              {
                /* 段 1 — 强制零电流：反向制动时间到（或已禁用），
                 * 速度环 + 目标=0 → control.c hard stop 无条件 iq=0。
                 * 此后只要摇杆保持中位，电机驱动电流恒为零。 */
                for (i = 0; i < MOTOR_COUNT; i++)
                {
                  debug_set_speed[i] = 0.0f;
                  MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_SPEED);
                  MotorControl_SetTargetSpeed(i, 0.0f);
                }
                break;  /* 已在循环内设置了全部 4 电机，跳出外层 for */
              }
              }  /* 关闭 brake_dur 作用域 */
            }
            else
            {
              /* 正常驱动区：速度环。同时清除制动区标记，下次进入时重新计时。 */
              brake_in_zone = 0U;
              debug_set_speed[i] = motor_speeds[i];
              MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_SPEED);
              MotorControl_SetTargetSpeed(i, motor_speeds[i]);
            }
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
