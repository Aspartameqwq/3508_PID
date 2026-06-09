#include "debug.h"

#include "control.h"

/* ======================== 快捷调试可调参数 ======================== */

/* 默认遥控器模式，烧录后电机待命等待遥控器输入。
 * 如需速度环/位置环调试，在 Ozone 中将 debug_quick_mode 改为
 * DEBUG_QUICK_MODE_SPEED 或 DEBUG_QUICK_MODE_POSITION。 */
volatile float debug_set_speed[MOTOR_COUNT]    = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float debug_set_position[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile uint8_t debug_quick_mode = DEBUG_QUICK_MODE_REMOTE;
volatile float debug_remote_max_speed = 100.0f;
volatile uint16_t debug_remote_deadzone = 20U;
volatile float debug_run_turns = 0.0f;
volatile float debug_gear_ratio = 3591.0f / 187.0f;
volatile float debug_turns_speed_rad_s = 40.0f;

/* ======================== 速度环 PID 参数（每个电机独立，Ozone 可直接修改） ======================== */

volatile float debug_speed_kp[MOTOR_COUNT]                     = {80.0f, 80.0f, 80.0f, 80.0f};
volatile float debug_speed_ki[MOTOR_COUNT]                     = {5.0f, 5.0f, 5.0f, 5.0f};
volatile float debug_speed_kd[MOTOR_COUNT]                     = {0.25f, 0.25f, 0.25f, 0.25f};
volatile float debug_speed_output_limit[MOTOR_COUNT]           = {10000.0f, 10000.0f, 10000.0f, 10000.0f};
volatile float debug_speed_integral_limit[MOTOR_COUNT]         = {100.0f, 100.0f, 100.0f, 100.0f};
volatile float debug_speed_integral_separation[MOTOR_COUNT]    = {10.0f, 10.0f, 10.0f, 10.0f};
volatile float debug_speed_ff_gain[MOTOR_COUNT]                = {40.0f, 40.0f, 40.0f, 40.0f};
volatile float debug_speed_derivative_alpha[MOTOR_COUNT]       = {0.15f, 0.15f, 0.15f, 0.15f};
volatile float debug_speed_zero_cross_threshold[MOTOR_COUNT]   = {1.0f, 1.0f, 1.0f, 1.0f};
volatile float debug_speed_zero_cross_brake[MOTOR_COUNT]       = {300.0f, 300.0f, 300.0f, 300.0f};
volatile float debug_speed_stiction_comp_iq[MOTOR_COUNT]       = {500.0f, 500.0f, 500.0f, 500.0f};

/* ======================== 位置环 PID 参数（每个电机独立，Ozone 可直接修改） ======================== */

volatile float debug_position_kp[MOTOR_COUNT]                     = {1.5f, 1.5f, 1.5f, 1.5f};
volatile float debug_position_ki[MOTOR_COUNT]                     = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float debug_position_kd[MOTOR_COUNT]                     = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float debug_position_output_limit[MOTOR_COUNT]           = {4.0f, 4.0f, 4.0f, 4.0f};
volatile float debug_position_integral_limit[MOTOR_COUNT]         = {5.0f, 5.0f, 5.0f, 5.0f};
volatile float debug_position_integral_separation[MOTOR_COUNT]    = {30.0f, 30.0f, 30.0f, 30.0f};
volatile float debug_position_deadband_deg[MOTOR_COUNT]           = {3.0f, 3.0f, 3.0f, 3.0f};
volatile float debug_position_derivative_alpha[MOTOR_COUNT]       = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float debug_position_stiction_threshold_deg[MOTOR_COUNT] = {10.0f, 10.0f, 10.0f, 10.0f};

/* ======================== 运行时诊断变量（每个电机独立，只读观察） ======================== */

volatile uint8_t debug_control_mode_active[MOTOR_COUNT]  = {MOTOR_CONTROL_MODE_SPEED, MOTOR_CONTROL_MODE_SPEED, MOTOR_CONTROL_MODE_SPEED, MOTOR_CONTROL_MODE_SPEED};
volatile uint8_t debug_feedback_valid[MOTOR_COUNT]       = {0U, 0U, 0U, 0U};
volatile float   debug_actual_angle_deg[MOTOR_COUNT]     = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float   debug_actual_speed_rad_s[MOTOR_COUNT]   = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float   debug_torque_cmd[MOTOR_COUNT]           = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float   debug_position_error_deg[MOTOR_COUNT]   = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float   debug_position_output_rad_s[MOTOR_COUNT]= {0.0f, 0.0f, 0.0f, 0.0f};

/* ======================== 全局诊断变量（标量，只读观察） ======================== */

volatile uint32_t debug_main_loop_count = 0U;
volatile uint32_t debug_control_task_count = 0U;

/* CAN 底层诊断：无论是否被控制层接受，每收到一帧 CAN 消息都计数。 */
volatile uint32_t debug_can_rx_raw_count = 0U;

/* 最近一帧 CAN 消息的 StdId（0=未收到），用于排查电机实际反馈 ID。 */
volatile uint16_t debug_last_rx_stdid = 0U;

/* ======================== DR16 DBUS 诊断变量 ======================== */

volatile uint32_t debug_dr16_idle_count = 0U;
volatile uint32_t debug_dr16_frame_count = 0U;
volatile uint16_t debug_dr16_ndtr = 0U;
volatile uint8_t  debug_dr16_ct = 0U;
volatile uint32_t debug_dr16_uart_sr = 0U;
volatile uint8_t  debug_dr16_raw[18] = {0};

/** @brief 最后一次收到有效 DR16 帧时的 HAL_GetTick() 值，0=尚未收到任何帧。
 *   main.c 遥控器模式据此判断信号是否丢失。 */
volatile uint32_t debug_dr16_last_frame_tick = 0U;

/** @brief 遥控器信号超时阈值（ms），默认 500 ms。 */
volatile uint32_t debug_dr16_signal_timeout_ms = 500U;

/* ======================== 快捷调试函数 ======================== */

/**
 * @brief 快捷调速：将所有 4 个电机切换到速度环，写入相同的目标转速。
 *        每个电机可在 Ozone 中单独修改 debug_set_speed[i] 以差异化控制。
 * @param speed_rad_s 目标转速（rad/s），应用到全部 4 个电机
 */
void Debug_QuickRunSpeed(float speed_rad_s)
{
  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
  {
    debug_set_speed[i] = speed_rad_s;
    MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_SPEED);
    MotorControl_SetTargetSpeed(i, speed_rad_s);
  }
}

/**
 * @brief 快捷调位：将所有 4 个电机切换到位置环，写入相同的目标角度。
 *        每个电机可在 Ozone 中单独修改 debug_set_position[i] 以差异化控制。
 * @param position_deg 目标角度（deg），应用到全部 4 个电机
 */
void Debug_QuickRunPosition(float position_deg)
{
  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
  {
    debug_set_position[i] = position_deg;
    MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_POSITION);
    MotorControl_SetTargetAngle(i, position_deg);
  }
}

/**
 * @brief 快捷转圈：将所有在线电机切换到位置环并启动相对转圈。
 *        使用"移动胡萝卜"驱动位置环 + 编码器脉冲累计判定完成。
 * @param turns 输出轴圈数（正=顺时针），例如 1.0 → 顺时针转 1 圈
 */
void Debug_QuickRunTurns(float turns)
{
  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
  {
    MotorControl_SetControlMode(i, MOTOR_CONTROL_MODE_POSITION);
    Control_StartTurns(i, turns);
  }
}
