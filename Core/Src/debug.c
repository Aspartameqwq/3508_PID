#include "debug.h"

#include "control.h"

/* ======================== 快捷调试可调参数 ======================== */

/* 默认位置环 0°，烧录后电机保持静止在 0°。如需速度环调试，在 Ozone 中将 debug_quick_mode 改为 DEBUG_QUICK_MODE_SPEED。 */
volatile float debug_set_speed = 0.0f;
volatile float debug_set_position = 0.0f;
volatile uint8_t debug_quick_mode = DEBUG_QUICK_MODE_POSITION;
volatile float debug_remote_max_speed = 100.0f;
volatile uint16_t debug_remote_deadzone = 20U;
volatile float debug_run_turns = 0.0f;
volatile float debug_gear_ratio = 3591.0f / 187.0f;
volatile float debug_turns_speed_rad_s = 40.0f;

/* ======================== 速度环 PID 参数（Ozone 可直接修改） ======================== */

volatile float debug_speed_kp = 80.0f;
volatile float debug_speed_ki = 5.0f;
volatile float debug_speed_kd = 0.25f;
volatile float debug_speed_output_limit = 10000.0f;
volatile float debug_speed_integral_limit = 100.0f;
volatile float debug_speed_integral_separation = 10.0f;
volatile float debug_speed_ff_gain = 0.0f;
volatile float debug_speed_derivative_alpha = 0.15f;
volatile float debug_speed_zero_cross_threshold = 1.0f;
volatile float debug_speed_zero_cross_brake = 300.0f;
volatile float debug_speed_stiction_comp_iq = 500.0f;

/* ======================== 位置环 PID 参数（Ozone 可直接修改） ======================== */

volatile float debug_position_kp = 1.5f;
volatile float debug_position_ki = 0.0f;
volatile float debug_position_kd = 0.0f;
volatile float debug_position_output_limit = 4.0f;
volatile float debug_position_integral_limit = 5.0f;
volatile float debug_position_integral_separation = 30.0f;
volatile float debug_position_deadband_deg = 3.0f;
volatile float debug_position_derivative_alpha = 0.0f;
volatile float debug_position_stiction_threshold_deg = 10.0f;

/* ======================== 运行时诊断变量（只读观察） ======================== */

volatile uint32_t debug_main_loop_count = 0U;
volatile uint32_t debug_control_task_count = 0U;
volatile uint8_t debug_feedback_valid = 0U;
volatile uint8_t debug_active_motor_id = 0U;
volatile float debug_actual_angle_deg = 0.0f;
volatile float debug_actual_speed_rad_s = 0.0f;
volatile float debug_torque_cmd = 0.0f;
volatile uint8_t debug_control_mode_active = MOTOR_CONTROL_MODE_POSITION;
volatile float debug_position_error_deg = 0.0f;
volatile float debug_position_output_rad_s = 0.0f;

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

/* ======================== 快捷调试函数 ======================== */

/* 快捷调速：切到速度环并写入目标速度，同步更新 debug_set_speed 供 Ozone 观察。 */
void Debug_QuickRunSpeed(float speed_rad_s)
{
  debug_set_speed = speed_rad_s;
  MotorControl_SetControlMode(MOTOR_CONTROL_MODE_SPEED);
  MotorControl_SetTargetSpeed(speed_rad_s);
}

/* 快捷调位：切到位置环并写入目标角度，同步更新 debug_set_position 供 Ozone 观察。 */
void Debug_QuickRunPosition(float position_deg)
{
  debug_set_position = position_deg;
  MotorControl_SetControlMode(MOTOR_CONTROL_MODE_POSITION);
  MotorControl_SetTargetAngle(position_deg);
}

/* 快捷转圈：以当前位置为起点，输出轴相对转动 turns 圈。
 * 例如 turns=1.0 → 输出轴顺时针转 1 圈；turns=-0.5 → 逆时针转半圈。
 * 内部用"移动胡萝卜"驱动位置环 + 编码器脉冲累计，不受 360° 取模影响。 */
void Debug_QuickRunTurns(float turns)
{
  MotorControl_SetControlMode(MOTOR_CONTROL_MODE_POSITION);
  Control_StartTurns(turns);
}
