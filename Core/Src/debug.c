/**
  ******************************************************************************
  * @file    debug.c
  * @brief   调试系统全局变量定义
  *
  *          所有 Ozone 可调参数与诊断变量在此定义并赋默认值。
  *          变量按功能分为 8 大块（与 debug.h 对应）。
  *
  *          添加新变量指南：
  *            - 在 debug.h 对应区块添加 extern 声明
  *            - 在本文对应区块添加 volatile 定义与默认值
  *            - 若需在控制循环中更新，在 control.c Control_Task() 中赋值
  *            - 更新 3508_PID.md 文档
  ******************************************************************************
  */

#include "debug.h"
#include "control.h"  /* MOTOR_CONTROL_MODE_SPEED, DEBUG_QUICK_MODE_REMOTE */

/* ========================================================================
 *   第 1 块：快捷调试可调参数
 * ======================================================================== */

volatile float    debug_set_speed[MOTOR_COUNT]    = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float    debug_set_position[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* 默认遥控器模式，烧录后电机待命等待遥控器输入。
 * 如需速度环/位置环调试，在 Ozone 中将此值改为对应模式。 */
volatile uint8_t  debug_quick_mode = DEBUG_QUICK_MODE_REMOTE;

volatile float    debug_run_turns              = 0.0f;
volatile float    debug_gear_ratio             = 3591.0f / 187.0f;
volatile float    debug_turns_speed_rad_s      = 40.0f;
volatile float    debug_remote_max_speed       = 100.0f;
volatile uint16_t debug_remote_deadzone        = 20U;

/* ====== 制动系统参数 ======
 *
 * 摇杆回中（所有通道=1024）→ 运动学解算输出全为零 → 底盘进入制动区。
 * 制动流程为两段式（无位置环）：
 *   段 0 — 底盘级反向制动（0~300ms）：前向运动学反算底盘实际运动 →
 *          施加与运动方向相反的小速度 → 逆运动学分解为四电机协同目标。
 *   段 1 — 强制零电流（300ms 后）：iq=0 无条件，电机自然滑行停止。
 *
 * 制动全程不使用位置环，确保摇杆长时间处于中位时电机驱动电流必为零。 */

/** @brief 制动区入口阈值（rad/s，电机轴侧）。运动学解算的电机目标转速绝对值低于
 *   此值时认为"摇杆回中"，进入制动流程。默认 0.3 rad/s。
 *   Ozone 调参：太小 → 轻微触碰摇杆就触发制动（不灵敏）；太大 → 车速接近零
 *   时才触发（制动太晚）。设 0 则完全禁用制动功能。 */
volatile float    debug_brake_threshold_rad_s     = 0.3f;

/** @brief 【段 0】底盘级反向制动线速度（m/s）。摇杆回中后，前向运动学反算出底盘
 *   实际移动方向，然后施加与此值等大的反向线速度进行制动。
 *   例如：底盘往前滑（vy>0）→ 施加 -0.1 m/s → 逆运动学分解为四电机协同反向转速。
 *   默认 0.1 m/s。Ozone 调参：摩擦力大 → 调小（0.05）；摩擦力小/重量大 → 调大（0.2~0.5）。
 *   设 0 则跳过反向线速度制动（仅角速度制动可能仍生效）。 */
volatile float    debug_brake_reverse_speed_ms   = 0.0f;

/** @brief 【段 0】底盘级反向制动角速度（rad/s）。摇杆回中后，前向运动学反算出底盘
 *   实际旋转方向，然后施加与此值等大的反向角速度进行制动。
 *   例如：底盘正在 CCW 旋转（omega>0）→ 施加 -0.5 rad/s → 逆运动学分解。
 *   默认 0.5 rad/s。Ozone 调参：旋转制动力不够 → 调大；制动时出现旋转振荡 → 调小。
 *   设 0 则跳过反向角速度制动（仅线速度制动可能仍生效）。 */
volatile float    debug_brake_reverse_omega_rad_s = 0.3f;

/** @brief 【已废弃】旧版逐电机反向制动速度（rad/s）。此参数针对单个电机施加反向
 *   转速，底盘级制动已移至上方的 debug_brake_reverse_speed_ms 和
 *   debug_brake_reverse_omega_rad_s。仅保留以便兼容旧版 Ozone 工程，
 *   当前代码不再引用此变量。 */
volatile float    debug_brake_reverse_speed_rad_s = 1.0f;

/* ========================================================================
 *   第 2 块：速度环 PID 参数（每电机独立 [MOTOR_COUNT]）
 * ======================================================================== */

volatile float debug_speed_kp[MOTOR_COUNT]                     = {80.0f, 80.0f, 80.0f, 80.0f};
volatile float debug_speed_ki[MOTOR_COUNT]                     = {5.0f, 5.0f, 5.0f, 5.0f};
volatile float debug_speed_kd[MOTOR_COUNT]                     = {0.125f, 0.125f, 0.125f, 0.125f};
volatile float debug_speed_output_limit[MOTOR_COUNT]           = {10000.0f, 10000.0f, 10000.0f, 10000.0f};
volatile float debug_speed_integral_limit[MOTOR_COUNT]         = {100.0f, 100.0f, 100.0f, 100.0f};
volatile float debug_speed_integral_separation[MOTOR_COUNT]    = {10.0f, 10.0f, 10.0f, 10.0f};
volatile float debug_speed_ff_gain[MOTOR_COUNT]                = {40.0f, 40.0f, 40.0f, 40.0f};
volatile float debug_speed_derivative_alpha[MOTOR_COUNT]       = {0.15f, 0.15f, 0.15f, 0.15f};
volatile float debug_speed_zero_cross_threshold[MOTOR_COUNT]   = {1.0f, 1.0f, 1.0f, 1.0f};
volatile float debug_speed_zero_cross_brake[MOTOR_COUNT]       = {300.0f, 300.0f, 300.0f, 300.0f};
volatile float debug_speed_stiction_comp_iq[MOTOR_COUNT]       = {500.0f, 500.0f, 500.0f, 500.0f};

/* ========================================================================
 *   第 3 块：位置环 PID 参数（每电机独立 [MOTOR_COUNT]）
 * ======================================================================== */

volatile float debug_position_kp[MOTOR_COUNT]                     = {1.5f, 1.5f, 1.5f, 1.5f};
volatile float debug_position_ki[MOTOR_COUNT]                     = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float debug_position_kd[MOTOR_COUNT]                     = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float debug_position_output_limit[MOTOR_COUNT]           = {4.0f, 4.0f, 4.0f, 4.0f};
volatile float debug_position_integral_limit[MOTOR_COUNT]         = {5.0f, 5.0f, 5.0f, 5.0f};
volatile float debug_position_integral_separation[MOTOR_COUNT]    = {30.0f, 30.0f, 30.0f, 30.0f};
volatile float debug_position_deadband_deg[MOTOR_COUNT]           = {3.0f, 3.0f, 3.0f, 3.0f};
volatile float debug_position_derivative_alpha[MOTOR_COUNT]       = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float debug_position_stiction_threshold_deg[MOTOR_COUNT] = {10.0f, 10.0f, 10.0f, 10.0f};

/* ========================================================================
 *   第 4 块：运动学可调参数
 *   由 chassis_kinematics.c 的运动学解算函数读取。
 * ======================================================================== */

/* 底盘最大平移速度（m/s）。典型竞赛机器人取值 1.0~4.0 m/s。 */
volatile float debug_kinematics_max_speed_ms = 2.0f;

/* 底盘最大旋转角速度（rad/s）。2π ≈ 1 rev/s，典型值 3.0~10.0 rad/s。 */
volatile float debug_kinematics_max_omega_rad_s = 6.28f;

/* 全向轮半径（m），默认 150 mm（常见 100 mm 直径全向轮）。 */
volatile float debug_kinematics_wheel_radius_m = 0.15f;

/* 底盘半轴长（m），轮位中心到 X/Y 轴的投影距离，默认 150 mm。 */
volatile float debug_kinematics_wheel_base_m = 0.15f;

/* 逆时针（CCW）偏航修正增益（rad/s 每 m/s 平移速度），默认 0 禁用。 */
volatile float debug_kinematics_ccw_correction_gain = 0.45f;

/* 速度矢量偏向角（°），正值=CCW 旋转，默认 0 不旋转。
 * 推正前车往左前偏 → 在 Ozone 中增大此值做顺时针修正。 */
volatile float debug_kinematics_bias_angle_deg = 0.0f;

/* ========================================================================
 *   第 5 块：运行时诊断变量（每电机独立 [MOTOR_COUNT]，只读观察）
 * ======================================================================== */

volatile uint8_t debug_control_mode_active[MOTOR_COUNT]  = {MOTOR_CONTROL_MODE_SPEED, MOTOR_CONTROL_MODE_SPEED, MOTOR_CONTROL_MODE_SPEED, MOTOR_CONTROL_MODE_SPEED};
volatile uint8_t debug_feedback_valid[MOTOR_COUNT]       = {0U, 0U, 0U, 0U};
volatile float   debug_actual_angle_deg[MOTOR_COUNT]     = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float   debug_actual_speed_rad_s[MOTOR_COUNT]   = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float   debug_torque_cmd[MOTOR_COUNT]           = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float   debug_position_error_deg[MOTOR_COUNT]   = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float   debug_position_output_rad_s[MOTOR_COUNT]= {0.0f, 0.0f, 0.0f, 0.0f};

/* ========================================================================
 *   第 6 块：全局诊断变量（标量，只读观察）
 * ======================================================================== */

volatile uint32_t debug_main_loop_count    = 0U;
volatile uint32_t debug_control_task_count = 0U;
volatile uint32_t debug_can_rx_raw_count   = 0U;
volatile uint16_t debug_last_rx_stdid      = 0U;

/* ========================================================================
 *   第 7 块：运动学诊断变量（只读观察）
 *   由 ChassisKinematics_ChassisToMotors() 每次调用时更新。
 * ======================================================================== */

volatile float debug_chassis_vx_ms                       = 0.0f;
volatile float debug_chassis_vy_ms                       = 0.0f;
volatile float debug_chassis_omega_rad_s                 = 0.0f;
volatile float debug_kinematics_motor_speed[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* ========================================================================
 *   第 8 块：DR16 DBUS 诊断变量
 * ======================================================================== */

volatile uint32_t debug_dr16_idle_count  = 0U;
volatile uint32_t debug_dr16_frame_count = 0U;
volatile uint16_t debug_dr16_ndtr        = 0U;
volatile uint8_t  debug_dr16_ct          = 0U;
volatile uint32_t debug_dr16_uart_sr     = 0U;
volatile uint8_t  debug_dr16_raw[18]     = {0};
volatile uint32_t debug_dr16_last_frame_tick   = 0U;
volatile uint32_t debug_dr16_signal_timeout_ms = 500U;

/* ========================================================================
 *   快捷调试函数
 * ======================================================================== */

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
