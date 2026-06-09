#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================
 *                       调试系统全局定义
 *
 *   本文件集中管理所有 Ozone 实时可调参数与诊断变量，按功能分为 8 大块：
 *     1. 快捷调试可调参数（模式切换、目标值、制动、转圈）
 *     2. 速度环 PID 参数（每电机独立）
 *     3. 位置环 PID 参数（每电机独立）
 *     4. 运动学可调参数（底盘尺寸、速度限制、CCW 修正）
 *     5. 运行时诊断变量（每电机独立，只读）
 *     6. 全局诊断变量（标量，只读）
 *     7. 运动学诊断变量（底盘速度、电机解算转速，只读）
 *     8. DR16 DBUS 诊断变量（遥控器信号质量与帧数据）
 *
 *   添加新变量指南：
 *     - 每电机参数 → 声明 volatile float xxx[MOTOR_COUNT]
 *     - 全局标量参数 → 声明 volatile float/uint32_t xxx
 *     - 在 debug.c 对应区块添加定义与默认值
 *     - 在 3508_PID.md 文档中记录
 * ======================================================================== */

/* ======================== 常量与宏 ======================== */

/** @brief 电机数量（CAN ID 1~4），所有每电机数组以此作为长度。 */
#define MOTOR_COUNT 4U

/* ====== 制动系统参数说明 ======
 *
 * 摇杆回中后制动流程（两段式，底盘级，无位置环）：
 *   段 0 — 反向制动：前向运动学反算底盘实际运动 (vx, vy, omega) →
 *          施加方向相反的小速度 → 逆运动学分解为四电机协同目标转速。
 *          线速度制动强度：debug_brake_reverse_speed_ms（默认 0.1 m/s）
 *          角速度制动强度：debug_brake_reverse_omega_rad_s（默认 0.5 rad/s）
 *   段 1 — 强制零电流：速度环 + 目标=0 → hard stop → iq=0 无条件。
 *          只要摇杆保持中位，电机驱动电流恒为零。
 *
 * 制动全程不涉及位置环，确保长时间中位时电机无电流输出。 */

/** @brief 制动反向阶段持续控制任务迭代次数。
 *
 *   时间换算（与 CONTROL_PERIOD_MS=10ms 配合）：
 *     BRAKE_HS_COUNT = 制动时长(ms) / 10ms
 *     例：300ms → 30 次    150ms → 15 次    500ms → 50 次
 *
 *   设 0 则跳过段 0（反向制动），摇杆回中后直接进入段 1（零电流）。
 *   增大此值可延长反向制动作用时间，适用于重量大/惯性大的底盘。 */
#define BRAKE_HS_COUNT 30U

/* ======================== 快捷调试函数 ======================== */

void Debug_QuickRunSpeed(float speed_rad_s);
void Debug_QuickRunPosition(float position_deg);
void Debug_QuickRunTurns(float turns);

/* ========================================================================
 *   第 1 块：快捷调试可调参数
 *   — 模式选择（速度/位置/遥控器）
 *   — 各模式的目标值（转速、角度）
 *   — 制动系统（阈值、反向线速度、反向角速度、迭代次数）
 *       详见 BRAKE_HS_COUNT 宏注释和下方各变量的 @brief 说明
 *   — 转圈参数（触发、减速比、速度）
 *   — 遥控器参数（最大转速、死区）
 * ======================================================================== */

/** @brief 每个电机的目标转速（rad/s），用于 DEBUG_QUICK_MODE_SPEED 模式。 */
extern volatile float debug_set_speed[MOTOR_COUNT];

/** @brief 每个电机的目标角度（deg），用于 DEBUG_QUICK_MODE_POSITION 模式。 */
extern volatile float debug_set_position[MOTOR_COUNT];

/** @brief 快捷调试模式选择（所有电机共用）。
 *   0=速度环 (DEBUG_QUICK_MODE_SPEED)
 *   1=位置环 (DEBUG_QUICK_MODE_POSITION)
 *   2=遥控器 (DEBUG_QUICK_MODE_REMOTE，默认) */
extern volatile uint8_t debug_quick_mode;

/** @brief 转圈触发：非零时所有在线电机执行一次相对转圈，完成后自动清零。 */
extern volatile float debug_run_turns;

/** @brief 减速比（电机轴:输出轴），用于转圈脉冲换算。默认 3591/187 ≈ 19.2。 */
extern volatile float debug_gear_ratio;

/** @brief 转圈时的目标转速（rad/s），绕过位置环输出限幅。 */
extern volatile float debug_turns_speed_rad_s;

/** @brief 遥控器模式下摇杆满偏时的最大目标转速（rad/s）。 */
extern volatile float debug_remote_max_speed;

/** @brief 遥控器摇杆死区（原始值偏移量，ch_center=1024）。 */
extern volatile uint16_t debug_remote_deadzone;

/* ====== 制动系统参数 ======
 *
 * 制动阈值决定"摇杆是否已回中"；线速度/角速度参数控制"施加多大的反向力"；
 * BRAKE_HS_COUNT 控制"反向制动持续多久"。三者配合调节。
 *
 * 调参流程（Ozone）：
 *   1. 先设定 debug_brake_threshold_rad_s（摇杆中位检测灵敏度）
 *   2. 推满速后松杆 → 看制动效果
 *   3. 线速度制不住 → 加大 debug_brake_reverse_speed_ms
 *   4. 旋转停不下来 → 加大 debug_brake_reverse_omega_rad_s
 *   5. 制动时间不够 → 加大 BRAKE_HS_COUNT（需重编译）
 *   6. 线速度和角速度的制动可独立启用/禁用（设为 0 即禁用） */

/** @brief 制动区入口阈值（rad/s，电机轴侧）。
 *
 *   运动学解算出的电机目标转速绝对值低于此值时，认为"摇杆已回中"，进入制动流程。
 *   默认 0.3 rad/s。设 0 则完全禁用制动（永不在摇杆中位触发）。
 *
 *   Ozone 调参指南：
 *   - 轻微触碰摇杆就误触发制动 → 调大（如 0.5~1.0）
 *   - 车速降至极低才触发（制动太晚，车已滑很远）→ 调小（如 0.1~0.2）
 *   - 典型值范围为 0.1~1.0 rad/s */
extern volatile float debug_brake_threshold_rad_s;

/** @brief 【段 0】底盘级反向制动线速度强度（m/s）。
 *
 *   摇杆回中后，前向运动学反算出底盘实际平移方向，然后施加与此值等大的反向
 *   线速度进行制动。通过逆运动学分解为四电机一致的目标转速，保证制动时底盘
 *   运动方向协调统一。
 *
 *   例如：底盘正往前滑（vy_actual > 0）→ 施加 vy = -0.1 m/s → 四电机协同反向。
 *
 *   默认 0.1 m/s。设 0 则跳过反向线速度制动。
 *
 *   Ozone 调参指南：
 *   - 轻量级底盘/低摩擦力地面 → 0.05~0.1 m/s
 *   - 重量级底盘/高惯性/高摩擦力 → 0.2~0.5 m/s
 *   - 调太大可能导致反向弹跳（停下来后又反向加速） */
extern volatile float debug_brake_reverse_speed_ms;

/** @brief 【段 0】底盘级反向制动角速度强度（rad/s）。
 *
 *   摇杆回中后，前向运动学反算出底盘实际旋转方向，然后施加与此值等大的反向
 *   角速度进行旋转制动。
 *
 *   例如：底盘正在 CCW 旋转（omega_actual > 0）→ 施加 omega = -0.5 rad/s。
 *
 *   默认 0.5 rad/s。设 0 则跳过反向角速度制动。
 *
 *   Ozone 调参指南：
 *   - 旋转惯性小/轮距窄 → 0.3~0.5 rad/s
 *   - 旋转惯性大/轮距宽 → 0.5~1.0 rad/s
 *   - 出现旋转方向持续振荡 → 调小或设 0（让线速度制动单独作用）
 *   - 线速度和角速度制动独立工作，可同时启用或只启用其中一个 */
extern volatile float debug_brake_reverse_omega_rad_s;

/** @brief 【已废弃】旧版逐电机反向制动速度（rad/s）。
 *
 *   此参数在旧版制动逻辑中，针对每个电机独立施加与其当前转速方向相反的目标转速。
 *   现已被底盘级制动取代（上方 debug_brake_reverse_speed_ms +
 *   debug_brake_reverse_omega_rad_s），当前代码不再引用。
 *
 *   保留定义仅为兼容旧版 Ozone 工程文件，避免打开旧 .jdebug 时报 "symbol not found"。
 *   新调参请使用底盘级制动参数。 */
extern volatile float debug_brake_reverse_speed_rad_s;

/* ========================================================================
 *   第 2 块：速度环 PID 参数（每电机独立 [MOTOR_COUNT]，Ozone 可修改）
 *   — 控制电机轴转速响应
 * ======================================================================== */

extern volatile float debug_speed_kp[MOTOR_COUNT];
extern volatile float debug_speed_ki[MOTOR_COUNT];
extern volatile float debug_speed_kd[MOTOR_COUNT];
extern volatile float debug_speed_output_limit[MOTOR_COUNT];
extern volatile float debug_speed_integral_limit[MOTOR_COUNT];
extern volatile float debug_speed_integral_separation[MOTOR_COUNT];
extern volatile float debug_speed_ff_gain[MOTOR_COUNT];
extern volatile float debug_speed_derivative_alpha[MOTOR_COUNT];
extern volatile float debug_speed_zero_cross_threshold[MOTOR_COUNT];
extern volatile float debug_speed_zero_cross_brake[MOTOR_COUNT];
extern volatile float debug_speed_stiction_comp_iq[MOTOR_COUNT];

/* ========================================================================
 *   第 3 块：位置环 PID 参数（每电机独立 [MOTOR_COUNT]，Ozone 可修改）
 *   — 控制电机轴角度响应（P-only 为主，Ki/Kd 默认 0）
 * ======================================================================== */

extern volatile float debug_position_kp[MOTOR_COUNT];
extern volatile float debug_position_ki[MOTOR_COUNT];
extern volatile float debug_position_kd[MOTOR_COUNT];
extern volatile float debug_position_output_limit[MOTOR_COUNT];
extern volatile float debug_position_integral_limit[MOTOR_COUNT];
extern volatile float debug_position_integral_separation[MOTOR_COUNT];
extern volatile float debug_position_deadband_deg[MOTOR_COUNT];
extern volatile float debug_position_derivative_alpha[MOTOR_COUNT];
extern volatile float debug_position_stiction_threshold_deg[MOTOR_COUNT];

/* ========================================================================
 *   第 4 块：运动学可调参数（Ozone 实时修改）
 *   — 底盘几何尺寸、速度限制、硬件偏航补偿
 *   — 由 chassis_kinematics.c 的运动学解算函数读取
 * ======================================================================== */

/** @brief 底盘最大平移速度（m/s）。遥控器满偏时 vx 或 vy 分量的最大值。
 *   典型竞赛机器人取值 1.0~4.0 m/s。 */
extern volatile float debug_kinematics_max_speed_ms;

/** @brief 底盘最大旋转角速度（rad/s）。遥控器满偏时的 omega 最大值。
 *   2π ≈ 1 rev/s，典型值 3.0~10.0 rad/s。 */
extern volatile float debug_kinematics_max_omega_rad_s;

/** @brief 全向轮半径（m）。用于线速度 → 电机角速度换算。
 *   默认 50 mm（常见 100 mm 直径全向轮）。 */
extern volatile float debug_kinematics_wheel_radius_m;

/** @brief 底盘半轴长（m）。轮位中心到 X/Y 轴的投影距离，影响旋转分量权重。
 *   默认 150 mm（正方形边长 300 mm）。 */
extern volatile float debug_kinematics_wheel_base_m;

/** @brief 逆时针（CCW）偏航修正增益（rad/s 每 m/s 平移速度）。
 *   底盘直行时若因机械不对称出现偏航，可用此参数叠加与平移速度成正比的
 *   逆时针角速度进行补偿。设 0 则禁用。正值=CCW，负值=CW。 */
extern volatile float debug_kinematics_ccw_correction_gain;

/** @brief 速度矢量偏向角（°）。将解算后的 (vx, vy) 旋转此角度后再送逆运动学。
 *   正值=逆时针（CCW）旋转，负值=顺时针（CW）旋转。
 *   例：推正前（vy>0）车却向左前跑 → 设正值把速度矢量往右修正。
 *   默认 0°（不旋转）。与 ccw_correction_gain 配合使用，Ozone 中可独立调节。 */
extern volatile float debug_kinematics_bias_angle_deg;

/* ========================================================================
 *   第 5 块：运行时诊断变量（每电机独立 [MOTOR_COUNT]，只读观察）
 *   — 反映各电机当前状态与控制输出
 * ======================================================================== */

extern volatile uint8_t debug_control_mode_active[MOTOR_COUNT];
extern volatile uint8_t debug_feedback_valid[MOTOR_COUNT];
extern volatile float   debug_actual_angle_deg[MOTOR_COUNT];
extern volatile float   debug_actual_speed_rad_s[MOTOR_COUNT];
extern volatile float   debug_torque_cmd[MOTOR_COUNT];
extern volatile float   debug_position_error_deg[MOTOR_COUNT];
extern volatile float   debug_position_output_rad_s[MOTOR_COUNT];

/* ========================================================================
 *   第 6 块：全局诊断变量（标量，只读观察）
 *   — 系统运行计数与 CAN 底层统计
 * ======================================================================== */

/** @brief 主循环迭代计数。每次 while(1) 循环自增 1。 */
extern volatile uint32_t debug_main_loop_count;

/** @brief 控制任务执行计数。每 10ms 自增 1（Control_Task 调用次数）。
 *   可用于观察控制频率，也作为制动硬停的计时基准。 */
extern volatile uint32_t debug_control_task_count;

/** @brief CAN RX 原始帧计数：每收到一帧 CAN 消息自增 1（含非电机帧）。 */
extern volatile uint32_t debug_can_rx_raw_count;

/** @brief 最近一帧 CAN 消息的 StdId（0=未收到），用于排查电机反馈 ID。 */
extern volatile uint16_t debug_last_rx_stdid;

/* ========================================================================
 *   第 7 块：运动学诊断变量（只读观察，Ozone 可查看）
 *   — 由 ChassisKinematics_ChassisToMotors() 每次调用时更新
 * ======================================================================== */

/** @brief 当前解算的底盘目标平移速度 X 分量（m/s，正值=右）。 */
extern volatile float debug_chassis_vx_ms;

/** @brief 当前解算的底盘目标平移速度 Y 分量（m/s，正值=前）。 */
extern volatile float debug_chassis_vy_ms;

/** @brief 当前解算的底盘目标旋转角速度（rad/s，正值=CCW 逆时针）。 */
extern volatile float debug_chassis_omega_rad_s;

/** @brief 运动学解算后的各电机目标转速（rad/s，电机轴侧，4 轮独立）。
 *   即 main.c 中 motor_speeds[] 的副本，Ozone 可直接对比目标与实际转速。 */
extern volatile float debug_kinematics_motor_speed[MOTOR_COUNT];

/* ========================================================================
 *   第 8 块：DR16 DBUS 诊断变量
 *   — 遥控器信号质量、帧原始数据、超时保护
 * ======================================================================== */

/** @brief USART3 IDLE 中断触发总次数（底层硬件中断计数）。 */
extern volatile uint32_t debug_dr16_idle_count;

/** @brief RemoteDataProcess() 成功调用次数（协议层解帧计数）。
 *   与 idle_count 对比可判断是否有 DMA 溢出或帧格式错误。 */
extern volatile uint32_t debug_dr16_frame_count;

/** @brief IDLE 中断时 DMA NDTR 快照（剩余字节数，18 字节帧正常=0）。 */
extern volatile uint16_t debug_dr16_ndtr;

/** @brief IDLE 中断时 DMA CT 位快照（0=M0, 1=M1，指示当前缓冲区）。 */
extern volatile uint8_t  debug_dr16_ct;

/** @brief IDLE 中断时 USART3 SR 寄存器快照（PE/FE/ORE/NE 错误标志位）。 */
extern volatile uint32_t debug_dr16_uart_sr;

/** @brief 最近一帧原始 18 字节 DBUS 数据快照（Ozone 可直接查看十六进制）。 */
extern volatile uint8_t  debug_dr16_raw[18];

/** @brief 最后一次收到有效 DR16 帧时的 HAL_GetTick() 值（ms）。
 *   0=尚未收到任何帧。main.c 遥控器模式据此判断信号是否丢失。 */
extern volatile uint32_t debug_dr16_last_frame_tick;

/** @brief 遥控器信号超时阈值（ms）。超过此时间未收到新帧→视为信号丢失→电机归零。
 *   Ozone 中可修改，默认 500 ms（DR16 正常帧间隔约 14 ms，500 ms ≈ 35 帧丢失）。 */
extern volatile uint32_t debug_dr16_signal_timeout_ms;

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
