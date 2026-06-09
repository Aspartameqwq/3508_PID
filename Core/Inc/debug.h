#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 电机数量 ======================== */

#define MOTOR_COUNT 4U

/* ======================== 快捷调试函数 ======================== */

void Debug_QuickRunSpeed(float speed_rad_s);
void Debug_QuickRunPosition(float position_deg);
void Debug_QuickRunTurns(float turns);

/* ======================== 快捷调试可调参数 ======================== */

/** @brief 每个电机的目标转速（rad/s），用于 DEBUG_QUICK_MODE_SPEED 模式。
 *   Ozone 中可单独修改每个电机的目标值。 */
extern volatile float debug_set_speed[MOTOR_COUNT];

/** @brief 每个电机的目标角度（deg），用于 DEBUG_QUICK_MODE_POSITION 模式。
 *   Ozone 中可单独修改每个电机的目标值。 */
extern volatile float debug_set_position[MOTOR_COUNT];

/** @brief 快捷调试模式选择（所有电机共用）。
 *   0=速度环, 1=位置环, 2=遥控器（默认） */
extern volatile uint8_t debug_quick_mode;

/** @brief 转圈触发：非零时所有在线电机执行一次相对转圈，完成后自动清零。 */
extern volatile float debug_run_turns;

/** @brief 减速比（电机轴:输出轴），用于转圈脉冲换算。 */
extern volatile float debug_gear_ratio;

/** @brief 转圈时的目标转速（rad/s），绕过位置环输出限幅。 */
extern volatile float debug_turns_speed_rad_s;

/** @brief 遥控器模式下摇杆满偏时的最大目标转速（rad/s）。 */
extern volatile float debug_remote_max_speed;

/** @brief 遥控器摇杆死区（原始值偏移量，ch_center=1024）。 */
extern volatile uint16_t debug_remote_deadzone;

/* ======================== 速度环 PID 参数（每个电机独立，Ozone 可直接修改） ======================== */

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

/* ======================== 位置环 PID 参数（每个电机独立，Ozone 可直接修改） ======================== */

extern volatile float debug_position_kp[MOTOR_COUNT];
extern volatile float debug_position_ki[MOTOR_COUNT];
extern volatile float debug_position_kd[MOTOR_COUNT];
extern volatile float debug_position_output_limit[MOTOR_COUNT];
extern volatile float debug_position_integral_limit[MOTOR_COUNT];
extern volatile float debug_position_integral_separation[MOTOR_COUNT];
extern volatile float debug_position_deadband_deg[MOTOR_COUNT];
extern volatile float debug_position_derivative_alpha[MOTOR_COUNT];
extern volatile float debug_position_stiction_threshold_deg[MOTOR_COUNT];

/* ======================== 运行时诊断变量（每个电机独立，只读观察） ======================== */

extern volatile uint8_t debug_control_mode_active[MOTOR_COUNT];
extern volatile uint8_t debug_feedback_valid[MOTOR_COUNT];
extern volatile float debug_actual_angle_deg[MOTOR_COUNT];
extern volatile float debug_actual_speed_rad_s[MOTOR_COUNT];
extern volatile float debug_torque_cmd[MOTOR_COUNT];
extern volatile float debug_position_error_deg[MOTOR_COUNT];
extern volatile float debug_position_output_rad_s[MOTOR_COUNT];

/* ======================== 全局诊断变量（标量，只读观察） ======================== */

extern volatile uint32_t debug_main_loop_count;
extern volatile uint32_t debug_control_task_count;
extern volatile uint32_t debug_can_rx_raw_count;
extern volatile uint16_t debug_last_rx_stdid;

/* ======================== DR16 DBUS 诊断变量 ======================== */

/* 底层中断/帧计数器：定位问题在硬件还是协议层。 */
extern volatile uint32_t debug_dr16_idle_count;       /* USART3 IDLE 中断触发总次数 */
extern volatile uint32_t debug_dr16_frame_count;      /* RemoteDataProcess 成功调用次数 */
extern volatile uint16_t debug_dr16_ndtr;             /* IDLE 中断时 DMA NDTR 快照（剩余字节数） */
extern volatile uint8_t  debug_dr16_ct;               /* IDLE 中断时 DMA CT 位快照（0=M0, 1=M1） */
extern volatile uint32_t debug_dr16_uart_sr;          /* IDLE 中断时 USART3 SR 寄存器快照（看 PE/FE/ORE/NE） */
extern volatile uint8_t  debug_dr16_raw[18];          /* 最近一帧原始 18 字节数据快照（Ozone 可直接查看） */

/** @brief 最后一次收到有效 DR16 帧时的 HAL_GetTick() 值（ms），0=尚未收到任何帧。 */
extern volatile uint32_t debug_dr16_last_frame_tick;

/** @brief 遥控器信号超时阈值（ms）。超过此时间未收到新帧→视为信号丢失→电机归零。
 *   Ozone 中可修改，默认 500 ms（DR16 正常帧间隔约 14 ms，500 ms ≈ 35 帧丢失）。 */
extern volatile uint32_t debug_dr16_signal_timeout_ms;

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
