#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 快捷调试函数 ======================== */

void Debug_QuickRunSpeed(float speed_rad_s);
void Debug_QuickRunPosition(float position_deg);
void Debug_QuickRunTurns(float turns);

/* ======================== 快捷调试可调参数 ======================== */

extern volatile float debug_set_speed;
extern volatile float debug_set_position;
extern volatile uint8_t debug_quick_mode;
extern volatile float debug_run_turns;
extern volatile float debug_gear_ratio;
extern volatile float debug_turns_speed_rad_s;
extern volatile float debug_remote_max_speed;
extern volatile uint16_t debug_remote_deadzone;

/* ======================== 速度环 PID 参数（Ozone 可直接修改） ======================== */

extern volatile float debug_speed_kp;
extern volatile float debug_speed_ki;
extern volatile float debug_speed_kd;
extern volatile float debug_speed_output_limit;
extern volatile float debug_speed_integral_limit;
extern volatile float debug_speed_integral_separation;
extern volatile float debug_speed_ff_gain;
extern volatile float debug_speed_derivative_alpha;
extern volatile float debug_speed_zero_cross_threshold;
extern volatile float debug_speed_zero_cross_brake;
extern volatile float debug_speed_stiction_comp_iq;

/* ======================== 位置环 PID 参数（Ozone 可直接修改） ======================== */

extern volatile float debug_position_kp;
extern volatile float debug_position_ki;
extern volatile float debug_position_kd;
extern volatile float debug_position_output_limit;
extern volatile float debug_position_integral_limit;
extern volatile float debug_position_integral_separation;
extern volatile float debug_position_deadband_deg;
extern volatile float debug_position_derivative_alpha;
extern volatile float debug_position_stiction_threshold_deg;

/* ======================== 运行时诊断变量（只读观察） ======================== */

extern volatile uint32_t debug_main_loop_count;
extern volatile uint32_t debug_control_task_count;
extern volatile uint8_t debug_feedback_valid;
extern volatile uint8_t debug_active_motor_id;
extern volatile float debug_actual_angle_deg;
extern volatile float debug_actual_speed_rad_s;
extern volatile float debug_torque_cmd;
extern volatile uint8_t debug_control_mode_active;
extern volatile float debug_position_error_deg;
extern volatile float debug_position_output_rad_s;

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

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
