#ifndef __CONTROL_H__
#define __CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* ======================== 控制/调试模式宏 ======================== */

/* 电机控制模式：速度环。 */
#define MOTOR_CONTROL_MODE_SPEED    0U
/* 电机控制模式：位置环。 */
#define MOTOR_CONTROL_MODE_POSITION 1U

/* 快捷调试模式：速度环（每个电机从 debug_set_speed[i] 读取目标）。 */
#define DEBUG_QUICK_MODE_SPEED    0U
/* 快捷调试模式：位置环（每个电机从 debug_set_position[i] 读取目标）。 */
#define DEBUG_QUICK_MODE_POSITION 1U
/* 快捷调试模式：遥控器模式（运动学占位 — TODO: 底盘运动学解算）。
 * 合并原 REMOTE(2) + REMOTE_POSITION(3) 为统一的遥控器入口。 */
#define DEBUG_QUICK_MODE_REMOTE   2U

/**
 * @brief 速度环 PID 参数
 *
 * - kp/ki/kd: PID 三项增益
 * - output_limit: 速度环输出限幅（对应力矩/电流命令）
 * - integral_limit: 积分状态限幅
 * - integral_separation_threshold: 积分分离阈值，|error| 大于该值时暂停积分
 * - ff_gain: 前馈增益，ff = ff_gain * target_speed
 * - zero_cross_speed_threshold: 速度过零判定阈值（rad/s）
 * - zero_cross_brake_current: 过零反向时追加的制动电流（iq）
 *
 * @note 此结构体仅用于类型定义，实际参数存储于 debug.c 的 volatile float 数组中，
 *       Ozone 直接修改 debug_speed_xxx[i] 即可实时调参，无需通过此 API。
 */
typedef struct
{
  float kp;
  float ki;
  float kd;
  float output_limit;
  float integral_limit;
  float integral_separation_threshold;
  float ff_gain;
  float zero_cross_speed_threshold;
  float zero_cross_brake_current;
} ControlSpeedPidParams;

/**
 * @brief 位置环 PID 参数
 *
 * - kp/ki/kd: PID 三项增益
 * - output_limit: 位置环输出限幅（通常作为目标速度限幅）
 * - integral_limit: 积分状态限幅
 * - integral_separation_threshold: 积分分离阈值
 */
typedef struct
{
  float kp;
  float ki;
  float kd;
  float output_limit;
  float integral_limit;
  float integral_separation_threshold;
} ControlPositionPidParams;

/* ======================== API 函数 ======================== */

/** @brief 初始化控制模块：4 个电机的 PID 状态、反馈、模式（默认速度环）。 */
void Control_Init(void);

/** @brief 控制任务周期执行入口（建议 10ms 固定周期调用）。
 *  内部遍历 4 个电机：PID 级联 + 启动探测 + CAN 发送。 */
void Control_Task(float dt_s);

/** @brief 处理单帧 CAN 接收数据，路由到对应电机的反馈状态。
 *  @param rx_header CAN 帧头（StdId 0x201~0x204 → 电机 1~4）
 *  @param rx_data   8 字节 CAN 数据 */
void Control_OnCanRxMessage(const CAN_RxHeaderTypeDef *rx_header, const uint8_t rx_data[8]);

/** @brief 清零全部 8 个 PID 控制器的内部积分/历史误差状态。 */
void Control_ResetPidState(void);

/** @brief 设置指定电机的控制模式。
 *  @param motor_id     电机索引（0~3，对应 CAN ID 1~4）
 *  @param control_mode MOTOR_CONTROL_MODE_SPEED 或 MOTOR_CONTROL_MODE_POSITION */
void MotorControl_SetControlMode(uint8_t motor_id, uint8_t control_mode);

/** @brief 设置指定电机的速度环目标速度。
 *  @param motor_id          电机索引（0~3）
 *  @param target_speed_rad_s 目标转速（rad/s，正值=正转） */
void MotorControl_SetTargetSpeed(uint8_t motor_id, float target_speed_rad_s);

/** @brief 设置指定电机的位置环目标角度。
 *  @param motor_id         电机索引（0~3）
 *  @param target_angle_deg 目标角度（deg，自动归一化到 [0, 360)） */
void MotorControl_SetTargetAngle(uint8_t motor_id, float target_angle_deg);

/** @brief 将指定电机的位置目标复位为当前角度，并清零其位置环 PID 状态。 */
void MotorControl_ResetAngle(uint8_t motor_id);

/** @brief 启动指定电机的相对转圈。
 *  @param motor_id 电机索引（0~3）
 *  @param turns    输出轴转动圈数（正=顺时针，受 debug_gear_ratio 影响）
 *  @note  仅在 feedback_valid[motor_id] 为真时生效。 */
void Control_StartTurns(uint8_t motor_id, float turns);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H__ */
