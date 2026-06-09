#ifndef __CONTROL_H__
#define __CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* ======================== 控制/调试模式宏（同类放一起） ======================== */

/* 电机控制模式：速度环。 */
#define MOTOR_CONTROL_MODE_SPEED    0U
/* 电机控制模式：位置环。 */
#define MOTOR_CONTROL_MODE_POSITION 1U

/* 快捷调试模式：速度环。 */
#define DEBUG_QUICK_MODE_SPEED              0U
/* 快捷调试模式：位置环。 */
#define DEBUG_QUICK_MODE_POSITION           1U
/* 快捷调试模式：遥控器速度环（ch3 摇杆映射到目标转速）。 */
#define DEBUG_QUICK_MODE_REMOTE             2U
/* 快捷调试模式：遥控器位置环（ch2 摇杆映射到目标角度 0~360°）。 */
#define DEBUG_QUICK_MODE_REMOTE_POSITION    3U
/* 是否锁定单个电机 ID（1: 锁定，0: 跟随反馈自动识别）。 */
#define CONTROL_LOCK_MOTOR_ID_ENABLE 0U
/* 锁定目标电机 ID（C620 反馈 ID = 0x200 + 该值）。 */
#define CONTROL_LOCKED_MOTOR_ID      3U

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

/** @brief 初始化控制模块状态与 PID。 */
void Control_Init(void);
/** @brief 控制任务周期执行入口（建议固定周期调用）。 */
void Control_Task(float dt_s);
/** @brief 处理单帧 CAN 接收数据并更新反馈状态。 */
void Control_OnCanRxMessage(const CAN_RxHeaderTypeDef *rx_header, const uint8_t rx_data[8]);

/** @brief 设置速度环 PID 参数。 */
void Control_SetSpeedPidParams(const ControlSpeedPidParams *params);
/** @brief 设置位置环 PID 参数。 */
void Control_SetPositionPidParams(const ControlPositionPidParams *params);
/** @brief 清零速度环与位置环内部积分/历史误差状态。 */
void Control_ResetPidState(void);

/** @brief 设置位置环目标角度（单位：deg）。 */
void MotorControl_SetTargetAngle(float target_angle_deg);
/** @brief 角度清零并复位位置环状态。 */
void MotorControl_ResetAngle(void);
/** @brief 选择控制模式：MOTOR_CONTROL_MODE_SPEED / MOTOR_CONTROL_MODE_POSITION。 */
void MotorControl_SetControlMode(uint8_t control_mode);
/** @brief 设置速度环目标速度（单位：rad/s）。 */
void MotorControl_SetTargetSpeed(float target_speed_rad_s);
/** @brief 启动相对转圈：输出轴转动 turns 圈（正=顺时针，受 debug_gear_ratio 减速比影响）。 */
void Control_StartTurns(float turns);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H__ */
