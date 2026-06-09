/**
  ******************************************************************************
  * @file    chassis_kinematics.h
  * @brief   底盘运动学解算模块（全向轮 X 型切线布局）
  *          - 遥控器通道 → 底盘目标运动（vx, vy, omega）
  *          - 底盘目标运动 → 4 电机目标转速（逆运动学）
  *
  *          布局说明（俯视图，Y+=前，X+=右，轮位构成正方形，半轴长 L）：
  *
  *            电机 3（ID 4）                 电机 0（ID 1）
  *            左前 (-L,+L)                   右前 (+L,+L)
  *            滚动方向 225°                  滚动方向 135°
  *            n = [-1/√2, -1/√2]             n = [-1/√2, +1/√2]
  *
  *                        ┌──────────────┐
  *                        │              │
  *                        │   底盘中心   │
  *                        │    (0,0)     │
  *                        │              │
  *                        └──────────────┘
  *
  *            电机 2（ID 3）                 电机 1（ID 2）
  *            左后 (-L,-L)                   右后 (+L,-L)
  *            滚动方向 315°                  滚动方向 45°
  *            n = [+1/√2, -1/√2]             n = [+1/√2, +1/√2]
  *
  *          所有电机滚动方向为切线方向（垂直于径向），与 X、Y 轴均呈 45°。
  *          此布局使全部 4 个电机均等地参与平移和旋转，为全向轮标准 X 构型。
  *
  *          摇杆映射（DR16）：
  *            左摇杆 ch2（左右）→ X 平移    右摇杆 ch0（左右）→ 旋转
  *            左摇杆 ch3（上下）→ Y 平移    右摇杆 ch1（上下）→ 预留
  ******************************************************************************
  */

#ifndef __CHASSIS_KINEMATICS_H__
#define __CHASSIS_KINEMATICS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 轮组数量 ======================== */

#define WHEEL_COUNT 4U

/* ======================== 运动学解算 API ======================== */

/**
 * @brief  将 DR16 遥控器通道值解算为底盘目标运动（vx, vy, omega）。
 *
 *         左摇杆（ch2 左右, ch3 上下）控制底盘前后左右平移；
 *         右摇杆（ch0 左右）控制底盘原地旋转（ch1 预留）。
 *
 *         死区取 debug_remote_deadzone（定义于 debug.c），与旧遥控器逻辑共用。
 *
 * @param  ch0   右摇杆水平（364~1684，中位 1024）
 * @param  ch1   右摇杆垂直（364~1684，中位 1024，预留未使用）
 * @param  ch2   左摇杆水平（364~1684，中位 1024）
 * @param  ch3   左摇杆垂直（364~1684，中位 1024）
 * @param  vx    [out] 底盘 X 方向目标速度（m/s，正值=右）
 * @param  vy    [out] 底盘 Y 方向目标速度（m/s，正值=前）
 * @param  omega [out] 底盘目标旋转角速度（rad/s，正值=CCW 逆时针）
 */
void ChassisKinematics_RemoteToChassis(
    uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3,
    float *vx, float *vy, float *omega);

/**
 * @brief  将底盘目标运动（vx, vy, omega）解算为 4 个电机的目标转速。
 *
 *         逆运动学公式（全向轮切线布局）：
 *           v_wheel_i = n_x_i·v_x + n_y_i·v_y + ω·(r_x_i·n_y_i - r_y_i·n_x_i)
 *           motor_speed_i = v_wheel_i · (gear_ratio / wheel_radius)
 *
 *         代入各轮参数（n 和 r 见文件头图示）：
 *           电机 0 (ID 1): motor = K·(-v_x + v_y + 2L·ω)      K = gear_ratio/(√2·wheel_radius)
 *           电机 1 (ID 2): motor = K·(+v_x + v_y + 2L·ω)
 *           电机 2 (ID 3): motor = K·(+v_x - v_y + 2L·ω)
 *           电机 3 (ID 4): motor = K·(-v_x - v_y + 2L·ω)
 *
 *         全部 4 个电机均等地参与平移和旋转（r×n = L√2 对所有电机恒等），
 *         不存在"仅部分电机参与旋转"的情况。
 *
 * @param  vx           底盘 X 方向目标速度（m/s，正值=右）
 * @param  vy           底盘 Y 方向目标速度（m/s，正值=前）
 * @param  omega        底盘目标旋转角速度（rad/s，正值=CCW 逆时针）
 * @param  motor_speeds  [out] 4 个电机的目标转速（rad/s，电机轴侧，
 *                       可直接传入 MotorControl_SetTargetSpeed()）
 */
void ChassisKinematics_ChassisToMotors(
    float vx, float vy, float omega,
    float motor_speeds[WHEEL_COUNT]);

/**
 * @brief  前向运动学：从 4 电机实际转速反算底盘运动（vx, vy, omega）。
 *
 *         逆矩阵（由逆运动学公式求逆推导）：
 *           K  = gear_ratio / (√2 · wheel_radius)
 *           vx = (m1 + m2 - m0 - m3) / (4·K)
 *           vy = (m0 + m1 - m2 - m3) / (4·K)
 *         omega = (m0 + m1 + m2 + m3) / (8·K·L)
 *
 *         用于制动阶段：从实际电机转速反推底盘运动方向，
 *         以便施加方向一致的底盘级反向制动力。
 *
 * @param  motor_speeds  [in]  4 个电机的实际转速（rad/s，电机轴侧）
 * @param  vx            [out] 底盘 X 方向速度（m/s，正值=右）
 * @param  vy            [out] 底盘 Y 方向速度（m/s，正值=前）
 * @param  omega         [out] 底盘旋转角速度（rad/s，正值=CCW）
 */
void ChassisKinematics_MotorsToChassis(
    const volatile float motor_speeds[WHEEL_COUNT],
    float *vx, float *vy, float *omega);

/* ======================== 运动学可调参数与诊断变量 ========================
 *
 *   所有运动学相关的可调参数（底盘尺寸、速度限制、CCW 修正）
 *   和诊断变量（vx/vy/omega 快照、电机解算转速）已统一迁移到
 *   debug.h / debug.c 的 第 4 块和第 7 块 中集中管理。
 *
 *   在 Ozone 中可直接修改和观察这些变量，无需查找多个文件。
 * ======================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_KINEMATICS_H__ */
