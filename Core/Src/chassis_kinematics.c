/**
  ******************************************************************************
  * @file    chassis_kinematics.c
  * @brief   底盘运动学解算实现（全向轮 X 型切线布局）
  *          - 遥控器通道 → 底盘目标运动（vx, vy, omega）
  *          - 底盘目标运动 → 4 电机目标转速（逆运动学）
  ******************************************************************************
  */

#include "chassis_kinematics.h"
#include "debug.h"    /* debug_remote_deadzone, debug_gear_ratio, MOTOR_COUNT */

#include <stddef.h>   /* NULL */

/* ======================== 预计算常量 ======================== */

/* √2 ≈ 1.41421356，避免引入 math 库增加代码体积 */
#define SQRT2 1.41421356f

/* 遥控器摇杆中位值（DR16 规格，4 通道一致） */
#define RC_MID_VALUE  1024

/* 遥控器摇杆单侧最大偏移量（1684 - 1024 = 1024 - 364 = 660） */
#define RC_MAX_OFFSET 660

/* ======================== 运动学可调参数与诊断变量 ========================
 *
 *   所有运动学相关的 volatile 变量定义已统一迁移到 debug.c
 *   （第 4 块 — 运动学可调参数，第 7 块 — 运动学诊断变量）。
 *   本文件通过 debug.h 的 extern 声明访问这些变量。
 * ======================================================================== */

/* ======================== 运动学解算函数实现 ======================== */

/**
 * @brief  将 DR16 遥控器通道值解算为底盘目标运动（vx, vy, omega）。
 *
 *         通道映射（DR16 规格）：
 *           ch0（右摇杆水平）→ 旋转（右=CW 顺时针=omega<0，左=CCW 逆时针=omega>0）
 *           ch1（右摇杆垂直）→ 预留未使用
 *           ch2（左摇杆水平）→ X 平移（右=正，左=负）
 *           ch3（左摇杆垂直）→ Y 平移（上=前=正，下=后=负）
 *           （注意：DJI DR16 推杆向前 ch3 值减小，故 vy ∝ 1024 - ch3）
 *
 *         死区：|offset| <= debug_remote_deadzone 时对应通道输出 0，
 *         防止摇杆中位抖动导致电机微动。
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
    float *vx, float *vy, float *omega)
{
    int32_t offset;

    /* 右摇杆垂直通道（ch1）预留，暂不使用 — 后续可扩展为二级旋转控制或其他功能 */
    (void)ch1;

    /* 指针有效性检查：任一为 NULL 则退出，避免非法内存访问 */
    if (vx == NULL || vy == NULL || omega == NULL)
    {
        return;
    }

    /* ====== X 方向平移：ch2（左摇杆水平），右=正 ======
     * （符号方向由电机安装方向决定，若左右反了就改这里的 offset 符号） */
    offset = (int32_t)RC_MID_VALUE - (int32_t)ch2;
    if ((uint32_t)(offset > 0 ? offset : -offset) <= (uint32_t)debug_remote_deadzone)
    {
        *vx = 0.0f;
    }
    else
    {
        /* 线性映射：offset / 660 * max_speed，满偏时达到 debug_kinematics_max_speed_ms */
        *vx = (float)offset / (float)RC_MAX_OFFSET * debug_kinematics_max_speed_ms;
    }

    /* ====== Y 方向平移：ch3（左摇杆垂直），前=正 ====== */
    /* DJI DR16 推杆向前（上）→ ch3 值减小（趋向 364）→ offset = 中位 - ch3 > 0 */
    offset = (int32_t)RC_MID_VALUE - (int32_t)ch3;
    if ((uint32_t)(offset > 0 ? offset : -offset) <= (uint32_t)debug_remote_deadzone)
    {
        *vy = 0.0f;
    }
    else
    {
        *vy = (float)offset / (float)RC_MAX_OFFSET * debug_kinematics_max_speed_ms;
    }

    /* ====== 旋转：ch0（右摇杆水平），右=CCW=正，左=CW=负 ====== */
    /* DJI DR16 推杆向右 → ch0 值增大 → offset 为正 → omega 为正（CCW 逆时针）
     * （符号方向由电机安装方向决定，若旋转反了就改这里的 offset 符号） */
    offset = (int32_t)ch0 - (int32_t)RC_MID_VALUE;
    if ((uint32_t)(offset > 0 ? offset : -offset) <= (uint32_t)debug_remote_deadzone)
    {
        *omega = 0.0f;
    }
    else
    {
        *omega = (float)offset / (float)RC_MAX_OFFSET * debug_kinematics_max_omega_rad_s;
    }

    /*
     * 注意：CCW 偏航修正（debug_kinematics_ccw_correction_gain）和
     * 速度矢量偏向角修正（debug_kinematics_bias_angle_deg）已从本函数
     * 移至 main.c 的遥控器路径中执行。
     *
     * 原因：这两项修正仅在遥控器模式下生效（仅作用于遥控器解算出的速度矢量），
     * 制动等其他路径不应受其影响。将此修正放在运动学通用库函数中会导致
     * 职责不清，增加调试难度。
     *
     * 当前本函数仅做纯摇杆通道 → 底盘速度映射（含死区），不做任何硬件补偿。 */
}

/**
 * @brief  将底盘目标运动（vx, vy, omega）解算为 4 个电机的目标转速。
 *
 *         布局说明（俯视图，Y+=前，X+=右，轮位构成正方形，半轴长 L）：
 *
 *           电机 3（ID 4）                 电机 0（ID 1）
 *           左前 (-L,+L)                   右前 (+L,+L)
 *           滚动方向 225°                  滚动方向 135°
 *           n = [-1/√2, -1/√2]             n = [-1/√2, +1/√2]
 *
 *           电机 2（ID 3）                 电机 1（ID 2）
 *           左后 (-L,-L)                   右后 (+L,-L)
 *           滚动方向 315°                  滚动方向 45°
 *           n = [+1/√2, -1/√2]             n = [+1/√2, +1/√2]
 *
 *         所有电机滚动方向为切线方向（垂直于从中心到轮位的径向），
 *         与 X、Y 轴均呈 45°。此布局使全部 4 个电机均等地参与平移和旋转。
 *
 *         逆运动学推导：
 *           全向轮 i 在位置 r_i = (r_x_i, r_y_i) 处的驱动线速度：
 *             v_wheel_i = n_x_i·v_x + n_y_i·v_y + ω·(r_x_i·n_y_i - r_y_i·n_x_i)
 *           电机转速（rad/s）= v_wheel_i · gear_ratio / wheel_radius
 *
 *           关键：所有 4 轮的 r×n = r_x·n_y - r_y·n_x = L√2（常数），
 *           因此旋转分量 ω·L√2 对全部电机一致。
 *
 *         代入各轮参数（K = gear_ratio / (√2 · wheel_radius)）：
 *           电机 0 (ID 1, +L,+L, 135°): motor = K·(-v_x + v_y + 2L·ω)
 *           电机 1 (ID 2, +L,-L,  45°): motor = K·(+v_x + v_y + 2L·ω)
 *           电机 2 (ID 3, -L,-L, 315°): motor = K·(+v_x - v_y + 2L·ω)
 *           电机 3 (ID 4, -L,+L, 225°): motor = K·(-v_x - v_y + 2L·ω)
 *
 *         验证（前向 pure +Y, ω=0）：motor ∝ [+1, +1, -1, -1]·v_y
 *           右前(135°)正转→推力(-X,+Y)，右后(45°)正转→推力(+X,+Y)，
 *           左后(315°)反转→推力(-X,+Y)，左前(225°)反转→推力(+X,+Y)
 *           合力 (0, 4·v_y) → 纯前向。✓
 *
 *         验证（CCW 旋转 pure +ω, v=0）：motor ∝ ω·2L·[+1, +1, +1, +1]
 *           4 轮同向正转，全部产生 CCW 扭矩，无平移分量。✓
 *
 * @param  vx           底盘 X 方向目标速度（m/s，正值=右）
 * @param  vy           底盘 Y 方向目标速度（m/s，正值=前）
 * @param  omega        底盘目标旋转角速度（rad/s，正值=CCW 逆时针）
 * @param  motor_speeds  [out] 4 个电机的目标转速（rad/s，电机轴侧）
 */
void ChassisKinematics_ChassisToMotors(
    float vx, float vy, float omega,
    float motor_speeds[WHEEL_COUNT])
{
    if (motor_speeds == NULL)
    {
        return;
    }

    /* 配置有效性检查：radius 和 gear_ratio 必须 > 0，否则除零或反向 */
    float radius = debug_kinematics_wheel_radius_m;
    float base_l = debug_kinematics_wheel_base_m;
    float gear_r = debug_gear_ratio;        /* 复用 debug.c 的减速比（3591/187 ≈ 19.2） */

    if (radius <= 0.0f || gear_r <= 0.0f)
    {
        /* 配置无效 → 全部电机输出 0，安全停机 */
        motor_speeds[0] = 0.0f;
        motor_speeds[1] = 0.0f;
        motor_speeds[2] = 0.0f;
        motor_speeds[3] = 0.0f;
        return;
    }

    /* K = gear_ratio / (√2 · wheel_radius)：公共增益（rad·s⁻¹ / m·s⁻¹） */
    float K = gear_r / (SQRT2 * radius);

    /* 旋转分量：2L·ω（r×n/√2 = L√2/√2 = L，再 ×2 是因为 2L 合并），分母的 √2 已归入 K */
    float rot_term = 2.0f * base_l * omega;

    /* ====== 各电机目标转速 ====== */

    /* 电机 0（ID 1，右前 +L,+L，n=[-1/√2, +1/√2]） */
    motor_speeds[0] = K * (-vx + vy + rot_term);

    /* 电机 1（ID 2，右后 +L,-L，n=[+1/√2, +1/√2]） */
    motor_speeds[1] = K * (+vx + vy + rot_term);

    /* 电机 2（ID 3，左后 -L,-L，n=[+1/√2, -1/√2]） */
    motor_speeds[2] = K * (+vx - vy + rot_term);

    /* 电机 3（ID 4，左前 -L,+L，n=[-1/√2, -1/√2]） */
    motor_speeds[3] = K * (-vx - vy + rot_term);

    /* ====== 同步诊断变量（Ozone 观察窗可实时查看） ====== */
    debug_chassis_vx_ms       = vx;
    debug_chassis_vy_ms       = vy;
    debug_chassis_omega_rad_s = omega;
    debug_kinematics_motor_speed[0] = motor_speeds[0];
    debug_kinematics_motor_speed[1] = motor_speeds[1];
    debug_kinematics_motor_speed[2] = motor_speeds[2];
    debug_kinematics_motor_speed[3] = motor_speeds[3];
}

/**
 * @brief  前向运动学：从 4 电机实际转速反算底盘运动（vx, vy, omega）。
 *
 *         逆矩阵推导（由逆运动学公式求逆）：
 *           设 motor[0] = K·(-vx + vy + 2L·ω)
 *              motor[1] = K·(+vx + vy + 2L·ω)
 *              motor[2] = K·(+vx - vy + 2L·ω)
 *              motor[3] = K·(-vx - vy + 2L·ω)
 *           其中 K = gear_ratio / (√2 · wheel_radius)
 *
 *           解方程：
 *             vx = (m1 + m2 - m0 - m3) / (4·K)
 *             vy = (m0 + m1 - m2 - m3) / (4·K)
 *           omega = (m0 + m1 + m2 + m3) / (8·K·L)
 *
 *         验证（pure +Y forward, ω=0）：
 *           m0=K·vy, m1=K·vy, m2=-K·vy, m3=-K·vy
 *           vx = (Kvy+K(-vy) - Kvy - K(-vy)) / (4K) = 0  ✓
 *           vy = (Kvy+Kvy - K(-vy) - K(-vy)) / (4K) = vy  ✓
 *
 * @param  motor_speeds  [in]  4 个电机的实际转速（rad/s，电机轴侧）
 * @param  vx            [out] 底盘 X 方向速度（m/s，正值=右）
 * @param  vy            [out] 底盘 Y 方向速度（m/s，正值=前）
 * @param  omega         [out] 底盘旋转角速度（rad/s，正值=CCW）
 */
void ChassisKinematics_MotorsToChassis(
    const volatile float motor_speeds[WHEEL_COUNT],
    float *vx, float *vy, float *omega)
{
    if (motor_speeds == NULL || vx == NULL || vy == NULL || omega == NULL)
    {
        return;
    }

    float radius = debug_kinematics_wheel_radius_m;
    float base_l = debug_kinematics_wheel_base_m;
    float gear_r = debug_gear_ratio;

    /* K = gear_ratio / (√2 · wheel_radius) */
    if (radius <= 0.0f || gear_r <= 0.0f)
    {
        *vx    = 0.0f;
        *vy    = 0.0f;
        *omega = 0.0f;
        return;
    }

    float K      = gear_r / (SQRT2 * radius);
    float m0     = motor_speeds[0];
    float m1     = motor_speeds[1];
    float m2     = motor_speeds[2];
    float m3     = motor_speeds[3];
    float sum_m  = m0 + m1 + m2 + m3;

    /* 若所有电机转速均为 0，直接返回避免除零（K > 0 始终成立但 L 可能为 0） */
    if (sum_m == 0.0f && m0 == 0.0f && m1 == 0.0f && m2 == 0.0f && m3 == 0.0f)
    {
        *vx    = 0.0f;
        *vy    = 0.0f;
        *omega = 0.0f;
        return;
    }

    *vx    = (m1 + m2 - m0 - m3) / (4.0f * K);
    *vy    = (m0 + m1 - m2 - m3) / (4.0f * K);
    *omega = (base_l > 0.0f) ? sum_m / (8.0f * K * base_l) : 0.0f;
}
