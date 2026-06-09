#include "control.h"

#include <math.h>

#include "can.h"
#include "debug.h"
#include "pid.h"

/* ======================== CAN 协议常量 ======================== */

/* C620 电流控制帧 ID（控制 1~4 号电机）。 */
#define CAN_MOTOR_TX_STDID_LOW    0x200U
/* C620 电流控制帧 ID（控制 5~8 号电机）。 */
#define CAN_MOTOR_TX_STDID_HIGH   0x1FFU
/* 电机反馈帧最小 ID。 */
#define CAN_MOTOR_RX_STDID_MIN    0x201U
/* 电机反馈帧最大 ID。 */
#define CAN_MOTOR_RX_STDID_MAX    0x208U
/* 圆周率常量。 */
#define PI_F                      3.14159265358979323846f
/* RPM 转换为 rad/s 的系数。 */
#define RPM_TO_RADPS              (2.0f * PI_F / 60.0f)
/* 无反馈启动阶段探测电流（iq）。 */
#define MOTOR_STARTUP_CURRENT     2500
/* 无反馈启动探测超时时间（ms）。 */
#define MOTOR_STARTUP_TIMEOUT_MS  1500U
/* 编码器一圈计数（0~8191 共 8192 点）。 */
#define MOTOR_ENCODER_CPR         8192.0f
/* 胡萝卜挂在前方度数（转圈模式）。 */
#define TURNS_CARROT_DEG  30.0f

/* ======================== 控制器内部状态区（每电机独立） ======================== */

/* PID 控制器实例：每个电机独立一套速度环 + 位置环。 */
static PIDController speed_pid[MOTOR_COUNT];
static PIDController position_pid[MOTOR_COUNT];

/* 控制模式（每电机独立：速度环或位置环）。 */
static uint8_t control_mode[MOTOR_COUNT];

/* 每电机控制目标。 */
static float target_angle_deg[MOTOR_COUNT];
static float target_speed_rad_s[MOTOR_COUNT];
static float prev_target_speed_rad_s[MOTOR_COUNT];     /* 上一周期目标速度，用于检测阶跃 */

/* 电机反馈状态（每电机独立）。 */
static uint8_t  feedback_valid[MOTOR_COUNT];
static uint16_t encoder_raw[MOTOR_COUNT];
static int16_t  motor_rpm[MOTOR_COUNT];
static float    actual_angle_deg[MOTOR_COUNT];
static float    actual_speed_rad_s[MOTOR_COUNT];

/* 每电机控制中间量。 */
static float position_error_deg[MOTOR_COUNT];
static float speed_setpoint_rad_s[MOTOR_COUNT];
static float position_output_rad_s[MOTOR_COUNT];
static float torque_cmd_iq[MOTOR_COUNT];

/* 无反馈时的启动探测状态（每电机独立计时）。 */
static uint32_t no_feedback_elapsed_ms[MOTOR_COUNT];

/* 相对转圈状态（每电机独立）："移动胡萝卜"驱动 + 编码器累计。 */
static uint8_t  turns_active[MOTOR_COUNT];
static float    turns_target_ticks[MOTOR_COUNT];        /* 需累计的编码器脉冲数 */
static float    turns_accumulated_ticks[MOTOR_COUNT];   /* 已累计脉冲数 */
static uint16_t turns_last_encoder[MOTOR_COUNT];
static float    turns_direction_sign[MOTOR_COUNT];      /* +1 顺时针, -1 逆时针 */

/* Ozone 观察用：每个电机当前胡萝卜目标角度。 */
static float debug_target_angle_deg_watch[MOTOR_COUNT];

/* ======================== 工具函数（与单电机版本一致） ======================== */

/* 将任意角度归一化到 [0, 360)。 */
static float Control_NormalizeAngleDeg(float angle_deg)
{
  float wrapped = fmodf(angle_deg, 360.0f);
  if (wrapped < 0.0f)
  {
    wrapped += 360.0f;
  }
  return wrapped;
}

/* 计算最短圆弧角差，输出范围为 [-180, 180]。
 *
 * 方向偏好：当双向路径接近等长（|error| ≈ 180°）时，优先选择减小角度
 * （error 取负值）。原因：
 *   1. 编码器在 180° 对面的浮点抖动会使 error 在 +180° 和 -180° 之间
 *      反复翻转，D 项每次产生巨大冲量（Δerror≈360°/dt），电机原地高频
 *      振动，永远到不了目标。
 *   2. 统一走减小方向消除翻转，方向确定性 >> 最多 1° 的额外路径。
 *
 * 阈值 179.0° 保证 ±1° 范围内的 180° 对称点都锁定为减小方向。 */
static float Control_ShortestArcErrorDeg(float target_deg, float current_deg)
{
  float error = target_deg - current_deg;

  /* 正向超 180°：绕一圈走反向（减小角度）。 */
  if (error > 180.0f)
  {
    error -= 360.0f;
  }
  /* 负向超 180°：绕一圈走正向。 */
  else if (error < -180.0f)
  {
    error += 360.0f;
  }

  /* ====== 双向接近等距时的方向锁定：优先减小角度 ======
   *
   * 当 error 接近 +180°（即正向和反向路径长度差 < 2°）时，
   * 强制走反向（error -= 360 → 变为负值）。
   *
   * 例如：target=0°, current=180.04° → error=-180.04 → < -180
   *      → error+=360=179.96 → > 179.0 → error-=360=-180.04
   *      → 结果: 锁定在减小方向，避免与 -179.96 之间来回翻转。 */
  if (error > 179.0f)
  {
    error -= 360.0f;
  }

  return error;
}

/* ======================== PID 参数同步 ======================== */

/* 将可调 PID 参数（debug_xxx 数组）同步到运行中的 PID 控制器实例。
 * 每个电机独立从自己的 debug 数组槽位读取参数。 */
static void Control_ApplyPidParams(void)
{
  uint8_t i;
  for (i = 0; i < MOTOR_COUNT; i++)
  {
    /* --- 速度环参数 --- */
    speed_pid[i].kp = debug_speed_kp[i];
    speed_pid[i].ki = debug_speed_ki[i];
    speed_pid[i].kd = debug_speed_kd[i];
    speed_pid[i].derivative_alpha = debug_speed_derivative_alpha[i];
    speed_pid[i].output_limit = fabsf(debug_speed_output_limit[i]);
    speed_pid[i].integral_limit = fabsf(debug_speed_integral_limit[i]);
    speed_pid[i].integral_separation_threshold = fabsf(debug_speed_integral_separation[i]);
    speed_pid[i].integral = PID_Clamp(speed_pid[i].integral, speed_pid[i].integral_limit);

    /* --- 位置环参数 --- */
    position_pid[i].kp = debug_position_kp[i];
    position_pid[i].ki = debug_position_ki[i];
    position_pid[i].kd = debug_position_kd[i];
    position_pid[i].derivative_alpha = debug_position_derivative_alpha[i];
    position_pid[i].output_limit = fabsf(debug_position_output_limit[i]);
    position_pid[i].integral_limit = fabsf(debug_position_integral_limit[i]);
    position_pid[i].integral_separation_threshold = fabsf(debug_position_integral_separation[i]);
    position_pid[i].integral = PID_Clamp(position_pid[i].integral, position_pid[i].integral_limit);
  }
}

/* ======================== CAN 通信 ======================== */

/* 打包并发送 C620 电流控制帧。
 * @param stdid CAN 标准帧 ID（0x200 控制电机 1~4，0x1FF 控制电机 5~8）
 * @param iq1~iq4 四路电流命令（int16，范围 -10000~10000） */
static HAL_StatusTypeDef Control_SendCurrentFrame(uint32_t stdid, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8];
  uint32_t tx_mailbox = 0U;

  tx_header.StdId = stdid;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = (uint8_t)(((uint16_t)iq1) >> 8);
  tx_data[1] = (uint8_t)(iq1 & 0xFF);
  tx_data[2] = (uint8_t)(((uint16_t)iq2) >> 8);
  tx_data[3] = (uint8_t)(iq2 & 0xFF);
  tx_data[4] = (uint8_t)(((uint16_t)iq3) >> 8);
  tx_data[5] = (uint8_t)(iq3 & 0xFF);
  tx_data[6] = (uint8_t)(((uint16_t)iq4) >> 8);
  tx_data[7] = (uint8_t)(iq4 & 0xFF);

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
  {
    return HAL_BUSY;
  }

  return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

/* ======================== 初始化 ======================== */

/* 初始化控制模块：4 个电机的 PID 状态、反馈、模式。
 * 默认全部初始化为速度环模式，配合遥控器模式启动。 */
void Control_Init(void)
{
  uint8_t i;

  for (i = 0; i < MOTOR_COUNT; i++)
  {
    /* 默认速度环模式（配合遥控器）。 */
    control_mode[i] = MOTOR_CONTROL_MODE_SPEED;

    /* 目标归零。 */
    target_angle_deg[i] = 0.0f;
    target_speed_rad_s[i] = 0.0f;
    prev_target_speed_rad_s[i] = 0.0f;

    /* 反馈无效，等待 CAN 消息。 */
    feedback_valid[i] = 0U;
    encoder_raw[i] = 0U;
    motor_rpm[i] = 0;
    actual_angle_deg[i] = 0.0f;
    actual_speed_rad_s[i] = 0.0f;

    /* 中间量归零。 */
    position_error_deg[i] = 0.0f;
    speed_setpoint_rad_s[i] = 0.0f;
    position_output_rad_s[i] = 0.0f;
    torque_cmd_iq[i] = 0.0f;

    /* 启动探测计时归零。 */
    no_feedback_elapsed_ms[i] = 0U;

    /* 转圈状态归零。 */
    turns_active[i] = 0U;
    turns_target_ticks[i] = 0.0f;
    turns_accumulated_ticks[i] = 0.0f;
    turns_last_encoder[i] = 0U;
    turns_direction_sign[i] = 1.0f;

    debug_target_angle_deg_watch[i] = 0.0f;

    /* 初始化速度环 PID。 */
    PID_Init(&speed_pid[i],
             debug_speed_kp[i],
             debug_speed_ki[i],
             debug_speed_kd[i],
             fabsf(debug_speed_output_limit[i]),
             fabsf(debug_speed_integral_limit[i]),
             fabsf(debug_speed_integral_separation[i]));

    /* 初始化位置环 PID。 */
    PID_Init(&position_pid[i],
             debug_position_kp[i],
             debug_position_ki[i],
             debug_position_kd[i],
             fabsf(debug_position_output_limit[i]),
             fabsf(debug_position_integral_limit[i]),
             fabsf(debug_position_integral_separation[i]));
  }
}

/* ======================== CAN 接收处理 ======================== */

/* 消化一帧 CAN 反馈，路由到对应电机的反馈状态。
 *
 * 接收 StdId 范围 0x201~0x208（电机 1~8），仅处理 0x201~0x204（电机 1~4）。
 * 0x201 → 电机 1 (索引 0), 0x202 → 电机 2 (索引 1), 以此类推。 */
void Control_OnCanRxMessage(const CAN_RxHeaderTypeDef *rx_header, const uint8_t rx_data[8])
{
  uint8_t rx_motor_id;

  if ((rx_header == 0) || (rx_data == 0))
  {
    return;
  }

  if ((rx_header->IDE != CAN_ID_STD) || (rx_header->DLC != 8U))
  {
    return;
  }

  if ((rx_header->StdId < CAN_MOTOR_RX_STDID_MIN) || (rx_header->StdId > CAN_MOTOR_RX_STDID_MAX))
  {
    return;
  }

  /* StdId - 0x201 → 0-based 索引。电机 1~4 → 索引 0~3。 */
  rx_motor_id = (uint8_t)(rx_header->StdId - 0x201U);

  /* 只处理电机 1~4（索引 0~3）。 */
  if (rx_motor_id >= MOTOR_COUNT)
  {
    return;
  }

  /* 更新对应电机的反馈状态。 */
  encoder_raw[rx_motor_id] = (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
  actual_angle_deg[rx_motor_id] = ((float)encoder_raw[rx_motor_id] * 360.0f) / MOTOR_ENCODER_CPR;
  motor_rpm[rx_motor_id] = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
  actual_speed_rad_s[rx_motor_id] = (float)motor_rpm[rx_motor_id] * RPM_TO_RADPS;
  feedback_valid[rx_motor_id] = 1U;

  /* 同步诊断变量（该电机槽位）。 */
  debug_feedback_valid[rx_motor_id] = 1U;
  debug_actual_angle_deg[rx_motor_id] = actual_angle_deg[rx_motor_id];
  debug_actual_speed_rad_s[rx_motor_id] = actual_speed_rad_s[rx_motor_id];
}

/* ======================== 主控制任务 ======================== */

/* 主控制周期：对 4 个电机独立执行 PID 级联，打包发送一帧 CAN 电流命令。
 *
 * 每个电机的 PID 级联逻辑与单电机版本完全一致：
 *   位置环（可选）→ 速度环 → 力矩电流 → CAN 0x200 帧。
 *
 * 对于尚未收到反馈的电机，发送启动探测电流直到超时。 */
void Control_Task(float dt_s)
{
  uint32_t dt_ms;
  uint8_t i;

  if (dt_s <= 0.0f)
  {
    return;
  }

  debug_control_task_count++;

  /* 同步所有 PID 参数（Ozone 可能在任意时刻修改了 debug_xxx 数组）。 */
  Control_ApplyPidParams();

  dt_ms = (uint32_t)(dt_s * 1000.0f + 0.5f);
  if (dt_ms == 0U)
  {
    dt_ms = 1U;
  }

  /* ====== 遍历 4 个电机，独立计算电流命令 ====== */
  for (i = 0; i < MOTOR_COUNT; i++)
  {
    float position_error;
    float startup_dir;

    /* 同步该电机当前控制模式到诊断变量。 */
    debug_control_mode_active[i] = control_mode[i];

    /* ========== 无反馈：启动探测 ========== */
    if (feedback_valid[i] == 0U)
    {
      if (no_feedback_elapsed_ms[i] <= (0xFFFFFFFFU - dt_ms))
      {
        no_feedback_elapsed_ms[i] += dt_ms;
      }

      /* 探测方向：位置环默认正方向，速度环跟随目标符号。 */
      if (control_mode[i] == MOTOR_CONTROL_MODE_POSITION)
      {
        startup_dir = 1.0f;
      }
      else
      {
        startup_dir = (target_speed_rad_s[i] >= 0.0f) ? 1.0f : -1.0f;
      }

      /* 超时前持续探测，让电机获得启动电流。 */
      if ((no_feedback_elapsed_ms[i] <= MOTOR_STARTUP_TIMEOUT_MS) &&
          (MOTOR_STARTUP_CURRENT > 0))
      {
        torque_cmd_iq[i] = startup_dir * (float)MOTOR_STARTUP_CURRENT;
      }
      else
      {
        torque_cmd_iq[i] = 0.0f;
      }

      /* 该电机尚未上线，跳过 PID 级联。 */
      debug_torque_cmd[i] = torque_cmd_iq[i];
      continue;
    }

    /* ========== 有反馈：正常运行 ========== */

    /* 收到反馈后清零探测计时。 */
    no_feedback_elapsed_ms[i] = 0U;

    /* ---------- 速度环模式 ---------- */
    if (control_mode[i] == MOTOR_CONTROL_MODE_SPEED)
    {
      position_output_rad_s[i] = 0.0f;
      speed_setpoint_rad_s[i] = target_speed_rad_s[i];
      debug_position_error_deg[i] = 0.0f;
      debug_position_output_rad_s[i] = 0.0f;
    }
    /* ---------- 位置环模式 ---------- */
    else
    {
      /* ====== 转圈模式：移动胡萝卜驱动位置环 + 编码器累计判定完成 ======
       *
       * 为什么不用绝对目标：target = current + N*360 经取模后与 current 同位置，
       * 误差=0，电机不动。
       *
       * 胡萝卜法：每周期将目标设在当前角度前方 30°，位置环持续追赶。
       * 总行程用编码器原始脉冲累计（正确处理 0↔8191 绕回），
       * 不受角度归一化影响。 */
      if (turns_active[i])
      {
        int32_t raw_delta = (int32_t)encoder_raw[i] - (int32_t)turns_last_encoder[i];
        if (raw_delta > 4096)       { raw_delta -= 8192; }
        else if (raw_delta < -4096) { raw_delta += 8192; }

        turns_accumulated_ticks[i] += fabsf((float)raw_delta);
        turns_last_encoder[i] = encoder_raw[i];

        if (turns_accumulated_ticks[i] >= turns_target_ticks[i])
        {
          /* 完成：停在当前角度 */
          turns_active[i] = 0U;
          target_angle_deg[i] = actual_angle_deg[i];
          debug_set_position[i] = actual_angle_deg[i];
          position_error_deg[i] = 0.0f;
          position_pid[i].integral = 0.0f;
        }
        else
        {
          /* 胡萝卜挂前方 30°，ShortestArcError 自动选最短方向 */
          target_angle_deg[i] = actual_angle_deg[i]
                              + turns_direction_sign[i] * TURNS_CARROT_DEG;
          debug_target_angle_deg_watch[i] = target_angle_deg[i];
        }
      }

      /* 位置环过零处理：始终按最短角差（<=180°）计算误差。
       * 方向偏好：双向等距时优先减小角度（已在 ShortestArcErrorDeg 内置）。 */
      position_error = Control_ShortestArcErrorDeg(
        Control_NormalizeAngleDeg(target_angle_deg[i]),
        Control_NormalizeAngleDeg(actual_angle_deg[i]));

      /* ====== 线性死区：简单、可预测、无速度墙 ======
       *
       * 为什么不用非线性软着陆：
       *   软着陆的 ramp 衰减在死区边界附近极陡（误差从 3°→2° 时有效值
       *   从 ~0.3° 骤降到 0），形成无形的 "速度墙"。电机冲到墙边后速度
       *   命令太小，无法克服静摩擦 → 停下来 → 速度环积分慢慢累积 →
       *   突然冲过死区 → 又停下来 → 阶梯状爬行。
       *
       * 线性死区的做法：
       *   直接减去死区宽度，余数作为有效误差。例如 deadband=3°, 误差=5°
       *   → 有效误差 = 5-3 = 2°。误差=3.1° → 有效误差 = 0.1°。
       *   过渡连续、平滑、无墙。Kd=0 彻底消除 D 项噪声放大。 */
      {
        float abs_err = fabsf(position_error);
        float db = debug_position_deadband_deg[i];

        if (abs_err <= db)
        {
          position_error = 0.0f;
          position_pid[i].integral = 0.0f;
        }
        else
        {
          /* 保持方向，减去死区 */
          position_error = (position_error > 0.0f ? 1.0f : -1.0f)
                         * (abs_err - db);
        }
      }

      position_error_deg[i] = position_error;
      position_output_rad_s[i] = PID_Calculate(&position_pid[i], position_error, 0.0f, dt_s);
      speed_setpoint_rad_s[i] = position_output_rad_s[i];

      /* 同步诊断变量 */
      debug_position_error_deg[i] = position_error_deg[i];
      debug_position_output_rad_s[i] = position_output_rad_s[i];
    }

    /* ====== 转圈中直接控速：绕过位置环输出限幅 ======
     *
     * 胡萝卜法里位置环的输出上限由 position_output_limit (4.0 rad/s) 钳死，
     * 无论 debug_turns_speed_rad_s 设多大都提不上去。
     *
     * 此处绕开位置环，将 speed_setpoint 直接覆写为期望转速，
     * 速度环 PID 跟随产出对应扭矩。编码器累计与完成判定仍在上方位置分支内处理。 */
    if (turns_active[i] && (turns_accumulated_ticks[i] < turns_target_ticks[i]))
    {
      speed_setpoint_rad_s[i] = turns_direction_sign[i] * fabsf(debug_turns_speed_rad_s);
      position_output_rad_s[i] = speed_setpoint_rad_s[i];
      debug_position_output_rad_s[i] = speed_setpoint_rad_s[i];
    }

    /* ====== 目标速度阶跃检测：积分清零 ======
     *
     * 当速度环目标发生 >2 rad/s 的跳变时，清零速度环积分器。
     * 解决问题：
     *   1. 高→低跳变：旧稳态积分（正值）会对抗 P 项的刹车力，导致减速极慢
     *   2. 低→高跳变：旧稳态积分不适用于新工作点，保留反而引入扰动
     *
     * 位置环不受影响（级联结构中位置环输出平缓变化）。 */
    if (control_mode[i] == MOTOR_CONTROL_MODE_SPEED)
    {
      if (fabsf(target_speed_rad_s[i] - prev_target_speed_rad_s[i]) > 2.0f)
      {
        speed_pid[i].integral = 0.0f;
      }
      prev_target_speed_rad_s[i] = target_speed_rad_s[i];
    }

    /* 过零静止区：目标与实际都足够接近 0 时，清积分并保持 0 输出，抑制抖动。 */
    if ((fabsf(debug_speed_zero_cross_threshold[i]) > 0.0f) &&
        (fabsf(speed_setpoint_rad_s[i]) <= fabsf(debug_speed_zero_cross_threshold[i])) &&
        (fabsf(actual_speed_rad_s[i]) <= fabsf(debug_speed_zero_cross_threshold[i])))
    {
      speed_pid[i].integral = 0.0f;
      speed_setpoint_rad_s[i] = 0.0f;
    }

    /* 速度环 PID + 前馈。 */
    torque_cmd_iq[i] = PID_Calculate(&speed_pid[i], speed_setpoint_rad_s[i], actual_speed_rad_s[i], dt_s);
    torque_cmd_iq[i] += speed_setpoint_rad_s[i] * debug_speed_ff_gain[i];

    /* ====== 超速制动增强 ======
     *
     * 当实际速度显著高于目标速度时（超速 >0.5 rad/s），按超速量等比例
     * 叠加制动电流。解决纯 P 控制在大偏差下制动不足的问题：
     *   - Kp=80, error=20 → P 只产生 1600 iq 刹车（16% 能力）
     *   - 叠加 20×150=3000 iq → 总计 4600 iq 刹车（46% 能力）
     *
     * 稳态（超速 <0.5 rad/s）不触发，不影响正常调节精度。 */
    if ((control_mode[i] == MOTOR_CONTROL_MODE_SPEED) &&
        (actual_speed_rad_s[i] > speed_setpoint_rad_s[i] + 0.5f))
    {
      float overspeed = actual_speed_rad_s[i] - speed_setpoint_rad_s[i];
      torque_cmd_iq[i] -= overspeed * 150.0f;
    }

    /* 过零制动：目标与实际方向相反时增加制动补偿，帮助平稳快速过零。 */
    if ((fabsf(debug_speed_zero_cross_threshold[i]) > 0.0f) &&
        (fabsf(debug_speed_zero_cross_brake[i]) > 0.0f) &&
        (speed_setpoint_rad_s[i] * actual_speed_rad_s[i] < 0.0f) &&
        (fabsf(speed_setpoint_rad_s[i]) > fabsf(debug_speed_zero_cross_threshold[i])) &&
        (fabsf(actual_speed_rad_s[i]) > fabsf(debug_speed_zero_cross_threshold[i])))
    {
      torque_cmd_iq[i] += (actual_speed_rad_s[i] > 0.0f)
                        ? -fabsf(debug_speed_zero_cross_brake[i])
                        : fabsf(debug_speed_zero_cross_brake[i]);
    }

    /* ====== 静摩擦补偿（仅位置环） ======
     *
     * 仅位置环启用。速度环由用户直接控制目标速度，不需要自动补偿。
     *
     * 补偿量随位置误差增减：误差大→全量补偿（突破静摩擦），
     * 误差小→线性缩减至 0（微调时不引入冲击）。 */
    if ((control_mode[i] == MOTOR_CONTROL_MODE_POSITION) &&
        (fabsf(speed_setpoint_rad_s[i]) > 0.5f) && (fabsf(actual_speed_rad_s[i]) < 0.2f))
    {
      float stiction_comp = debug_speed_stiction_comp_iq[i];
      float abs_pos_err = fabsf(position_error_deg[i]);
      float thresh = fabsf(debug_position_stiction_threshold_deg[i]);

      if (thresh > 0.0f && abs_pos_err < thresh)
      {
        /* 位置误差 < 阈值：补偿量线性缩减。
         * 例如 threshold=10°, 误差=2° → 补偿 = 500 × 0.2 = 100 iq
         *       误差=0°(死区内) → position_error_deg=0 → 补偿 = 0 */
        stiction_comp *= (abs_pos_err / thresh);
      }

      torque_cmd_iq[i] += (speed_setpoint_rad_s[i] > 0.0f) ? stiction_comp : -stiction_comp;
    }

    /* ====== 硬停逻辑：target_speed=0 时强制电流归零 ======
     *
     * 无条件切断——不检查 actual_speed。只要速度环目标为 0，立即清零积分
     * 并将输出电流置 0，覆盖 PID/前馈/制动/静摩擦等所有中间计算结果。
     *
     * 这是用户明确要求的行为：set speed=0 时电机必须停转，不允许 PID
     * 有任何输出（即使电机仍在惯性滑行）。电机靠自身摩擦减速至静止。
     *
     * 位置环模式不受影响——位置保持需要持续转矩对抗负载。 */
    if ((control_mode[i] == MOTOR_CONTROL_MODE_SPEED) &&
        (fabsf(target_speed_rad_s[i]) < 0.001f))
    {
      speed_pid[i].integral = 0.0f;
      torque_cmd_iq[i] = 0.0f;
    }

    /* 同步该电机的扭矩诊断变量。 */
    debug_torque_cmd[i] = torque_cmd_iq[i];
  }

  /* ====== 打包发送一帧 0x200（四路电流，电机 1~4 → iq1~iq4） ====== */
  {
    int16_t iq_int[4];
    for (i = 0; i < MOTOR_COUNT; i++)
    {
      iq_int[i] = (int16_t)PID_Clamp(torque_cmd_iq[i], 10000.0f);
    }
    (void)Control_SendCurrentFrame(CAN_MOTOR_TX_STDID_LOW, iq_int[0], iq_int[1], iq_int[2], iq_int[3]);
  }
}

/* ======================== API 函数 ======================== */

/* 设置指定电机的控制模式。切换模式时自动清零该电机的 PID 状态，避免积分残留。 */
void MotorControl_SetControlMode(uint8_t motor_id, uint8_t mode)
{
  uint8_t new_mode;

  if (motor_id >= MOTOR_COUNT)
  {
    return;
  }

  new_mode = (mode == MOTOR_CONTROL_MODE_SPEED) ? MOTOR_CONTROL_MODE_SPEED : MOTOR_CONTROL_MODE_POSITION;

  if (new_mode != control_mode[motor_id])
  {
    PID_Reset(&speed_pid[motor_id]);
    PID_Reset(&position_pid[motor_id]);
  }

  control_mode[motor_id] = new_mode;
}

/* 设置指定电机的速度环目标速度。 */
void MotorControl_SetTargetSpeed(uint8_t motor_id, float setpoint_speed_rad_s)
{
  if (motor_id >= MOTOR_COUNT)
  {
    return;
  }

  target_speed_rad_s[motor_id] = setpoint_speed_rad_s;
}

/* 设置指定电机的位置环目标角度（自动归一化到 [0, 360)）。 */
void MotorControl_SetTargetAngle(uint8_t motor_id, float setpoint_angle_deg)
{
  if (motor_id >= MOTOR_COUNT)
  {
    return;
  }

  target_angle_deg[motor_id] = Control_NormalizeAngleDeg(setpoint_angle_deg);
}

/* 将指定电机的位置目标复位为当前角度，并复位其位置环内部状态。 */
void MotorControl_ResetAngle(uint8_t motor_id)
{
  if (motor_id >= MOTOR_COUNT)
  {
    return;
  }

  target_angle_deg[motor_id] = actual_angle_deg[motor_id];
  PID_Reset(&position_pid[motor_id]);
}

/* 启动指定电机的相对转圈：输出轴转动 turns 圈（正=顺时针）。
 * 内部用"移动胡萝卜"驱动位置环 + 编码器脉冲累计判定完成。
 * 仅在 feedback_valid[motor_id] 为真时生效。 */
void Control_StartTurns(uint8_t motor_id, float turns)
{
  if (motor_id >= MOTOR_COUNT)
  {
    return;
  }

  if ((fabsf(turns) < 0.001f) || (feedback_valid[motor_id] == 0U))
  {
    return;
  }

  /* 输出轴圈数 → 电机编码器脉冲数 */
  turns_target_ticks[motor_id] = fabsf(turns) * 8192.0f * fabsf(debug_gear_ratio);
  turns_accumulated_ticks[motor_id] = 0.0f;
  turns_last_encoder[motor_id] = encoder_raw[motor_id];
  turns_direction_sign[motor_id] = (turns >= 0.0f) ? 1.0f : -1.0f;

  /* 初始胡萝卜：挂在前方 30° */
  target_angle_deg[motor_id] = actual_angle_deg[motor_id]
                             + turns_direction_sign[motor_id] * TURNS_CARROT_DEG;
  PID_Reset(&position_pid[motor_id]);
  turns_active[motor_id] = 1U;
}

/* 清零全部 8 个 PID（4 速度环 + 4 位置环）的内部积分/历史误差状态。 */
void Control_ResetPidState(void)
{
  uint8_t i;
  for (i = 0; i < MOTOR_COUNT; i++)
  {
    PID_Reset(&speed_pid[i]);
    PID_Reset(&position_pid[i]);
  }
}
