#include "control.h"

#include <math.h>

#include "can.h"
#include "debug.h"
#include "pid.h"

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
/* 启动探测时切换下一个电机 ID 的间隔（ms）。 */
#define MOTOR_PROBE_SWITCH_MS     200U
/* 支持的最小电机 ID。 */
#define MOTOR_CAN_ID_MIN          1U
/* 支持的最大电机 ID。 */
#define MOTOR_CAN_ID_MAX          8U
/* 编码器一圈计数（0~8191 共 8192 点）。 */
#define MOTOR_ENCODER_CPR         8192.0f
/* 低速静摩擦补偿电流（iq），默认值在 debug.c 中定义。Ozone 可直接修改 debug_speed_stiction_comp_iq。 */

/* ========================= 可调参数区（外部可见） ========================= */
/* PID 参数存储已迁移到 debug.c 中的独立 volatile float 变量，
   方便在 Ozone watch 窗口中直接观察和修改。此处保留结构体实例用于
   兼容旧 API，但实际控制回路从 debug_xxx 变量读取。 */

/* ========================= 控制器内部状态区（私有） ========================= */
/* PID 控制器内部状态。 */
static PIDController speed_pid;
static PIDController position_pid;

/* 控制目标与模式状态。 */
static uint8_t control_mode = MOTOR_CONTROL_MODE_POSITION;
static float target_angle_deg = 0.0f;
static float target_speed_rad_s = 0.0f;
static float prev_target_speed_rad_s = 0.0f;     /* 上一周期目标速度，用于检测阶跃 */

/* 电机反馈状态。 */
static uint8_t feedback_valid = 0U;
static uint8_t active_motor_id = 1U;
static uint16_t encoder_raw = 0U;
static int16_t motor_rpm = 0;
static float actual_angle_deg = 0.0f;
static float actual_speed_rad_s = 0.0f;
static float speed_setpoint_rad_s = 0.0f;
static float position_output_rad_s = 0.0f;
static float torque_cmd_iq = 0.0f;

/* 无反馈时的启动探测状态。 */
static uint32_t no_feedback_elapsed_ms = 0U;
static uint8_t startup_probe_motor_id = 1U;
static uint32_t startup_probe_elapsed_ms = 0U;

/* 相对转圈状态："移动胡萝卜"驱动 + 编码器累计 */
static uint8_t  turns_active = 0U;
static float    turns_target_ticks = 0.0f;      /* 需累计的编码器脉冲数 */
static float    turns_accumulated_ticks = 0.0f;  /* 已累计脉冲数 */
static uint16_t turns_last_encoder = 0U;
static float    turns_direction_sign = 1.0f;     /* +1 顺时针, -1 逆时针 */
#define TURNS_CARROT_DEG  30.0f                  /* 胡萝卜挂在前方 30° */

float debug_target_angle_deg_watch = 0.0f;

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

/* 将可调 PID 参数同步到运行中的 PID 控制器实例。参数从 debug_xxx 独立变量读取。 */
static void Control_ApplyPidParams(void)
{
  speed_pid.kp = debug_speed_kp;
  speed_pid.ki = debug_speed_ki;
  speed_pid.kd = debug_speed_kd;
  speed_pid.derivative_alpha = debug_speed_derivative_alpha;
  speed_pid.output_limit = fabsf(debug_speed_output_limit);
  speed_pid.integral_limit = fabsf(debug_speed_integral_limit);
  speed_pid.integral_separation_threshold = fabsf(debug_speed_integral_separation);
  speed_pid.integral = PID_Clamp(speed_pid.integral, speed_pid.integral_limit);

  position_pid.kp = debug_position_kp;
  position_pid.ki = debug_position_ki;
  position_pid.kd = debug_position_kd;
  position_pid.derivative_alpha = debug_position_derivative_alpha;
  position_pid.output_limit = fabsf(debug_position_output_limit);
  position_pid.integral_limit = fabsf(debug_position_integral_limit);
  position_pid.integral_separation_threshold = fabsf(debug_position_integral_separation);
  position_pid.integral = PID_Clamp(position_pid.integral, position_pid.integral_limit);
}

/* 打包并发送 0x200/0x1FF 电流控制帧。 */
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

/* 按电机 ID 将单路电流命令映射到 C620 四路打包帧。 */
static HAL_StatusTypeDef Control_SendSingleCurrent(uint8_t target_motor_id, int16_t iq)
{
  int16_t iq1 = 0;
  int16_t iq2 = 0;
  int16_t iq3 = 0;
  int16_t iq4 = 0;

  if ((target_motor_id < MOTOR_CAN_ID_MIN) || (target_motor_id > MOTOR_CAN_ID_MAX))
  {
    return HAL_ERROR;
  }

  if (target_motor_id <= 4U)
  {
    if (target_motor_id == 1U) iq1 = iq;
    if (target_motor_id == 2U) iq2 = iq;
    if (target_motor_id == 3U) iq3 = iq;
    if (target_motor_id == 4U) iq4 = iq;
    return Control_SendCurrentFrame(CAN_MOTOR_TX_STDID_LOW, iq1, iq2, iq3, iq4);
  }

  if (target_motor_id == 5U) iq1 = iq;
  if (target_motor_id == 6U) iq2 = iq;
  if (target_motor_id == 7U) iq3 = iq;
  if (target_motor_id == 8U) iq4 = iq;
  return Control_SendCurrentFrame(CAN_MOTOR_TX_STDID_HIGH, iq1, iq2, iq3, iq4);
}

/* 初始化控制模块，默认位置环 0°。 */
void Control_Init(void)
{
  control_mode = MOTOR_CONTROL_MODE_POSITION;
  target_angle_deg = 0.0f;
  target_speed_rad_s = 0.0f;
  speed_setpoint_rad_s = 0.0f;
  position_output_rad_s = 0.0f;
  torque_cmd_iq = 0.0f;

  feedback_valid = 0U;
  active_motor_id = 1U;
  encoder_raw = 0U;
  motor_rpm = 0;
  actual_angle_deg = 0.0f;
  actual_speed_rad_s = 0.0f;
  no_feedback_elapsed_ms = 0U;
  startup_probe_motor_id = 1U;
  startup_probe_elapsed_ms = 0U;

  PID_Init(&speed_pid,
           debug_speed_kp,
           debug_speed_ki,
           debug_speed_kd,
           fabsf(debug_speed_output_limit),
           fabsf(debug_speed_integral_limit),
           fabsf(debug_speed_integral_separation));
  PID_Init(&position_pid,
           debug_position_kp,
           debug_position_ki,
           debug_position_kd,
           fabsf(debug_position_output_limit),
           fabsf(debug_position_integral_limit),
           fabsf(debug_position_integral_separation));
}

/* 消化一帧 CAN 反馈，更新转速/角度计算所需状态。 */
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

  rx_motor_id = (uint8_t)(rx_header->StdId - 0x200U);

  /* 锁定单电机时只接受指定 ID 的反馈，忽略其他电机消息。 */
  if (CONTROL_LOCK_MOTOR_ID_ENABLE)
  {
    if (rx_motor_id != CONTROL_LOCKED_MOTOR_ID)
    {
      return;
    }
  }

  active_motor_id = rx_motor_id;
  encoder_raw = (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
  actual_angle_deg = ((float)encoder_raw * 360.0f) / MOTOR_ENCODER_CPR;
  motor_rpm = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
  actual_speed_rad_s = (float)motor_rpm * RPM_TO_RADPS;
  feedback_valid = 1U;

  /* 同步诊断变量 */
  debug_feedback_valid = feedback_valid;
  debug_active_motor_id = active_motor_id;
  debug_actual_angle_deg = actual_angle_deg;
  debug_actual_speed_rad_s = actual_speed_rad_s;
}

/* 主控制周期：位置环(可选)->速度环->电流命令。 */
void Control_Task(float dt_s)
{
  uint32_t dt_ms;
  int16_t torque_cmd_int16;
  float startup_dir;
  float position_error_deg;

  if (dt_s <= 0.0f)
  {
    return;
  }

  debug_control_task_count++;
  debug_control_mode_active = control_mode;

  Control_ApplyPidParams();

  dt_ms = (uint32_t)(dt_s * 1000.0f + 0.5f);
  if (dt_ms == 0U)
  {
    dt_ms = 1U;
  }

  if (feedback_valid == 0U)
  {
    uint8_t probe_id;

    if (no_feedback_elapsed_ms <= (0xFFFFFFFFU - dt_ms))
    {
      no_feedback_elapsed_ms += dt_ms;
    }

    /* 确定探测方向：速度环跟随目标符号，位置环默认正方向。 */
    if (control_mode == MOTOR_CONTROL_MODE_POSITION)
    {
      startup_dir = 1.0f;
    }
    else
    {
      startup_dir = (target_speed_rad_s >= 0.0f) ? 1.0f : -1.0f;
    }

    /* 超时前持续探测，让电机获得启动电流。 */
    if ((no_feedback_elapsed_ms <= MOTOR_STARTUP_TIMEOUT_MS) &&
        (MOTOR_STARTUP_CURRENT > 0))
    {
      torque_cmd_iq = startup_dir * (float)MOTOR_STARTUP_CURRENT;
      torque_cmd_int16 = (int16_t)PID_Clamp(torque_cmd_iq, 10000.0f);

      /* 锁定单电机时只探测指定 ID，否则轮询 1~8。 */
      if (CONTROL_LOCK_MOTOR_ID_ENABLE)
      {
        probe_id = CONTROL_LOCKED_MOTOR_ID;
      }
      else
      {
        probe_id = startup_probe_motor_id;
      }

      if (Control_SendSingleCurrent(probe_id, torque_cmd_int16) == HAL_OK)
      {
        if (!CONTROL_LOCK_MOTOR_ID_ENABLE)
        {
          if (startup_probe_elapsed_ms <= (0xFFFFFFFFU - dt_ms))
          {
            startup_probe_elapsed_ms += dt_ms;
          }
          if (startup_probe_elapsed_ms >= MOTOR_PROBE_SWITCH_MS)
          {
            startup_probe_elapsed_ms = 0U;
            startup_probe_motor_id++;
            if (startup_probe_motor_id > MOTOR_CAN_ID_MAX)
            {
              startup_probe_motor_id = MOTOR_CAN_ID_MIN;
            }
          }
        }
      }
    }
    else
    {
      torque_cmd_iq = 0.0f;
      (void)Control_SendCurrentFrame(CAN_MOTOR_TX_STDID_LOW, 0, 0, 0, 0);
      (void)Control_SendCurrentFrame(CAN_MOTOR_TX_STDID_HIGH, 0, 0, 0, 0);
    }

    debug_torque_cmd = torque_cmd_iq;
    debug_feedback_valid = 0U;
    return;
  }

  no_feedback_elapsed_ms = 0U;
  startup_probe_elapsed_ms = 0U;
  startup_probe_motor_id = active_motor_id;

  if (control_mode == MOTOR_CONTROL_MODE_SPEED)
  {
    position_output_rad_s = 0.0f;
    speed_setpoint_rad_s = target_speed_rad_s;
    debug_position_error_deg = 0.0f;
    debug_position_output_rad_s = 0.0f;
  }
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
    if (turns_active)
    {
      int32_t raw_delta = (int32_t)encoder_raw - (int32_t)turns_last_encoder;
      if (raw_delta > 4096)       { raw_delta -= 8192; }
      else if (raw_delta < -4096) { raw_delta += 8192; }

      turns_accumulated_ticks += fabsf((float)raw_delta);
      turns_last_encoder = encoder_raw;

      if (turns_accumulated_ticks >= turns_target_ticks)
      {
        /* 完成：停在当前角度 */
        turns_active = 0U;
        target_angle_deg = actual_angle_deg;
        debug_set_position = actual_angle_deg;
        position_error_deg = 0.0f;
        position_pid.integral = 0.0f;
      }
      else
      {
        /* 胡萝卜挂前方 30°，ShortestArcError 自动选最短方向 */
        target_angle_deg = actual_angle_deg
                         + turns_direction_sign * TURNS_CARROT_DEG;
        debug_target_angle_deg_watch = target_angle_deg;
      }
    }

    /* 位置环过零处理：始终按最短角差（<=180°）计算误差。
     * 方向偏好：双向等距时优先减小角度（已在 ShortestArcErrorDeg 内置）。 */
    position_error_deg = Control_ShortestArcErrorDeg(
      Control_NormalizeAngleDeg(target_angle_deg),
      Control_NormalizeAngleDeg(actual_angle_deg));

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
      float abs_err = fabsf(position_error_deg);
      float db = debug_position_deadband_deg;

      if (abs_err <= db)
      {
        position_error_deg = 0.0f;
        position_pid.integral = 0.0f;
      }
      else
      {
        /* 保持方向，减去死区 */
        position_error_deg = (position_error_deg > 0.0f ? 1.0f : -1.0f)
                           * (abs_err - db);
      }
    }

    position_output_rad_s = PID_Calculate(&position_pid, position_error_deg, 0.0f, dt_s);
    speed_setpoint_rad_s = position_output_rad_s;

    /* 同步诊断变量 */
    debug_position_error_deg = position_error_deg;
    debug_position_output_rad_s = position_output_rad_s;
  }

  /* ====== 转圈中直接控速：绕过位置环输出限幅 ======
   *
   * 胡萝卜法里位置环的输出上限由 position_output_limit (4.0 rad/s) 钳死，
   * 无论 debug_turns_speed_rad_s 设多大都提不上去。
   *
   * 此处绕开位置环，将 speed_setpoint 直接覆写为期望转速，
   * 速度环 PID 跟随产出对应扭矩。编码器累计与完成判定仍在上方位置分支内处理。 */
  if (turns_active && (turns_accumulated_ticks < turns_target_ticks))
  {
    speed_setpoint_rad_s = turns_direction_sign * fabsf(debug_turns_speed_rad_s);
    position_output_rad_s = speed_setpoint_rad_s;
    debug_position_output_rad_s = speed_setpoint_rad_s;
  }

  /* ====== 目标速度阶跃检测：积分清零 ======
   *
   * 当速度环目标发生 >2 rad/s 的跳变时，清零速度环积分器。
   * 解决问题：
   *   1. 高→低跳变：旧稳态积分（正值）会对抗 P 项的刹车力，导致减速极慢
   *   2. 低→高跳变：旧稳态积分不适用于新工作点，保留反而引入扰动
   *
   * 位置环不受影响（级联结构中位置环输出平缓变化）。 */
  if (control_mode == MOTOR_CONTROL_MODE_SPEED)
  {
    if (fabsf(target_speed_rad_s - prev_target_speed_rad_s) > 2.0f)
    {
      speed_pid.integral = 0.0f;
    }
    prev_target_speed_rad_s = target_speed_rad_s;
  }

  /* 过零静止区：目标与实际都足够接近 0 时，清积分并保持 0 输出，抑制抖动。 */
  if ((fabsf(debug_speed_zero_cross_threshold) > 0.0f) &&
      (fabsf(speed_setpoint_rad_s) <= fabsf(debug_speed_zero_cross_threshold)) &&
      (fabsf(actual_speed_rad_s) <= fabsf(debug_speed_zero_cross_threshold)))
  {
    speed_pid.integral = 0.0f;
    speed_setpoint_rad_s = 0.0f;
  }

  torque_cmd_iq = PID_Calculate(&speed_pid, speed_setpoint_rad_s, actual_speed_rad_s, dt_s);
  torque_cmd_iq += speed_setpoint_rad_s * debug_speed_ff_gain;

  /* ====== 超速制动增强 ======
   *
   * 当实际速度显著高于目标速度时（超速 >0.5 rad/s），按超速量等比例
   * 叠加制动电流。解决纯 P 控制在大偏差下制动不足的问题：
   *   - Kp=80, error=20 → P 只产生 1600 iq 刹车（16% 能力）
   *   - 叠加 20×150=3000 iq → 总计 4600 iq 刹车（46% 能力）
   *
   * 稳态（超速 <0.5 rad/s）不触发，不影响正常调节精度。 */
  if ((control_mode == MOTOR_CONTROL_MODE_SPEED) &&
      (actual_speed_rad_s > speed_setpoint_rad_s + 0.5f))
  {
    float overspeed = actual_speed_rad_s - speed_setpoint_rad_s;
    torque_cmd_iq -= overspeed * 150.0f;
  }

  /* 过零制动：目标与实际方向相反时增加制动补偿，帮助平稳快速过零。 */
  if ((fabsf(debug_speed_zero_cross_threshold) > 0.0f) &&
      (fabsf(debug_speed_zero_cross_brake) > 0.0f) &&
      (speed_setpoint_rad_s * actual_speed_rad_s < 0.0f) &&
      (fabsf(speed_setpoint_rad_s) > fabsf(debug_speed_zero_cross_threshold)) &&
      (fabsf(actual_speed_rad_s) > fabsf(debug_speed_zero_cross_threshold)))
  {
    torque_cmd_iq += (actual_speed_rad_s > 0.0f)
                   ? -fabsf(debug_speed_zero_cross_brake)
                   : fabsf(debug_speed_zero_cross_brake);
  }

  /* ====== 静摩擦补偿（仅位置环） ======
   *
   * 仅位置环启用。速度环由用户直接控制目标速度，不需要自动补偿。
   *
   * 补偿量随位置误差增减：误差大→全量补偿（突破静摩擦），
   * 误差小→线性缩减至 0（微调时不引入冲击）。 */
  if ((control_mode == MOTOR_CONTROL_MODE_POSITION) &&
      (fabsf(speed_setpoint_rad_s) > 0.5f) && (fabsf(actual_speed_rad_s) < 0.2f))
  {
    float stiction_comp = debug_speed_stiction_comp_iq;
    float abs_pos_err = fabsf(position_error_deg);
    float thresh = fabsf(debug_position_stiction_threshold_deg);

    if (thresh > 0.0f && abs_pos_err < thresh)
    {
      /* 位置误差 < 阈值：补偿量线性缩减。
       * 例如 threshold=10°, 误差=2° → 补偿 = 500 × 0.2 = 100 iq
       *       误差=0°(死区内) → position_error_deg=0 → 补偿 = 0 */
      stiction_comp *= (abs_pos_err / thresh);
    }

    torque_cmd_iq += (speed_setpoint_rad_s > 0.0f) ? stiction_comp : -stiction_comp;
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
  if ((control_mode == MOTOR_CONTROL_MODE_SPEED) &&
      (fabsf(target_speed_rad_s) < 0.001f))
  {
    speed_pid.integral = 0.0f;
    torque_cmd_iq = 0.0f;
  }

  torque_cmd_int16 = (int16_t)PID_Clamp(torque_cmd_iq, 10000.0f);
  debug_torque_cmd = torque_cmd_iq;
  (void)Control_SendSingleCurrent(active_motor_id, torque_cmd_int16);
}

/* 写入速度环参数（下一控制周期生效）。 */
void Control_SetSpeedPidParams(const ControlSpeedPidParams *params)
{
  if (params == 0)
  {
    return;
  }

  debug_speed_kp = params->kp;
  debug_speed_ki = params->ki;
  debug_speed_kd = params->kd;
  debug_speed_output_limit = params->output_limit;
  debug_speed_integral_limit = params->integral_limit;
  debug_speed_integral_separation = params->integral_separation_threshold;
  debug_speed_ff_gain = params->ff_gain;
  debug_speed_zero_cross_threshold = params->zero_cross_speed_threshold;
  debug_speed_zero_cross_brake = params->zero_cross_brake_current;
}

/* 写入位置环参数（下一控制周期生效）。 */
void Control_SetPositionPidParams(const ControlPositionPidParams *params)
{
  if (params == 0)
  {
    return;
  }

  debug_position_kp = params->kp;
  debug_position_ki = params->ki;
  debug_position_kd = params->kd;
  debug_position_output_limit = params->output_limit;
  debug_position_integral_limit = params->integral_limit;
  debug_position_integral_separation = params->integral_separation_threshold;
}

/* 清零速度环与位置环内部状态。 */
void Control_ResetPidState(void)
{
  PID_Reset(&speed_pid);
  PID_Reset(&position_pid);
}

/* 启动相对转圈：输出轴转动 turns 圈（正=顺时针）。
 * 内部用"移动胡萝卜"驱动位置环 + 编码器脉冲累计判定完成。 */
void Control_StartTurns(float turns)
{
  if ((fabsf(turns) < 0.001f) || (feedback_valid == 0U))
  {
    return;
  }

  /* 输出轴圈数 → 电机编码器脉冲数 */
  turns_target_ticks = fabsf(turns) * 8192.0f * fabsf(debug_gear_ratio);
  turns_accumulated_ticks = 0.0f;
  turns_last_encoder = encoder_raw;
  turns_direction_sign = (turns >= 0.0f) ? 1.0f : -1.0f;

  /* 初始胡萝卜：挂在前方 30° */
  target_angle_deg = actual_angle_deg + turns_direction_sign * TURNS_CARROT_DEG;
  PID_Reset(&position_pid);
  turns_active = 1U;
}

/* 设置位置环目标角。 */
void MotorControl_SetTargetAngle(float setpoint_angle_deg)
{
  target_angle_deg = Control_NormalizeAngleDeg(setpoint_angle_deg);
}

/* 设置控制模式（速度环/位置环）。切换模式时自动清零 PID 状态，避免积分残留。 */
void MotorControl_SetControlMode(uint8_t mode)
{
  uint8_t new_mode = (mode == MOTOR_CONTROL_MODE_SPEED) ? MOTOR_CONTROL_MODE_SPEED : MOTOR_CONTROL_MODE_POSITION;
  if (new_mode != control_mode)
  {
    PID_Reset(&speed_pid);
    PID_Reset(&position_pid);
  }
  control_mode = new_mode;
}

/* 设置速度环目标速度。 */
void MotorControl_SetTargetSpeed(float setpoint_speed_rad_s)
{
  target_speed_rad_s = setpoint_speed_rad_s;
}

/* 将位置目标复位为当前角度，并复位位置环内部状态。 */
void MotorControl_ResetAngle(void)
{
  target_angle_deg = actual_angle_deg;
  PID_Reset(&position_pid);
}
