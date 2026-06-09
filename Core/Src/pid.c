#include "pid.h"

#include <math.h>

static float pid_clamp_range(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  return value;
}

/* 对称限幅：将 value 限制到 [-limit, limit]。 */
float PID_Clamp(float value, float limit)
{
  if (limit <= 0.0f)
  {
    return value;
  }
  return pid_clamp_range(value, -limit, limit);
}

/* 初始化 PID 参数与内部状态。 */
void PID_Init(PIDController *pid,
              float kp,
              float ki,
              float kd,
              float output_limit,
              float integral_limit,
              float integral_separation_threshold)
{
  if (pid == 0)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->prev_derivative = 0.0f;
  pid->derivative_alpha = 0.15f;   /* 默认 2.4 Hz 截止 @ 10ms 周期 */
  pid->output_limit = output_limit;
  pid->integral_limit = integral_limit;
  pid->integral_separation_threshold = fabsf(integral_separation_threshold);
}

/* 清零 PID 的积分与历史误差。 */
void PID_Reset(PIDController *pid)
{
  if (pid == 0)
  {
    return;
  }

  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->prev_derivative = 0.0f;
}

/* 计算离散 PID 输出（含积分分离、积分限幅、反计算抗饱和）。 */
float PID_Calculate(PIDController *pid, float setpoint, float measurement, float dt_s)
{
  float error;
  float derivative;
  float output;
  float output_clamped;

  if ((pid == 0) || (dt_s <= 0.0f))
  {
    return 0.0f;
  }

  error = setpoint - measurement;

  /* 积分分离：误差过大时暂停积分，避免大幅扰动引起积分累积。 */
  if ((pid->integral_separation_threshold <= 0.0f) ||
      (fabsf(error) <= pid->integral_separation_threshold))
  {
    pid->integral += error * dt_s;
  }
  pid->integral = PID_Clamp(pid->integral, pid->integral_limit);

  /* ====== D 项低通滤波：抑制高频噪声与间歇振荡 ======
   *
   * 问题：原始差分 derivative_raw = Δerror / dt 对高频噪声极其敏感。
   *       例如 CAN 反馈抖动 0.5 rad/s → Δerror=0.5 → ÷0.01s = 50，
   *       Kd×50 = 12.5 iq 噪声直接注入输出端，激发间歇性极限环振荡。
   *
   * 方案：一阶 IIR 低通滤波器。
   *       alpha = 0.15 → 截止频率 ≈ 0.15/(2π·0.01) ≈ 2.4 Hz
   *       电机机械带宽通常 < 5 Hz，2.4 Hz 截止保留真实动态、滤除噪声。 */
  {
    float raw_derivative = (error - pid->prev_error) / dt_s;
    pid->prev_derivative = pid->derivative_alpha * raw_derivative
                         + (1.0f - pid->derivative_alpha) * pid->prev_derivative;
    derivative = pid->prev_derivative;
  }

  output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
  output_clamped = PID_Clamp(output, pid->output_limit);

  /* 反计算抗积分饱和：输出被限幅时，反算积分值使其恰好达到限幅边界，
     防止积分在饱和期间持续累积导致过冲。 */
  if ((fabsf(output - output_clamped) > 1e-6f) && (pid->ki > 1e-6f))
  {
    pid->integral = (output_clamped - pid->kp * error - pid->kd * derivative) / pid->ki;
    pid->integral = PID_Clamp(pid->integral, pid->integral_limit);
  }

  pid->prev_error = error;
  return output_clamped;
}
