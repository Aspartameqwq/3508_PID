#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float prev_derivative;          /* D 项一阶低通滤波状态 */
  float derivative_alpha;         /* D 项滤波系数 (0~1, 越小滤波越强, 典型 0.10~0.25) */
  float output_limit;
  float integral_limit;
  float integral_separation_threshold;
} PIDController;

void PID_Init(PIDController *pid,
              float kp,
              float ki,
              float kd,
              float output_limit,
              float integral_limit,
              float integral_separation_threshold);
void PID_Reset(PIDController *pid);
float PID_Calculate(PIDController *pid, float setpoint, float measurement, float dt_s);
float PID_Clamp(float value, float limit);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
