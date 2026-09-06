/**
  ******************************************************************************
  * @file    m2006_motor.c
  * @brief   M2006 电机实例实现：反馈换算、多圈累计角度、级联闭环、安全门
  *
  * 控制流程（由调用方 1kHz 周期调用 m2006_motor_update()）：
  *   超时判定 → 物理量换算 → 模式分支（OPEN_LOOP / SPEED / POSITION）
  *   → 级联 PID → 电流钳位 → 安全门（断使能 / 超时 / 超速）→ 输出电流
  *
  * 时钟来源为调用方传入的 tick_ms，本模块不依赖 RTOS；
  * 依赖 Lib/pid_lib（pid_t / pid_inc_t）。
  ******************************************************************************
  */
#include "m2006_motor.h"

#include "m2006_protocol.h"

/* 控制周期（秒），与 1kHz 控制任务一致 */
#define M2006_MOTOR_DT_SEC (0.001f)

/* 反馈超时判定时间（毫秒，按调用方时钟） */
#define M2006_MOTOR_RX_TIMEOUT_MS (20U)

/* 安全保护默认值 */
#define M2006_MOTOR_CURRENT_LIMIT_DEFAULT (10000)  /* 默认电流钳位 10A（电调满量程；调试放开，带负载/上线前应收回到 3A 额定） */
#define M2006_MOTOR_SPEED_LIMIT_DEFAULT_RPM (0)    /* 默认超速保护阈值：0 = 关闭保护（调试期默认），>0 时生效 */

/* 输出轴力矩换算：torque_out_nm = torque_raw × 0.18 / 1000
   （C610 反馈的"实际输出转矩"实为电流环反馈电流，1000 LSB = 1A；
    转矩常数 0.18 N·m/A 为 M2006 输出轴等效值，已含 36:1 减速比与传动效率） */
#define M2006_MOTOR_TORQUE_SCALE_NM \
    (M2006_PROTOCOL_TORQUE_CONSTANT_NM_PER_A / 1000.0f)

/* ---- 控制模式默认值（与原单电机调试面板初值一致） ---- */
#define M2006_MOTOR_POS_PID_KP_DEFAULT (1.0f)
#define M2006_MOTOR_POS_DEADBAND_DEFAULT (0.5f)
#define M2006_MOTOR_POS_MAX_SPEED_DEFAULT (0.0f)
#define M2006_MOTOR_SPD_PID_KP_DEFAULT (30.0f)
#define M2006_MOTOR_SPD_PID_KI_DEFAULT (5.0f)
#define M2006_MOTOR_SPD_SETPOINT_RATE_DEFAULT (0.0f)

void m2006_motor_init(m2006_motor_t *motor, uint8_t esc_id)
{
  if (motor == 0)
  {
    return;
  }

  /* ---- 配置区默认值 ---- */
  motor->esc_id = esc_id;
  motor->is_enabled = 0U;
  motor->current_limit = M2006_MOTOR_CURRENT_LIMIT_DEFAULT;
  motor->speed_limit_rpm = M2006_MOTOR_SPEED_LIMIT_DEFAULT_RPM;
  motor->mode = M2006_MOTOR_MODE_OPEN_LOOP;
  motor->pos_setpoint_deg = 0.0f;
  motor->speed_setpoint_rpm = 0.0f;
  motor->pos_pid_kp = M2006_MOTOR_POS_PID_KP_DEFAULT;
  motor->pos_deadband_deg = M2006_MOTOR_POS_DEADBAND_DEFAULT;
  motor->pos_max_speed_rpm = M2006_MOTOR_POS_MAX_SPEED_DEFAULT;
  motor->spd_pid_kp = M2006_MOTOR_SPD_PID_KP_DEFAULT;
  motor->spd_pid_ki = M2006_MOTOR_SPD_PID_KI_DEFAULT;
  motor->spd_setpoint_rate = M2006_MOTOR_SPD_SETPOINT_RATE_DEFAULT;
  motor->current_setpoint = 0;

  /* ---- 观测区清零 ---- */
  motor->angle_raw_deg = 0.0f;
  motor->angle_total_deg = 0.0f;
  motor->speed_out_rpm = 0.0f;
  motor->torque_out_nm = 0.0f;
  motor->pos_feedback_deg = 0.0f;
  motor->speed_feedback_rpm = 0.0f;
  motor->speed_cmd_rpm = 0.0f;
  motor->current_cmd_raw = 0;
  motor->output_current = 0;
  motor->rx_msg_count = 0U;
  motor->is_rx_timeout = 1U;
  motor->pos_in_deadband = 0U;

  /* ---- 内部状态清零 ---- */
  motor->angle_raw = 0U;
  motor->speed_rpm = 0;
  motor->torque_raw = 0;
  motor->prev_mode = M2006_MOTOR_MODE_OPEN_LOOP;
  motor->last_rx_tick = 0U;
  motor->prev_angle_raw = 0U;
  motor->accumulated_lsb = 0;
  motor->angle_valid = 0U;

  /* ---- PID 实例初始化 ---- */

  /* 位置环：位置式，纯 P + 死区，输出限幅 = 位置模式最大速度 */
  motor->pos_pid.kp = motor->pos_pid_kp;
  motor->pos_pid.ki = 0.0f;
  motor->pos_pid.kd = 0.0f;
  motor->pos_pid.dt = M2006_MOTOR_DT_SEC;
  motor->pos_pid.out_min = -motor->pos_max_speed_rpm;
  motor->pos_pid.out_max = motor->pos_max_speed_rpm;
  motor->pos_pid.integral_min = 0.0f;
  motor->pos_pid.integral_max = 0.0f;
  motor->pos_pid.setpoint_rate = 0.0f;
  motor->pos_pid.deadband = motor->pos_deadband_deg;
  motor->pos_pid.hysteresis = 0.0f;
  motor->pos_pid.d_filter_alpha = 1.0f;
  motor->pos_pid.integral_hold_error = 0.0f;
  (void)pid_init(&motor->pos_pid);

  /* 速度环：增量式，PI，输出限幅 = 电流钳位（单一来源 current_limit） */
  motor->spd_pid.kp = motor->spd_pid_kp;
  motor->spd_pid.ki = motor->spd_pid_ki;
  motor->spd_pid.kd = 0.0f;
  motor->spd_pid.dt = M2006_MOTOR_DT_SEC;
  motor->spd_pid.out_min = -(float)motor->current_limit;
  motor->spd_pid.out_max = (float)motor->current_limit;
  motor->spd_pid.setpoint_rate = motor->spd_setpoint_rate;
  motor->spd_pid.deadband = 0.0f;
  motor->spd_pid.hysteresis = 0.0f;
  motor->spd_pid.d_filter_alpha = 1.0f;
  (void)pid_inc_init(&motor->spd_pid);
}

void m2006_motor_feed_feedback(m2006_motor_t *motor,
                               const m2006_measure_t *measure,
                               uint32_t tick_ms)
{
  if ((motor == 0) || (measure == 0))
  {
    return;
  }

  motor->angle_raw = measure->angle_raw;
  motor->speed_rpm = measure->speed_rpm;
  motor->torque_raw = measure->torque_raw;
  motor->rx_msg_count++;
  motor->last_rx_tick = tick_ms;
}

int16_t m2006_motor_update(m2006_motor_t *motor, uint32_t tick_ms)
{
  float current_cmd;
  float current_limit_f;
  int16_t speed_limit_rpm;
  int16_t output_current;

  if (motor == 0)
  {
    return 0;
  }

  /* 1. 反馈超时判定：超过 M2006_MOTOR_RX_TIMEOUT_MS 未收到反馈则断输出 */
  if ((tick_ms - motor->last_rx_tick) > M2006_MOTOR_RX_TIMEOUT_MS)
  {
    motor->is_rx_timeout = 1U;
  }
  else
  {
    motor->is_rx_timeout = 0U;
  }

  /* 2. 物理量换算（单圈相位角 / 多圈累计角 / 输出轴转速 / 力矩） */
  motor->angle_raw_deg = (float)motor->angle_raw
                         * M2006_PROTOCOL_ANGLE_RAW_SCALE_DEG;

  /* 多圈累计角度：仅在收到反馈后开始（rx_msg_count > 0），避免首帧前误累计 */
  if (motor->rx_msg_count > 0U)
  {
    if (!motor->angle_valid)
    {
      motor->prev_angle_raw = motor->angle_raw;
      motor->angle_valid = 1U;
    }
    motor->accumulated_lsb += m2006_protocol_unwrap_angle(
        motor->prev_angle_raw, motor->angle_raw);
    motor->prev_angle_raw = motor->angle_raw;
  }
  motor->angle_total_deg = (float)motor->accumulated_lsb
                           * M2006_PROTOCOL_ANGLE_SCALE_DEG;

  motor->speed_out_rpm = (float)motor->speed_rpm
                         / (float)M2006_PROTOCOL_GEAR_RATIO;
  motor->torque_out_nm = (float)motor->torque_raw
                         * M2006_MOTOR_TORQUE_SCALE_NM;

  /* 3. 反馈映射到控制层观测（本实例内换算，无跨模块复制） */
  motor->pos_feedback_deg = motor->angle_total_deg;
  motor->speed_feedback_rpm = motor->speed_out_rpm;

  /* 4. 运行时参数同步：Keil Watch 修改配置区字段后立即生效 */
  motor->pos_pid.kp = motor->pos_pid_kp;
  motor->pos_pid.deadband = motor->pos_deadband_deg;
  motor->pos_pid.out_min = -motor->pos_max_speed_rpm;
  motor->pos_pid.out_max = motor->pos_max_speed_rpm;

  current_limit_f = (float)motor->current_limit;
  motor->spd_pid.kp = motor->spd_pid_kp;
  motor->spd_pid.ki = motor->spd_pid_ki;
  motor->spd_pid.setpoint_rate = motor->spd_setpoint_rate;
  motor->spd_pid.out_min = -current_limit_f;
  motor->spd_pid.out_max = current_limit_f;

  /* 5. 模式切换：复位两个 PID，并把速度环累加器预置为当前输出电流，
     使切换瞬间电流不突变（如从开环 3000 切闭环，累加器从 3000 起步） */
  if (motor->mode != motor->prev_mode)
  {
    pid_reset(&motor->pos_pid);
    pid_inc_reset(&motor->spd_pid);
    motor->spd_pid.output = (float)motor->output_current;
    motor->prev_mode = motor->mode;
  }

  /* 6. 模式分支级联闭环 */
  switch (motor->mode)
  {
    case M2006_MOTOR_MODE_OPEN_LOOP:
      /* 电流开环：目标电流直通（current_setpoint），钳位仍生效 */
      motor->speed_cmd_rpm = 0.0f;
      motor->pos_in_deadband = 0U;
      current_cmd = (float)motor->current_setpoint;
      break;

    case M2006_MOTOR_MODE_SPEED:
      /* 速度闭环：位置环旁路，速度设定直通速度环 */
      motor->speed_cmd_rpm = motor->speed_setpoint_rpm;
      motor->pos_in_deadband = 0U;
      current_cmd = pid_inc_update(&motor->spd_pid,
                                   motor->speed_setpoint_rpm,
                                   motor->speed_feedback_rpm);
      break;

    case M2006_MOTOR_MODE_POSITION:
      /* 位置闭环：位置环输出作为速度环设定，级联计算 */
      motor->speed_cmd_rpm = pid_update(&motor->pos_pid,
                                        motor->pos_setpoint_deg,
                                        motor->pos_feedback_deg);
      motor->pos_in_deadband = (uint8_t)motor->pos_pid.in_deadband;
      current_cmd = pid_inc_update(&motor->spd_pid,
                                   motor->speed_cmd_rpm,
                                   motor->speed_feedback_rpm);
      break;

    default:
      motor->speed_cmd_rpm = 0.0f;
      motor->pos_in_deadband = 0U;
      current_cmd = 0.0f;
      break;
  }

  /* 7. 电流钳位（所有模式统一，含 OPEN_LOOP 直通值） */
  if (current_cmd > current_limit_f)
  {
    current_cmd = current_limit_f;
  }
  else if (current_cmd < -current_limit_f)
  {
    current_cmd = -current_limit_f;
  }
  motor->current_cmd_raw = (int16_t)current_cmd;

  /* 8. 安全门：断使能 / 反馈超时 / 超速任一成立则输出 0 */
  output_current = (int16_t)current_cmd;
  if ((!motor->is_enabled) || (motor->is_rx_timeout != 0U))
  {
    output_current = 0;
  }
  else
  {
    speed_limit_rpm = motor->speed_limit_rpm;
    if ((speed_limit_rpm > 0)
        && ((motor->speed_out_rpm > (float)speed_limit_rpm)
            || (motor->speed_out_rpm < -(float)speed_limit_rpm)))
    {
      output_current = 0;
    }
  }

  motor->output_current = output_current;
  return output_current;
}
