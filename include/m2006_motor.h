/**
  ******************************************************************************
  * @file    m2006_motor.h
  * @brief   M2006 电机实例：一个电机的一切状态与计算（零 HAL、零 RTOS 依赖）
  *
  * 本模块合并了单电机实现中"驱动层"（反馈换算、多圈累计角度、安全门）与
  * "控制层"（位置环 + 速度环级联、三模式）的职责，形成可多实例化的电机单元：
  * - 每个 m2006_motor_t 实例对应一台物理电机，字段完全独立；
  * - 时钟由调用方传入（tick_ms），因此不依赖 RTOS，超时判定可在主机端测试；
  * - 依赖 Lib/pid_lib（pid_t / pid_inc_t），不重复实现 PID。
  *
  * 数据流：
  *   HAL RX 回调 → m2006_motor_feed_feedback()（中断安全，仅写状态）
  *   1kHz 任务  → m2006_motor_update()（换算/闭环/安全门，返回最终电流 raw）
  *
  * 结构体字段公开（Keil Watch 可直接查看/修改），
  * 配置区可写、观测区只读、内部区勿直接改。
  ******************************************************************************
  */
#ifndef M2006_MOTOR_H
#define M2006_MOTOR_H

#include <stdint.h>

#include "m2006_protocol.h"
#include "pid.h"

/* 控制模式 */
typedef enum m2006_motor_mode
{
  M2006_MOTOR_MODE_OPEN_LOOP = 0,  /* 电流开环：目标电流取 current_setpoint */
  M2006_MOTOR_MODE_SPEED,          /* 速度闭环：速度设定 → 速度环（增量式 PI）→ 电流 */
  M2006_MOTOR_MODE_POSITION,       /* 位置闭环：位置环（位置式 P）→ 速度环 → 电流 */
} m2006_motor_mode_t;

/**
  * @brief  M2006 电机实例（透明结构体，可静态分配数组）
  * @note   配置区：Keil Watch 可直接修改，运行时立即生效；
  *         观测区：只读，由 update/feed 更新；
  *         内部区：状态机内部使用，外部不应直接改。
  */
typedef struct m2006_motor
{
  /* ==== 配置区（可写） ==== */

  /* 电调 ID，范围 1~8（配合 C610，0x200 控制 ID 1~4，0x1FF 控制 ID 5~8） */
  uint8_t esc_id;

  /* 输出使能：0 断输出（电流恒为 0），1 使能控制 */
  uint8_t is_enabled;

  /* 电流钳位限幅，输出电流绝对值不超过该值（默认 10000 = 10A 调试放开；
     带负载/上线前应收回 3A 额定，即 3000） */
  int16_t current_limit;

  /* 输出轴转速限幅 rpm，超速时输出置 0（默认 0 = 关闭超速保护） */
  int16_t speed_limit_rpm;

  /* 控制模式：OPEN_LOOP / SPEED / POSITION，切换时内部复位 PID 并预置累加器 */
  m2006_motor_mode_t mode;

  /* 位置设定：输出轴累计角度（度），支持多圈连续角度（如 540.0） */
  float pos_setpoint_deg;

  /* 速度设定：输出轴转速（rpm），SPEED 模式使用 */
  float speed_setpoint_rpm;

  /* 位置环比例增益（默认 1.0），输出单位 rpm/° */
  float pos_pid_kp;

  /* 位置环死区宽度（度，默认 0.5）：|误差| 进入死区后误差置零，到位判停 */
  float pos_deadband_deg;

  /* 位置环输出限幅 = 位置模式最大速度（rpm，默认 0 = 不限幅） */
  float pos_max_speed_rpm;

  /* 速度环比例增益（默认 30.0） */
  float spd_pid_kp;

  /* 速度环积分增益（默认 5.0） */
  float spd_pid_ki;

  /* 速度设定最大变化率（rpm/s，默认 0 = 禁用斜坡） */
  float spd_setpoint_rate;

  /* 目标电流（驱动输入，OPEN_LOOP 模式直通值），范围 -10000~+10000 */
  int16_t current_setpoint;

  /* ==== 观测区（只读） ==== */

  /* 转子单圈相位角（度，0~360° 随编码器回绕） */
  float angle_raw_deg;

  /* 输出轴累计角度（度，多圈连续不回绕） */
  float angle_total_deg;

  /* 输出轴转速（rpm，= 转子 rpm ÷ 36） */
  float speed_out_rpm;

  /* 输出轴力矩（N·m，= 电流 A × 0.18，M2006 官方转矩常数输出轴等效值） */
  float torque_out_nm;

  /* 实际位置：输出轴累计角度（度，= angle_total_deg） */
  float pos_feedback_deg;

  /* 实际速度：输出轴转速（rpm，= speed_out_rpm） */
  float speed_feedback_rpm;

  /* 位置环输出（速度环设定值），POSITION 模式有效 */
  float speed_cmd_rpm;

  /* 速度环输出（电流指令 raw，钳位后），镜像到 output_current 前值 */
  int16_t current_cmd_raw;

  /* 驱动实际下发的电流值（钳位/安全门后的结果） */
  int16_t output_current;

  /* 反馈超时标志：1 表示最近 M2006_MOTOR_RX_TIMEOUT_MS 内未收到反馈 */
  uint8_t is_rx_timeout;

  /* 位置误差是否在死区内（POSITION 模式有效） */
  uint8_t pos_in_deadband;

  /* ==== 内部状态（勿直接改） ==== */

  /* 电调回传原始值（未解析换算，最近一帧）。
     volatile：由 HAL RX 中断（feed_feedback）写入、任务（update）读取，
     防止编译器优化缓存旧值 */
  volatile uint16_t angle_raw;
  volatile int16_t speed_rpm;
  volatile int16_t torque_raw;

  /* 已接收电调反馈帧计数（中断递增，任务读取） */
  volatile uint32_t rx_msg_count;

  /* 最近一次收到反馈的时刻（调用方时钟，中断写入，任务读取） */
  volatile uint32_t last_rx_tick;

  /* 位置环实例（位置式 P，POSITION 模式使用） */
  pid_t pos_pid;

  /* 速度环实例（增量式 PI，SPEED/POSITION 模式使用） */
  pid_inc_t spd_pid;

  /* 上一周期模式：用于切换检测（切换时复位 PID 并预置累加器） */
  m2006_motor_mode_t prev_mode;

  /* 多圈累计角度状态：上一周期转子编码、累计 LSB、是否已首帧 */
  uint16_t prev_angle_raw;
  int32_t accumulated_lsb;
  uint8_t angle_valid;
} m2006_motor_t;

/**
  * @brief  初始化电机实例：写入电调 ID、设置 PID 与安全门默认配置、清空状态
  * @param  motor  电机实例指针
  * @param  esc_id 电调 ID，范围 1~8（由调用方保证；attach 时还会校验）
  */
void m2006_motor_init(m2006_motor_t *motor, uint8_t esc_id);

/**
  * @brief  喂入电调反馈（HAL RX 回调调用，中断安全：仅写状态字段）
  * @param  motor   电机实例指针
  * @param  measure 解析后的反馈测量值
  * @param  tick_ms 当前时钟（毫秒，单调递增），用于反馈超时判定
  */
void m2006_motor_feed_feedback(m2006_motor_t *motor,
                               const m2006_measure_t *measure,
                               uint32_t tick_ms);

/**
  * @brief  周期更新（建议 1kHz 调用）：超时判定 → 换算 → 模式分支级联闭环
  *         → 钳位 → 安全门，返回最终电流 raw
  * @param  motor   电机实例指针
  * @param  tick_ms 当前时钟（毫秒，单调递增），用于反馈超时判定
  * @retval 最终输出电流 raw（已钳位 + 安全门，-current_limit ~ +current_limit）
  */
int16_t m2006_motor_update(m2006_motor_t *motor, uint32_t tick_ms);

#endif /* M2006_MOTOR_H */
