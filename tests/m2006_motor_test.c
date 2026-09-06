/**
  ******************************************************************************
  * @file    m2006_motor_test.c
  * @brief   m2006_motor 主机端单元测试（gcc 编译运行，不依赖嵌入式平台）
  *
  * 编译：gcc -Wall -Wextra -I Lib/m2006_lib/include -I Lib/pid_lib/include
  *       -o Lib/m2006_lib/tests/m2006_motor_test.exe
  *       Lib/m2006_lib/tests/m2006_motor_test.c Lib/m2006_lib/src/m2006_motor.c
  *       Lib/m2006_lib/src/m2006_protocol.c Lib/pid_lib/src/pid.c
  * 运行：./Lib/m2006_lib/tests/m2006_motor_test.exe
  *
  * 覆盖：默认状态、开环直通/钳位、断使能、反馈超时（tick 注入）、
  *       多圈累计角度、速度环方向与限幅、位置环死区、级联电流限幅、
  *       模式切换累加器预置、超速保护、非法模式。
  ******************************************************************************
  */
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include "m2006_motor.h"
#include "m2006_protocol.h"

/* ---- 简易测试框架 ---- */
static int g_test_count = 0;
static int g_test_fail = 0;

#define M2006_MOTOR_TEST_ASSERT(cond, msg)                                     \
  do {                                                          \
    g_test_count++;                                            \
    if (!(cond)) {                                             \
      fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); \
      g_test_fail++;                                           \
    }                                                           \
  } while (0)

#define M2006_MOTOR_TEST_ASSERT_FLOAT_NEAR(actual, expected, eps, msg)         \
  do {                                                          \
    g_test_count++;                                            \
    if (fabsf((float)(actual) - (float)(expected)) > (float)(eps)) { \
      fprintf(stderr, "FAIL: %s (line %d) actual=%f expected=%f\n", \
              msg, __LINE__, (double)(actual), (double)(expected)); \
      g_test_fail++;                                           \
    }                                                           \
  } while (0)

/* ---- 测试辅助 ---- */

static m2006_motor_t g_motor;

static void motor_init_ready(uint8_t esc_id)
{
  m2006_motor_init(&g_motor, esc_id);
  g_motor.is_enabled = 1U;
  g_motor.current_setpoint = 500;
}

static void feed_measure(uint16_t angle_raw, int16_t speed_rpm, uint32_t tick_ms)
{
  m2006_measure_t measure;

  measure.angle_raw = angle_raw;
  measure.speed_rpm = speed_rpm;
  measure.torque_raw = 0;
  m2006_motor_feed_feedback(&g_motor, &measure, tick_ms);
}

/* ---- 默认状态 ---- */

static void test_init_default_state(void)
{
  m2006_motor_t motor;

  m2006_motor_init(&motor, 2U);
  M2006_MOTOR_TEST_ASSERT(motor.esc_id == 2U, "esc_id stored");
  M2006_MOTOR_TEST_ASSERT(motor.is_enabled == 0U, "disabled by default");
  M2006_MOTOR_TEST_ASSERT(motor.current_limit == 10000, "current limit default");
  M2006_MOTOR_TEST_ASSERT(motor.speed_limit_rpm == 0, "speed limit off by default");
  M2006_MOTOR_TEST_ASSERT(motor.mode == M2006_MOTOR_MODE_OPEN_LOOP,
                          "open loop by default");
  M2006_MOTOR_TEST_ASSERT(motor.output_current == 0, "output zero after init");
  M2006_MOTOR_TEST_ASSERT(motor.rx_msg_count == 0U, "no feedback yet");
  M2006_MOTOR_TEST_ASSERT(motor.is_rx_timeout == 1U, "timeout before first rx");
}

/* ---- 开环模式 ---- */

static void test_open_loop_direct(void)
{
  motor_init_ready(2U);
  feed_measure(0U, 0, 0U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == 500,
                          "open loop direct output");
}

static void test_open_loop_clamped(void)
{
  motor_init_ready(2U);
  g_motor.current_setpoint = 12000;
  feed_measure(0U, 0, 0U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == 10000,
                          "open loop clamped to current limit");
}

static void test_open_loop_negative_clamped(void)
{
  motor_init_ready(2U);
  g_motor.current_setpoint = -12000;
  feed_measure(0U, 0, 0U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == -10000,
                          "open loop negative clamped");
}

/* ---- 安全门：断使能 / 超时 / 超速 ---- */

static void test_disabled_forces_zero(void)
{
  motor_init_ready(2U);
  g_motor.is_enabled = 0U;
  feed_measure(0U, 0, 0U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == 0,
                          "disabled output zero");
}

static void test_rx_timeout_breaks_output_and_recovers(void)
{
  motor_init_ready(2U);
  feed_measure(0U, 0, 0U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == 500,
                          "output before timeout");
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 26U) == 0,
                          "output zero after 20ms timeout");
  M2006_MOTOR_TEST_ASSERT(g_motor.is_rx_timeout == 1U, "timeout flag set");
  feed_measure(0U, 0, 30U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 35U) == 500,
                          "output recovers after new feedback");
  M2006_MOTOR_TEST_ASSERT(g_motor.is_rx_timeout == 0U, "timeout flag cleared");
}

static void test_speed_limit_breaks_output(void)
{
  motor_init_ready(2U);
  g_motor.speed_limit_rpm = 10;
  /* 转子 720 rpm ÷ 36 = 输出轴 20 rpm > 10 */
  feed_measure(0U, 720, 0U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == 0,
                          "overspeed output zero");
}

/* ---- 多圈累计角度 ---- */

static void test_accumulated_angle_forward(void)
{
  motor_init_ready(2U);
  feed_measure(0U, 0, 0U);
  (void)m2006_motor_update(&g_motor, 5U);   /* 首帧：记录基准，不累计 */
  feed_measure(100U, 0, 10U);
  (void)m2006_motor_update(&g_motor, 15U);
  M2006_MOTOR_TEST_ASSERT_FLOAT_NEAR(
      g_motor.angle_total_deg,
      100.0f * M2006_PROTOCOL_ANGLE_SCALE_DEG,
      1e-3f, "accumulated angle forward");
  M2006_MOTOR_TEST_ASSERT(g_motor.pos_feedback_deg == g_motor.angle_total_deg,
                          "position feedback mirrors accumulated angle");
}

static void test_accumulated_angle_wrap_continuous(void)
{
  motor_init_ready(2U);
  feed_measure(8000U, 0, 0U);
  (void)m2006_motor_update(&g_motor, 5U);
  feed_measure(100U, 0, 10U);
  (void)m2006_motor_update(&g_motor, 15U);
  M2006_MOTOR_TEST_ASSERT_FLOAT_NEAR(
      g_motor.angle_total_deg,
      292.0f * M2006_PROTOCOL_ANGLE_SCALE_DEG,
      1e-3f, "wrap forward stays continuous");
}

/* ---- 速度环 ---- */

static void test_speed_loop_direction_and_magnitude(void)
{
  motor_init_ready(2U);
  g_motor.mode = M2006_MOTOR_MODE_SPEED;
  g_motor.speed_setpoint_rpm = 100.0f;
  feed_measure(0U, 0, 0U);
  /* 增量式 PI 首拍：kp*e = 30*100 = 3000，ki*e*dt = 0.5，累加器从 0 起步 */
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == 3000,
                          "speed loop first sample output");
  M2006_MOTOR_TEST_ASSERT(g_motor.speed_cmd_rpm == 100.0f,
                          "speed command passes through");
}

static void test_speed_loop_clamped_by_current_limit(void)
{
  motor_init_ready(2U);
  g_motor.mode = M2006_MOTOR_MODE_SPEED;
  g_motor.speed_setpoint_rpm = 400.0f;
  g_motor.current_limit = 3000;
  feed_measure(0U, 0, 0U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == 3000,
                          "speed loop clamped to current limit");
}

/* ---- 位置环 ---- */

static void test_position_loop_deadband(void)
{
  motor_init_ready(2U);
  g_motor.mode = M2006_MOTOR_MODE_POSITION;
  g_motor.pos_setpoint_deg = 0.2f;   /* |误差| < 死区 0.5 */
  feed_measure(0U, 0, 0U);
  (void)m2006_motor_update(&g_motor, 5U);
  M2006_MOTOR_TEST_ASSERT(g_motor.pos_in_deadband == 1U, "inside deadband");
  M2006_MOTOR_TEST_ASSERT(g_motor.speed_cmd_rpm == 0.0f, "speed cmd zero in deadband");
  M2006_MOTOR_TEST_ASSERT(g_motor.output_current == 0, "output zero in deadband");
}

static void test_position_loop_cascade(void)
{
  motor_init_ready(2U);
  g_motor.mode = M2006_MOTOR_MODE_POSITION;
  g_motor.pos_setpoint_deg = 10.0f;
  feed_measure(0U, 0, 0U);
  (void)m2006_motor_update(&g_motor, 5U);
  /* 位置环 kp=1 → speed_cmd = 10 rpm；速度环 kp=30 → 电流 ≈ 300 */
  M2006_MOTOR_TEST_ASSERT(g_motor.speed_cmd_rpm == 10.0f,
                          "position loop output is speed command");
  M2006_MOTOR_TEST_ASSERT(g_motor.output_current == 300,
                          "cascade current = 30 * speed command");
}

/* ---- 模式切换累加器预置 ---- */

static void test_mode_switch_presets_accumulator(void)
{
  motor_init_ready(2U);
  g_motor.current_setpoint = 3000;
  feed_measure(0U, 0, 0U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == 3000,
                          "open loop at 3000");
  g_motor.mode = M2006_MOTOR_MODE_SPEED;
  g_motor.speed_setpoint_rpm = 100.0f;
  /* 累加器预置为 3000，再加首拍增量 3000.5 → 6000 */
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 10U) == 6000,
                          "switch presets accumulator to avoid jump");
}

/* ---- 非法模式 ---- */

static void test_invalid_mode_forces_zero(void)
{
  motor_init_ready(2U);
  g_motor.mode = (m2006_motor_mode_t)99;
  feed_measure(0U, 0, 0U);
  M2006_MOTOR_TEST_ASSERT(m2006_motor_update(&g_motor, 5U) == 0,
                          "invalid mode output zero");
}

/* ---- 反馈喂入 ---- */

static void test_feed_feedback_updates_raw_state(void)
{
  m2006_motor_t motor;
  m2006_measure_t measure;

  m2006_motor_init(&motor, 2U);
  measure.angle_raw = 0x1234U;
  measure.speed_rpm = -100;
  measure.torque_raw = 1000;
  m2006_motor_feed_feedback(&motor, &measure, 77U);
  M2006_MOTOR_TEST_ASSERT(motor.angle_raw == 0x1234U, "raw angle stored");
  M2006_MOTOR_TEST_ASSERT(motor.speed_rpm == -100, "raw speed stored");
  M2006_MOTOR_TEST_ASSERT(motor.torque_raw == 1000, "raw torque stored");
  M2006_MOTOR_TEST_ASSERT(motor.rx_msg_count == 1U, "rx count incremented");
  M2006_MOTOR_TEST_ASSERT(motor.last_rx_tick == 77U, "last rx tick stored");
}

int main(void)
{
  test_init_default_state();
  test_open_loop_direct();
  test_open_loop_clamped();
  test_open_loop_negative_clamped();
  test_disabled_forces_zero();
  test_rx_timeout_breaks_output_and_recovers();
  test_speed_limit_breaks_output();
  test_accumulated_angle_forward();
  test_accumulated_angle_wrap_continuous();
  test_speed_loop_direction_and_magnitude();
  test_speed_loop_clamped_by_current_limit();
  test_position_loop_deadband();
  test_position_loop_cascade();
  test_mode_switch_presets_accumulator();
  test_invalid_mode_forces_zero();
  test_feed_feedback_updates_raw_state();

  if (g_test_fail != 0)
  {
    (void)fprintf(stderr, "m2006_motor_test: %d/%d FAILED\n",
                  g_test_fail, g_test_count);
    return 1;
  }

  (void)printf("m2006_motor_test: PASS (%d asserts)\n", g_test_count);
  return 0;
}
