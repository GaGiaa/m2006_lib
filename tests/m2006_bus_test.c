/**
  ******************************************************************************
  * @file    m2006_bus_test.c
  * @brief   m2006_bus 主机端单元测试（gcc 编译运行，不依赖嵌入式平台）
  *
  * 编译：gcc -Wall -Wextra -I Lib/m2006_lib/include -I Lib/pid_lib/include
  *       -o Lib/m2006_lib/tests/m2006_bus_test.exe
  *       Lib/m2006_lib/tests/m2006_bus_test.c Lib/m2006_lib/src/m2006_bus.c
  *       Lib/m2006_lib/src/m2006_motor.c Lib/m2006_lib/src/m2006_protocol.c
  *       Lib/pid_lib/src/pid.c
  * 运行：./Lib/m2006_lib/tests/m2006_bus_test.exe
  *
  * 覆盖：电机注册（成功/越界/占用/空指针）、控制帧聚合打包（0x200/0x1FF
  *       两组帧、无电机、单段、双段）、反馈分发（正确 ID/越界 ID/未注册槽位）。
  ******************************************************************************
  */
#include <stdio.h>
#include <stdint.h>

#include "m2006_bus.h"
#include "m2006_motor.h"

/* ---- 简易测试框架 ---- */
static int g_test_count = 0;
static int g_test_fail = 0;

#define M2006_BUS_TEST_ASSERT(cond, msg)                                      \
  do {                                                          \
    g_test_count++;                                            \
    if (!(cond)) {                                             \
      fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); \
      g_test_fail++;                                           \
    }                                                           \
  } while (0)

/* ---- 测试辅助 ---- */

static m2006_bus_t g_bus;
static m2006_motor_t g_motor_low;
static m2006_motor_t g_motor_high;

static void bus_with_two_motors(void)
{
  m2006_bus_init(&g_bus);
  m2006_motor_init(&g_motor_low, 2U);
  m2006_motor_init(&g_motor_high, 7U);
  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor(&g_bus, &g_motor_low, 2U) == 1U,
                        "attach low motor");
  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor(&g_bus, &g_motor_high, 7U) == 1U,
                        "attach high motor");
}

/* ---- 注册 ---- */

static void test_attach_rejects_bad_arguments(void)
{
  m2006_bus_t bus;
  m2006_motor_t motor;

  m2006_bus_init(&bus);
  m2006_motor_init(&motor, 1U);

  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor(&bus, &motor, 0U) == 0U,
                        "reject esc_id below range");
  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor(&bus, &motor, 9U) == 0U,
                        "reject esc_id above range");
  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor(&bus, (m2006_motor_t *)0, 1U) == 0U,
                        "reject null motor");
  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor((m2006_bus_t *)0, &motor, 1U) == 0U,
                        "reject null bus");
}

static void test_attach_rejects_occupied_slot(void)
{
  m2006_bus_t bus;
  m2006_motor_t motor_a;
  m2006_motor_t motor_b;

  m2006_bus_init(&bus);
  m2006_motor_init(&motor_a, 1U);
  m2006_motor_init(&motor_b, 2U);

  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor(&bus, &motor_a, 3U) == 1U,
                        "first attach ok");
  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor(&bus, &motor_b, 3U) == 0U,
                        "reject occupied slot");
  M2006_BUS_TEST_ASSERT(bus.motor_count == 1U, "motor count stays one");
}

/* ---- 控制帧聚合打包 ---- */

static void test_pack_empty_bus_returns_zero_frames(void)
{
  m2006_bus_t bus;
  uint32_t frame_id[2];
  uint8_t frame_data[2][M2006_PROTOCOL_FRAME_BYTES];

  m2006_bus_init(&bus);
  M2006_BUS_TEST_ASSERT(
      m2006_bus_pack_tx_frames(&bus, frame_id, frame_data) == 0U,
      "empty bus packs zero frames");
}

static void test_pack_low_group_only(void)
{
  m2006_bus_t bus;
  m2006_motor_t motor;
  uint32_t frame_id[2];
  uint8_t frame_data[2][M2006_PROTOCOL_FRAME_BYTES];
  uint32_t frame_count;

  m2006_bus_init(&bus);
  m2006_motor_init(&motor, 2U);
  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor(&bus, &motor, 2U) == 1U,
                        "attach low motor");
  motor.output_current = 1000;

  frame_count = m2006_bus_pack_tx_frames(&bus, frame_id, frame_data);
  M2006_BUS_TEST_ASSERT(frame_count == 1U, "one frame when only low group");
  M2006_BUS_TEST_ASSERT(frame_id[0] == M2006_PROTOCOL_CONTROL_ID_LOW,
                        "low frame id 0x200");
  /* esc_id=2 电流在 data[2..3] */
  M2006_BUS_TEST_ASSERT(frame_data[0][0] == 0x00U, "low byte0 zero");
  M2006_BUS_TEST_ASSERT(frame_data[0][1] == 0x00U, "low byte1 zero");
  M2006_BUS_TEST_ASSERT(frame_data[0][2] == 0x03U, "low byte2 0x03");
  M2006_BUS_TEST_ASSERT(frame_data[0][3] == 0xE8U, "low byte3 0xE8");
}

static void test_pack_high_group_only(void)
{
  m2006_bus_t bus;
  m2006_motor_t motor;
  uint32_t frame_id[2];
  uint8_t frame_data[2][M2006_PROTOCOL_FRAME_BYTES];
  uint32_t frame_count;

  m2006_bus_init(&bus);
  m2006_motor_init(&motor, 7U);
  M2006_BUS_TEST_ASSERT(m2006_bus_attach_motor(&bus, &motor, 7U) == 1U,
                        "attach high motor");
  motor.output_current = -500;

  frame_count = m2006_bus_pack_tx_frames(&bus, frame_id, frame_data);
  M2006_BUS_TEST_ASSERT(frame_count == 1U, "one frame when only high group");
  M2006_BUS_TEST_ASSERT(frame_id[0] == M2006_PROTOCOL_CONTROL_ID_HIGH,
                        "high frame id 0x1FF");
  /* esc_id=7 → 高段帧偏移 4..5（(7-5)*2 = 4）；-500 = 0xFE0C */
  M2006_BUS_TEST_ASSERT(frame_data[0][4] == 0xFEU, "high byte4 0xFE");
  M2006_BUS_TEST_ASSERT(frame_data[0][5] == 0x0CU, "high byte5 0x0C");
  M2006_BUS_TEST_ASSERT(frame_data[0][6] == 0x00U, "high byte6 zero (esc8 empty)");
  M2006_BUS_TEST_ASSERT(frame_data[0][7] == 0x00U, "high byte7 zero (esc8 empty)");
}

static void test_pack_both_groups(void)
{
  uint32_t frame_id[2];
  uint8_t frame_data[2][M2006_PROTOCOL_FRAME_BYTES];
  uint32_t frame_count;

  bus_with_two_motors();
  g_motor_low.output_current = 1000;   /* esc_id=2 → 低段偏移 2..3 */
  g_motor_high.output_current = -500;  /* esc_id=7 → 高段偏移 4..5 */

  frame_count = m2006_bus_pack_tx_frames(&g_bus, frame_id, frame_data);
  M2006_BUS_TEST_ASSERT(frame_count == 2U, "two frames when both groups");
  M2006_BUS_TEST_ASSERT(frame_id[0] == M2006_PROTOCOL_CONTROL_ID_LOW,
                        "frame0 is low group");
  M2006_BUS_TEST_ASSERT(frame_id[1] == M2006_PROTOCOL_CONTROL_ID_HIGH,
                        "frame1 is high group");
  M2006_BUS_TEST_ASSERT(frame_data[0][2] == 0x03U
                        && frame_data[0][3] == 0xE8U,
                        "low frame carries esc2 current");
  M2006_BUS_TEST_ASSERT(frame_data[1][4] == 0xFEU
                        && frame_data[1][5] == 0x0CU,
                        "high frame carries esc7 current");
  M2006_BUS_TEST_ASSERT(frame_data[0][0] == 0x00U, "low frame esc1 slot empty");
  M2006_BUS_TEST_ASSERT(frame_data[1][0] == 0x00U, "high frame esc5 slot empty");
  M2006_BUS_TEST_ASSERT(frame_data[1][6] == 0x00U, "high frame esc8 slot empty");
}

/* ---- 反馈分发 ---- */

static void test_rx_routes_to_attached_motor(void)
{
  static const uint8_t feedback_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x12U, 0x34U, 0x00U, 0x64U, 0x03U, 0xE8U, 0x00U, 0x00U
  };

  bus_with_two_motors();
  m2006_bus_handle_rx_frame(&g_bus, 0x202U, feedback_frame, 55U);

  M2006_BUS_TEST_ASSERT(g_motor_low.rx_msg_count == 1U,
                        "feedback routed to esc2 motor");
  M2006_BUS_TEST_ASSERT(g_motor_low.angle_raw == 0x1234U,
                        "esc2 motor raw angle updated");
  M2006_BUS_TEST_ASSERT(g_motor_low.speed_rpm == 100,
                        "esc2 motor raw speed updated");
  M2006_BUS_TEST_ASSERT(g_motor_low.last_rx_tick == 55U,
                        "esc2 motor tick updated");
  M2006_BUS_TEST_ASSERT(g_motor_high.rx_msg_count == 0U,
                        "esc7 motor untouched");
}

static void test_rx_ignores_control_id_and_out_of_range(void)
{
  static const uint8_t feedback_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
  };

  bus_with_two_motors();
  m2006_bus_handle_rx_frame(&g_bus, M2006_PROTOCOL_CONTROL_ID_LOW,
                            feedback_frame, 10U);
  m2006_bus_handle_rx_frame(&g_bus, 0x200U + 0U, feedback_frame, 10U);
  m2006_bus_handle_rx_frame(&g_bus, 0x200U + 9U, feedback_frame, 10U);
  m2006_bus_handle_rx_frame(&g_bus, 0x200U + 5U, feedback_frame, 10U);

  M2006_BUS_TEST_ASSERT(g_motor_low.rx_msg_count == 0U, "control id ignored");
  M2006_BUS_TEST_ASSERT(g_motor_high.rx_msg_count == 0U,
                        "unregistered slot ignored");
}

static void test_rx_routes_high_group_motor(void)
{
  static const uint8_t feedback_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x01U, 0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
  };

  bus_with_two_motors();
  m2006_bus_handle_rx_frame(&g_bus, 0x207U, feedback_frame, 70U);

  M2006_BUS_TEST_ASSERT(g_motor_high.rx_msg_count == 1U,
                        "feedback routed to esc7 motor");
  M2006_BUS_TEST_ASSERT(g_motor_high.angle_raw == 0x0102U,
                        "esc7 motor raw angle updated");
  M2006_BUS_TEST_ASSERT(g_motor_low.rx_msg_count == 0U,
                        "esc2 motor untouched");
}

int main(void)
{
  test_attach_rejects_bad_arguments();
  test_attach_rejects_occupied_slot();
  test_pack_empty_bus_returns_zero_frames();
  test_pack_low_group_only();
  test_pack_high_group_only();
  test_pack_both_groups();
  test_rx_routes_to_attached_motor();
  test_rx_ignores_control_id_and_out_of_range();
  test_rx_routes_high_group_motor();

  if (g_test_fail != 0)
  {
    (void)fprintf(stderr, "m2006_bus_test: %d/%d FAILED\n",
                  g_test_fail, g_test_count);
    return 1;
  }

  (void)printf("m2006_bus_test: PASS (%d asserts)\n", g_test_count);
  return 0;
}
