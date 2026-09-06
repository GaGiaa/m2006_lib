/**
  ******************************************************************************
  * @file    m2006_protocol_test.c
  * @brief   m2006_protocol 主机端单元测试（gcc 编译运行，不依赖嵌入式平台）
  *
  * 编译：gcc -Wall -Wextra -I Lib/m2006_lib/include
  *       -o Lib/m2006_lib/tests/m2006_protocol_test.exe
  *       Lib/m2006_lib/tests/m2006_protocol_test.c Lib/m2006_lib/src/m2006_protocol.c
  * 运行：./Lib/m2006_lib/tests/m2006_protocol_test.exe
  *
  * 覆盖：控制帧编码（ID 1~8 字节偏移）、反馈帧解析、角度回绕展开、非法参数。
  ******************************************************************************
  */
#include <stdint.h>
#include <stdio.h>

#include "m2006_protocol.h"

static int expect_bytes_equal(const uint8_t *actual_bytes,
                              const uint8_t *expected_bytes,
                              uint32_t byte_count)
{
  uint32_t byte_index;

  for (byte_index = 0U; byte_index < byte_count; ++byte_index)
  {
    if (actual_bytes[byte_index] != expected_bytes[byte_index])
    {
      return 0;
    }
  }

  return 1;
}

static int test_encode_control_id1_uses_first_two_bytes(void)
{
  static const uint8_t expected_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x03U, 0xE8U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
  };
  uint8_t encoded_frame[M2006_PROTOCOL_FRAME_BYTES];

  if (m2006_protocol_encode_control(1U, 1000, encoded_frame)
      != M2006_PROTOCOL_FRAME_BYTES)
  {
    return 0;
  }

  return expect_bytes_equal(encoded_frame,
                            expected_frame,
                            M2006_PROTOCOL_FRAME_BYTES);
}

static int test_encode_control_id2_positive_current(void)
{
  static const uint8_t expected_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x00U, 0x00U, 0x03U, 0xE8U, 0x00U, 0x00U, 0x00U, 0x00U
  };
  uint8_t encoded_frame[M2006_PROTOCOL_FRAME_BYTES];

  if (m2006_protocol_encode_control(2U, 1000, encoded_frame)
      != M2006_PROTOCOL_FRAME_BYTES)
  {
    return 0;
  }

  return expect_bytes_equal(encoded_frame,
                            expected_frame,
                            M2006_PROTOCOL_FRAME_BYTES);
}

static int test_encode_control_id2_negative_current(void)
{
  static const uint8_t expected_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x00U, 0x00U, 0xFCU, 0x18U, 0x00U, 0x00U, 0x00U, 0x00U
  };
  uint8_t encoded_frame[M2006_PROTOCOL_FRAME_BYTES];

  if (m2006_protocol_encode_control(2U, -1000, encoded_frame)
      != M2006_PROTOCOL_FRAME_BYTES)
  {
    return 0;
  }

  return expect_bytes_equal(encoded_frame,
                            expected_frame,
                            M2006_PROTOCOL_FRAME_BYTES);
}

static int test_encode_control_id4_uses_last_two_bytes(void)
{
  static const uint8_t expected_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x03U, 0xE8U
  };
  uint8_t encoded_frame[M2006_PROTOCOL_FRAME_BYTES];

  if (m2006_protocol_encode_control(4U, 1000, encoded_frame)
      != M2006_PROTOCOL_FRAME_BYTES)
  {
    return 0;
  }

  return expect_bytes_equal(encoded_frame,
                            expected_frame,
                            M2006_PROTOCOL_FRAME_BYTES);
}

static int test_encode_control_id5_uses_first_two_bytes_of_high_frame(void)
{
  /* ID 5~8 编码到 0x1FF 帧，字节偏移与 ID 1~4 相同（0/2/4/6） */
  static const uint8_t expected_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x03U, 0xE8U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
  };
  uint8_t encoded_frame[M2006_PROTOCOL_FRAME_BYTES];

  if (m2006_protocol_encode_control(5U, 1000, encoded_frame)
      != M2006_PROTOCOL_FRAME_BYTES)
  {
    return 0;
  }

  return expect_bytes_equal(encoded_frame,
                            expected_frame,
                            M2006_PROTOCOL_FRAME_BYTES);
}

static int test_encode_control_id8_uses_last_two_bytes_of_high_frame(void)
{
  static const uint8_t expected_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xFCU, 0x18U
  };
  uint8_t encoded_frame[M2006_PROTOCOL_FRAME_BYTES];

  if (m2006_protocol_encode_control(8U, -1000, encoded_frame)
      != M2006_PROTOCOL_FRAME_BYTES)
  {
    return 0;
  }

  return expect_bytes_equal(encoded_frame,
                            expected_frame,
                            M2006_PROTOCOL_FRAME_BYTES);
}

static int test_encode_control_rejects_bad_arguments(void)
{
  uint8_t encoded_frame[M2006_PROTOCOL_FRAME_BYTES];

  if (m2006_protocol_encode_control(0U, 0, encoded_frame) != 0U)
  {
    return 0;
  }

  if (m2006_protocol_encode_control(9U, 0, encoded_frame) != 0U)
  {
    return 0;
  }

  if (m2006_protocol_encode_control(1U, 0, (uint8_t *)0) != 0U)
  {
    return 0;
  }

  return 1;
}

static int test_parse_feedback_values(void)
{
  static const uint8_t feedback_frame[M2006_PROTOCOL_FRAME_BYTES] = {
    0x12U, 0x34U, 0xFFU, 0x9CU, 0x03U, 0xE8U, 0x00U, 0x00U
  };
  m2006_measure_t measure;

  if (m2006_protocol_parse_feedback(feedback_frame, &measure) != 1U)
  {
    return 0;
  }

  if (measure.angle_raw != 0x1234U)
  {
    return 0;
  }

  /* 0xFF9C 作为 int16 为 -100 */
  if (measure.speed_rpm != -100)
  {
    return 0;
  }

  if (measure.torque_raw != 1000)
  {
    return 0;
  }

  return 1;
}

static int test_parse_feedback_rejects_null_arguments(void)
{
  uint8_t feedback_frame[M2006_PROTOCOL_FRAME_BYTES] = {0U};
  m2006_measure_t measure;

  if (m2006_protocol_parse_feedback((const uint8_t *)0, &measure) != 0U)
  {
    return 0;
  }

  if (m2006_protocol_parse_feedback(feedback_frame, (m2006_measure_t *)0) != 0U)
  {
    return 0;
  }

  return 1;
}

static int test_unwrap_angle_forward(void)
{
  return m2006_protocol_unwrap_angle(100U, 200U) == 100;
}

static int test_unwrap_angle_wrap_forward(void)
{
  return m2006_protocol_unwrap_angle(8000U, 100U) == 292;
}

static int test_unwrap_angle_wrap_backward(void)
{
  return m2006_protocol_unwrap_angle(100U, 8000U) == -292;
}

static int test_unwrap_angle_zero(void)
{
  return m2006_protocol_unwrap_angle(5000U, 5000U) == 0;
}

static int test_unwrap_angle_boundary(void)
{
  return m2006_protocol_unwrap_angle(8191U, 0U) == 1;
}

static int test_unwrap_angle_wrap_backward_zero_boundary(void)
{
  return m2006_protocol_unwrap_angle(0U, 8191U) == -1;
}

int main(void)
{
  if (!test_encode_control_id1_uses_first_two_bytes())
  {
    return 1;
  }

  if (!test_encode_control_id2_positive_current())
  {
    return 2;
  }

  if (!test_encode_control_id2_negative_current())
  {
    return 3;
  }

  if (!test_encode_control_id4_uses_last_two_bytes())
  {
    return 4;
  }

  if (!test_encode_control_id5_uses_first_two_bytes_of_high_frame())
  {
    return 5;
  }

  if (!test_encode_control_id8_uses_last_two_bytes_of_high_frame())
  {
    return 6;
  }

  if (!test_encode_control_rejects_bad_arguments())
  {
    return 7;
  }

  if (!test_parse_feedback_values())
  {
    return 8;
  }

  if (!test_parse_feedback_rejects_null_arguments())
  {
    return 9;
  }

  if (!test_unwrap_angle_forward())
  {
    return 10;
  }

  if (!test_unwrap_angle_wrap_forward())
  {
    return 11;
  }

  if (!test_unwrap_angle_wrap_backward())
  {
    return 12;
  }

  if (!test_unwrap_angle_zero())
  {
    return 13;
  }

  if (!test_unwrap_angle_boundary())
  {
    return 14;
  }

  if (!test_unwrap_angle_wrap_backward_zero_boundary())
  {
    return 15;
  }

  (void)printf("m2006_protocol_test: PASS\n");
  return 0;
}
