/**
  ******************************************************************************
  * @file    m2006_bus.c
  * @brief   M2006 总线实例实现：电机注册、控制帧聚合打包、反馈分发
  * @note    本模块不依赖 STM32 HAL 或 RTOS，可在主机端直接编译测试。
  ******************************************************************************
  */
#include "m2006_bus.h"

void m2006_bus_init(m2006_bus_t *bus)
{
  uint8_t slot_index;

  if (bus == 0)
  {
    return;
  }

  for (slot_index = 0U; slot_index < M2006_BUS_MAX_MOTORS; ++slot_index)
  {
    bus->motor_slots[slot_index] = 0;
  }
  bus->motor_count = 0U;
}

uint32_t m2006_bus_attach_motor(m2006_bus_t *bus,
                                m2006_motor_t *motor,
                                uint8_t esc_id)
{
  uint8_t slot_index;

  if ((bus == 0) || (motor == 0))
  {
    return 0U;
  }

  if ((esc_id < M2006_PROTOCOL_MOTOR_ID_MIN)
      || (esc_id > M2006_PROTOCOL_MOTOR_ID_MAX))
  {
    return 0U;
  }

  slot_index = (uint8_t)(esc_id - M2006_PROTOCOL_MOTOR_ID_MIN);
  if (bus->motor_slots[slot_index] != 0)
  {
    return 0U;
  }

  bus->motor_slots[slot_index] = motor;
  bus->motor_count++;
  return 1U;
}

uint32_t m2006_bus_pack_tx_frames(
    m2006_bus_t *bus,
    uint32_t frame_id[2],
    uint8_t frame_data[2][M2006_PROTOCOL_FRAME_BYTES])
{
  uint8_t esc_id;
  uint8_t slot_index;
  uint8_t byte_index;
  uint8_t has_low_frame;
  uint8_t has_high_frame;
  uint32_t frame_count;

  if ((bus == 0) || (frame_id == 0) || (frame_data == 0))
  {
    return 0U;
  }

  /* 清两组控制帧数据 */
  for (byte_index = 0U; byte_index < M2006_PROTOCOL_FRAME_BYTES; ++byte_index)
  {
    frame_data[0][byte_index] = 0U;
    frame_data[1][byte_index] = 0U;
  }

  /* 第一遍：判断低段（ID 1~4）与高段（ID 5~8）是否挂有电机 */
  has_low_frame = 0U;
  has_high_frame = 0U;

  for (esc_id = M2006_PROTOCOL_MOTOR_ID_MIN;
       esc_id < M2006_PROTOCOL_MOTOR_ID_HIGH_MIN;
       ++esc_id)
  {
    slot_index = (uint8_t)(esc_id - M2006_PROTOCOL_MOTOR_ID_MIN);
    if (bus->motor_slots[slot_index] != 0)
    {
      has_low_frame = 1U;
    }
  }

  for (esc_id = M2006_PROTOCOL_MOTOR_ID_HIGH_MIN;
       esc_id <= M2006_PROTOCOL_MOTOR_ID_MAX;
       ++esc_id)
  {
    slot_index = (uint8_t)(esc_id - M2006_PROTOCOL_MOTOR_ID_MIN);
    if (bus->motor_slots[slot_index] != 0)
    {
      has_high_frame = 1U;
    }
  }

  /* 第二遍：按实际帧顺序打包，frame_id 与 frame_data 索引一一对应 */
  frame_count = 0U;

  if (has_low_frame != 0U)
  {
    frame_id[frame_count] = M2006_PROTOCOL_CONTROL_ID_LOW;
    for (esc_id = M2006_PROTOCOL_MOTOR_ID_MIN;
         esc_id < M2006_PROTOCOL_MOTOR_ID_HIGH_MIN;
         ++esc_id)
    {
      slot_index = (uint8_t)(esc_id - M2006_PROTOCOL_MOTOR_ID_MIN);
      if (bus->motor_slots[slot_index] != 0)
      {
        (void)m2006_protocol_encode_control(
            esc_id,
            bus->motor_slots[slot_index]->output_current,
            frame_data[frame_count]);
      }
    }
    frame_count++;
  }

  if (has_high_frame != 0U)
  {
    frame_id[frame_count] = M2006_PROTOCOL_CONTROL_ID_HIGH;
    for (esc_id = M2006_PROTOCOL_MOTOR_ID_HIGH_MIN;
         esc_id <= M2006_PROTOCOL_MOTOR_ID_MAX;
         ++esc_id)
    {
      slot_index = (uint8_t)(esc_id - M2006_PROTOCOL_MOTOR_ID_MIN);
      if (bus->motor_slots[slot_index] != 0)
      {
        (void)m2006_protocol_encode_control(
            esc_id,
            bus->motor_slots[slot_index]->output_current,
            frame_data[frame_count]);
      }
    }
    frame_count++;
  }

  return frame_count;
}

void m2006_bus_handle_rx_frame(m2006_bus_t *bus,
                               uint32_t can_id,
                               const uint8_t data[M2006_PROTOCOL_FRAME_BYTES],
                               uint32_t tick_ms)
{
  uint8_t esc_id;
  uint8_t slot_index;
  m2006_motor_t *motor;
  m2006_measure_t measure;

  if ((bus == 0) || (data == 0))
  {
    return;
  }

  /* 反馈帧标识符 = 0x200 + 电调ID，合法范围 0x201~0x208 */
  if ((can_id < M2006_PROTOCOL_FEEDBACK_ID_BASE + M2006_PROTOCOL_MOTOR_ID_MIN)
      || (can_id > M2006_PROTOCOL_FEEDBACK_ID_BASE + M2006_PROTOCOL_MOTOR_ID_MAX))
  {
    return;
  }

  esc_id = (uint8_t)(can_id - M2006_PROTOCOL_FEEDBACK_ID_BASE);
  slot_index = (uint8_t)(esc_id - M2006_PROTOCOL_MOTOR_ID_MIN);
  motor = bus->motor_slots[slot_index];
  if (motor == 0)
  {
    return;
  }

  if (m2006_protocol_parse_feedback(data, &measure) == 1U)
  {
    m2006_motor_feed_feedback(motor, &measure, tick_ms);
  }
}
