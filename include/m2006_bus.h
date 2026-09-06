/**
  ******************************************************************************
  * @file    m2006_bus.h
  * @brief   M2006 总线实例：一路 CAN 上的电机注册、控制帧聚合、反馈分发
  *
  * 本模块为纯逻辑（零 HAL、零 RTOS 依赖），与具体 FDCAN 外设解耦：
  * - 发送：m2006_bus_pack_tx_frames() 把挂载电机的输出电流聚合为
  *   0x200（电调 ID 1~4）与 0x1FF（电调 ID 5~8）两组控制帧，
  *   由调用方（HAL 适配层）依次发出；
  * - 接收：HAL RX 回调把 CAN ID 与数据喂给 m2006_bus_handle_rx_frame()，
  *   按反馈帧标识符 0x200+电调ID 路由到对应电机实例。
  *
  * 一路总线最多挂 8 台电机（C610 单总线容量）。
  ******************************************************************************
  */
#ifndef M2006_BUS_H
#define M2006_BUS_H

#include <stdint.h>

#include "m2006_motor.h"
#include "m2006_protocol.h"

/* 单路总线最大挂载电机数（C610 单总线容量） */
#define M2006_BUS_MAX_MOTORS (8U)

/**
  * @brief  M2006 总线实例（透明结构体）
  * @note   motor_slots 按电调 ID 索引（esc_id - 1），未挂载槽位为 0。
  */
typedef struct m2006_bus
{
  /* 电机实例指针表，索引 = esc_id - 1（0~7） */
  m2006_motor_t *motor_slots[M2006_BUS_MAX_MOTORS];

  /* 已挂载电机数 */
  uint8_t motor_count;
} m2006_bus_t;

/**
  * @brief  初始化总线实例：清空电机槽位与计数
  * @param  bus 总线实例指针
  */
void m2006_bus_init(m2006_bus_t *bus);

/**
  * @brief  注册电机：把电机实例挂到总线的指定电调 ID 槽位
  * @param  bus    总线实例指针
  * @param  motor  电机实例指针
  * @param  esc_id 电调 ID，范围 1~8
  * @retval 1 成功；0 参数为空、电调 ID 越界或槽位已被占用
  */
uint32_t m2006_bus_attach_motor(m2006_bus_t *bus,
                                m2006_motor_t *motor,
                                uint8_t esc_id);

/**
  * @brief  打包控制帧：把总线所有电机的输出电流聚合为 0x200 / 0x1FF 两组帧
  * @param  bus         总线实例指针
  * @param  frame_id    输出帧标识符数组，长度 2（仅前 frame_count 项有效）
  * @param  frame_data  输出帧数据数组，长度 2 × M2006_PROTOCOL_FRAME_BYTES
  * @retval 有效帧数 0~2：0 无电机；1 仅 0x200 或仅 0x1FF；2 两组都有
  * @note   frame_id 与 frame_data 按相同索引一一对应：低段帧（0x200，ID 1~4）
  *         优先排在索引 0，高段帧（0x1FF，ID 5~8）紧随其后；
  *         调用方按返回的帧数依次发送。
  */
uint32_t m2006_bus_pack_tx_frames(
    m2006_bus_t *bus,
    uint32_t frame_id[2],
    uint8_t frame_data[2][M2006_PROTOCOL_FRAME_BYTES]);

/**
  * @brief  处理一帧收到的 CAN 消息：按反馈帧标识符路由到对应电机并喂入反馈
  * @param  bus     总线实例指针
  * @param  can_id  CAN 标准帧标识符（反馈帧 0x200+电调ID，即 0x201~0x208）
  * @param  data    CAN 数据指针，长度 M2006_PROTOCOL_FRAME_BYTES
  * @param  tick_ms 当前时钟（毫秒，单调递增），透传给电机实例
  */
void m2006_bus_handle_rx_frame(m2006_bus_t *bus,
                               uint32_t can_id,
                               const uint8_t data[M2006_PROTOCOL_FRAME_BYTES],
                               uint32_t tick_ms);

#endif /* M2006_BUS_H */
