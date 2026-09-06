/**
  ******************************************************************************
  * @file    m2006_protocol.h
  * @brief   M2006 电机配合 C610 电调的 CAN 协议编解码（纯 C，零平台依赖）
  *
  * 协议依据《RoboMaster C610 无刷电机调速器使用说明》CAN 通信协议章节：
  * - 控制帧：标准帧 0x200（电调 ID 1~4）与 0x1FF（电调 ID 5~8），
  *   每个电调 ID 占 2 字节，高字节在前，
  *   电流值 -10000~+10000 对应 -10A~+10A（即 1000 LSB/A）。
  * - 反馈帧：标准帧 0x200+电调ID，DATA[0..1] 转子机械角度 0~8191，
  *   DATA[2..3] 转子转速 rpm（int16），DATA[4..5] 实际输出转矩（int16）。
  * 本模块为纯函数，不依赖 STM32 HAL 或 RTOS，可在主机端独立编译测试。
  ******************************************************************************
  */
#ifndef M2006_PROTOCOL_H
#define M2006_PROTOCOL_H

#include <stdint.h>

/* 协议常量 */
#define M2006_PROTOCOL_FRAME_BYTES (8U)          /* CAN 数据帧字节数 */
#define M2006_PROTOCOL_MOTOR_ID_MIN (1U)         /* 控制帧支持的最小电调 ID */
#define M2006_PROTOCOL_MOTOR_ID_MAX (8U)         /* 控制帧支持的最大电调 ID */
#define M2006_PROTOCOL_MOTOR_ID_HIGH_MIN (5U)    /* 第二控制帧（0x1FF）起始电调 ID */
#define M2006_PROTOCOL_CONTROL_ID_LOW (0x200U)   /* 控制帧：电调 ID 1~4 */
#define M2006_PROTOCOL_CONTROL_ID_HIGH (0x1FFU)  /* 控制帧：电调 ID 5~8 */
#define M2006_PROTOCOL_FEEDBACK_ID_BASE (0x200U) /* 反馈帧标识符 = 0x200 + 电调ID */
#define M2006_PROTOCOL_GEAR_RATIO (36U)          /* M2006 减速比 36:1 */
#define M2006_PROTOCOL_ANGLE_MAX (8191U)         /* 转子机械角度满量程 */
#define M2006_PROTOCOL_CURRENT_FULL_SCALE (10000) /* 电流满量程，对应 10A */

/* 输出轴角度换算系数：转子一圈 360° 经 36:1 减速 = 输出轴 10°/圈，再按 8191 归一 */
#define M2006_PROTOCOL_ANGLE_SCALE_DEG \
    (360.0f / (float)M2006_PROTOCOL_GEAR_RATIO / (float)M2006_PROTOCOL_ANGLE_MAX)

/* M2006 转矩常数（输出轴等效值，已含 36:1 减速比与传动效率）：0.18 N·m/A */
#define M2006_PROTOCOL_TORQUE_CONSTANT_NM_PER_A (0.18f)

/* 转子单圈相位角换算系数：编码器一圈 8191 LSB = 转子 360° */
#define M2006_PROTOCOL_ANGLE_RAW_SCALE_DEG \
    (360.0f / (float)M2006_PROTOCOL_ANGLE_MAX)

/* 电调反馈测量值 */
typedef struct m2006_measure
{
  uint16_t angle_raw;   /* 转子机械角度，范围 [0, 8191] */
  int16_t speed_rpm;    /* 转子转速，单位 rpm（高速侧，除以减速比得输出轴转速） */
  int16_t torque_raw;   /* 实际输出转矩原始值（实为电流环反馈电流，1000 LSB = 1A） */
} m2006_measure_t;

/**
  * @brief  解析电调反馈帧到测量结构体
  * @param  feedback_data  反馈帧数据指针，长度 M2006_PROTOCOL_FRAME_BYTES
  * @param  measure        输出测量结构体指针
  * @retval 1 成功；0 参数为空
  */
uint32_t m2006_protocol_parse_feedback(
    const uint8_t feedback_data[M2006_PROTOCOL_FRAME_BYTES],
    m2006_measure_t *measure);

/**
  * @brief  编码控制帧：把指定电调 ID 的电流写入对应字节偏移
  * @param  motor_id      电调 ID，范围 1~8（1~4 写入 0x200 帧偏移，5~8 写入 0x1FF 帧偏移）
  * @param  current_raw   控制电流值，范围 -10000~+10000
  * @param  control_data  输出控制帧数据指针，长度 M2006_PROTOCOL_FRAME_BYTES
  * @retval M2006_PROTOCOL_FRAME_BYTES 成功；0 参数为空或电调 ID 越界
  */
uint32_t m2006_protocol_encode_control(
    uint8_t motor_id,
    int16_t current_raw,
    uint8_t control_data[M2006_PROTOCOL_FRAME_BYTES]);

/**
  * @brief  角度回绕展开（纯函数）：把单圈编码差值折算为连续角度增量
  * @param  prev_raw 上一周期转子机械角度编码 [0, 8191]
  * @param  raw      本周期转子机械角度编码 [0, 8191]
  * @retval 有符号增量 LSB：正常差值；跨越 8191↔0 时按半圈阈值判别方向并折算，
  *         使多圈累计角度连续不回绕
  */
int32_t m2006_protocol_unwrap_angle(uint16_t prev_raw, uint16_t raw);

#endif /* M2006_PROTOCOL_H */
