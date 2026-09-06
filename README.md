# m2006_lib

STM32 平台 M2006 无刷电机（配 C610 电调）的纯 C 复用库。零 HAL、零 RTOS 依赖，仅依赖 `Lib/pid_lib`（同为独立仓库）。

## 目录结构

- `include/` 公开头文件
- `src/` 实现
- `tests/` 主机端单元测试

## 分层

| 模块 | 职责 |
| --- | --- |
| `m2006_protocol` | CAN 协议编解码纯函数（控制帧/反馈帧/换算） |
| `m2006_motor` | 电机实例：开环 / 速度环 / 位置环闭环，超时断输出、电流钳位、超速保护等安全门 |
| `m2006_bus` | 总线实例：多电机挂载、控制帧聚合（0x200/0x1FF）、反馈帧路由 |

## 集成方式

以 git submodule 接入工程（与 pid_lib 同一套工作流）：

```bash
git submodule add https://github.com/GaGiaa/m2006_lib.git Lib/m2006_lib
```

## 单元测试

主机端 gcc 编译运行（示例，按实际编译器调整）：

```bash
gcc -Iinclude src/m2006_motor.c src/m2006_protocol.c src/m2006_bus.c tests/m2006_motor_test.c -o tests/m2006_motor_test.exe
./tests/m2006_motor_test.exe
```
