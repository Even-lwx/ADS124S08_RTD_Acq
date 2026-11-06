# ADS124S08 四路 PT1000 采集与温控

本工程用于四路 PT1000 电阻温度采集和加热温控，主控为 STM32G031，包含 ADS124S08 驱动、PT1000 换算、PID 控制、RS485/Modbus RTU 通信以及 PCB 设计。

## 目录

- `code/`：固件、传感器驱动、温控算法和 Modbus 协议。
- `pcb/`：原理图、PCB 和本地元件库。
- `doc/`：ADS124S08 和相关器件资料。
- `tools/`：原理图解析脚本与 qModMaster 第三方 Modbus 主站调试工具，见 [qModMaster 使用说明](tools/qModMaster使用说明.md)。
- `analysis/schematic/`：原根目录的 `sch.txt` 与 `sch_latest.txt` 原理图解析记录，仅用于复核，设计以 `pcb/` 中源文件为准。

## 固件功能

固件按固定顺序采集四路 PT1000，使用参考电阻换算阻值，再按照 IEC 60751 关系换算温度。温控器使用带积分限幅、输出限幅和过温保护的 PID，只允许加热输出。RS485 方向由 PC6 控制，Modbus RTU 从机地址和寄存器定义见 `code/code/README.md`。

## 使用方法

使用 STM32CubeIDE 或 Keil MDK 打开 `code/` 中的工程。上电前确认 PT1000 接线、IDAC 回路、RS485 终端和加热使能信号。首次调试应先读取温度和状态寄存器，再验证目标温度、PWM 输出和过温停机。

电脑端可运行 `tools/qModMaster-Win64-exe-0.5.3-beta/qModMaster/qModMaster.exe`，作为 Modbus RTU 主站读取板卡寄存器。该目录是第三方工具发布包，包含运行所需的 DLL，并非本工程自行开发的上位机源码。

## 文档约定

项目自有说明和解释性注释使用中文；芯片型号、寄存器名称、Modbus 功能码、C 标识符和串口协议字段保留原文。`code/Drivers/` 中的厂商库保留原始版权和官方注释。
