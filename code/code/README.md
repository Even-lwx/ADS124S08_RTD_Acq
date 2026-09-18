# ADS124S08 四路 PT1000 采集与温控库

## 硬件映射

| 功能 | STM32G031 | PCB 网络/器件 | 有效电平 |
| --- | --- | --- | --- |
| SPI SCLK | PA1 | ADS124S08 SCLK | SPI Mode 1 |
| SPI MISO | PA6 | ADS124S08 DOUT/DRDY# | 低表示数据就绪 |
| SPI MOSI | PA7 | ADS124S08 DIN | — |
| SPI CS | PA5 | ADS124S08 CS# | 低有效 |
| Debug TX/RX | PA2/PA3，USART2 | H1-1/H1-2，115200 8N1 | — |
| 业务 TX/RX | PA9/PA10，USART1 | H3 跳线选择 RS485/TTL，9600 8N1 | — |
| RS485 方向 | PC6 | SN65HVD75 DE 与 RE# | 高发送、低接收 |
| PID PWM | PD1，TIM17_CH1 | PCB 的 EN 网络，1 kHz | 高有效 |
| 按键 SW2 | PB3 | BUTTON，R19/R20/SW2 | 按下为低 |
| 黄色 LED1 | PA12 | LED1、R21 | 低电平点亮 |
| 黄色 LED3 | PA15 | LED3、R27 | 低电平点亮 |

H3 短接 1-3、2-4 时 USART1 连接 RS485；短接 3-5、4-6 时连接 H4 TTL。

按键与 LED 极性来自最新原理图
`ADS124S08‑RTD‑Acq-Ver1.0/Sheet1.SchDoc`：BUTTON 节点由 R19（4.7 kΩ）上拉，
按下 SW2 后接地；LED1/LED3 的阳极分别经 1 kΩ 电阻接 3V3，阴极由 MCU 灌电流。

## 自有库目录

- `ads124s08/`：SPI 命令、寄存器配置、ID 校验和单次转换。
- `pt1000/`：比例码值转电阻、通道标定和 IEC 60751 温度换算。
- `pid/`：带积分限幅和输出限幅的离散 PID。
- `app/`：四通道周期采集、串口组帧和仅加热温控。
- `board_io/`：板载 LED 与带 20 ms 非阻塞消抖的按键驱动。

## 按键与 LED 接口

`MX_GPIO_Init()` 后调用 `BoardIO_Init()`，并在主循环中持续调用
`BoardButton_Update()`。应用层可使用：

```c
BoardLED_Set(BOARD_LED_1, 1U);       /* 点亮 LED1 */
BoardLED_Toggle(BOARD_LED_3);        /* 翻转 LED3 */

if (BoardButton_GetPressedEvent())   /* 每次稳定按下只返回一次 */
{
  BoardLED_Toggle(BOARD_LED_1);
}
```

`BoardButton_IsPressed()` 返回当前消抖状态；`BoardButton_GetPressedEvent()` 和
`BoardButton_GetReleasedEvent()` 分别读取并清除按下/释放事件。驱动已封装低有效极性。

## ADS124S08 配置

| 寄存器 | 值 | 含义 |
| --- | --- | --- |
| PGA (03h) | 08h | PGA 开启，增益 1 |
| DATARATE (04h) | 34h | 单次转换、低延迟滤波、20 SPS |
| REF (05h) | 52h | REFP0/REFN0，内部参考常开供 IDAC，并开启 0.3 V 参考监测 |
| IDACMAG (06h) | 04h | IDAC1/2 幅值 250 µA |
| SYS (09h) | 10h | 无 CRC、无状态前缀 |

通道顺序固定为：

1. IDAC1→AIN11，AIN0-AIN1。
2. IDAC1→AIN10，AIN2-AIN3。
3. IDAC1→AIN9，AIN4-AIN5。
4. IDAC1→AIN8，AIN6-AIN7。

IDAC2 始终断开；SPI 使用 Mode 1、8 MHz。

## 标定与串口输出

默认参考电阻为 3000.0 Ω，PT1000 标称电阻为 1000.0 Ω。调用
`PT1000_AppGetDefaultConfig()` 后，可修改参考电阻实测值以及各通道的
`resistance_gain`、`resistance_offset_ohm`。

USART2 每秒只输出一帧，不混入日志：

```text
raw1,temp1,raw2,temp2,raw3,temp3,raw4,temp4\r\n
```

原始码为有符号十进制，温度保留两位小数，异常温度字段为 `nan`。

### USART2 诊断输出

为排查 ADC 未接传感器、SPI 接线和外部参考欠压问题，USART2/H1（115200 8N1）
现在每轮在正式温度帧后额外输出一行 `DBG` 诊断帧。USART1/RS485 完全不发送这些诊断信息，
其 PA9/PA10 和 PC6 方向控制保持原有预留状态。

示例：

```text
DBG,INIT=0,ID=08,ST=80,REG=67:08:34:52:04:F8,RT=S80,AV=0:2768241,IT=0:838861,C1=0:2796203:S80,C2=6:8388607:S81,C3=6:8388607:S81,C4=6:8388607:S81,B=7F:FF:FF\r\n
```

字段含义：

- `INIT`：最近一次 ADS124S08 初始化状态。
- `ID`：读取的器件 ID 寄存器；正常 ADS124S08 的低 3 位应为 `0`。
- `ST`：STATUS 寄存器。bit7=`FL_POR`，bit6=`RDY`（器件通信就绪标志），
  bit1/bit0=`FL_REF_L1/FL_REF_L0`；其中 bit0 为 1 表示外部参考低于 0.3 V。
- `REG`：从 INPMUX 开始读回的 6 个寄存器，依次为 INPMUX、PGA、DATARATE、REF、IDACMAG、IDACMUX。
- `RT`：IDAC1 临时路由到 AIN1 后的参考回路自检 STATUS。该路径绕过
  PT1000 和 AIN11，仅经过 `IDAC1 → AIN1 → REFP0 → R17 → AGND`。
  `RT=S80` 表示 IDAC、AIN1回流、REFP0 和 R17 正常；`RT=S81` 表示这条
  最小参考回路仍然低于 0.3 V。
- `AV`：使用内部 2.5 V 参考测量 AVDD/4，格式为 `状态码:原始码`。
  AVDD=3.3 V 时原始码应约为 `2768241`；该测试不依赖 PT1000、R17 或
  REFP0，可用于判断 ADS124S08 模拟供电和内部参考是否工作。
- `IT`：使用内部 2.5 V 参考测量 CH1 的 AIN0-AIN1，同时保持
  IDAC1→AIN11，格式为 `状态码:原始码`。U1 焊盘跨接 1 kΩ 时应约为
  `838861`。该测试不使用 R17 产生的外部参考电压，可检查 IDAC 和 CH1
  电流回路；电流仍需经 R17 回到 AGND。
- `C1`～`C4`：每个通道的最终状态码、原始 ADC 码和该通道转换后的
  STATUS，格式为 `状态码:原始码:Sxx`。例如 `S81` 表示该通道参考欠压，
  `S80` 表示仅保留上电复位标志且参考监测正常。
- `B`：最近一次 RDATA 返回的三个原始字节；转换超时时不会执行 RDATA，因此该值可能保留上次结果。

状态码定义：`0=成功`、`1=参数错误`、`2=SPI 错误`、`3=ID 错误`、
`4=寄存器校验错误`、`5=转换超时`、`6=ADC 饱和`。若看到 `C1=5:0`，说明
DOUT/DRDY 在 150 ms 内没有变低；若看到 `INIT=2` 或 `ID=XX`，优先检查
SPI、CS、DOUT/DRDY 接线。驱动会在每次 START 前读回最低位为 1 的 INPMUX，
先把复用的 DOUT/DRDY 强制恢复为高。数据手册表 13 指出 20 SPS 低延迟
单次转换首个结果约需 56.504 ms，因此驱动等待 70 ms 后才判断数据就绪，
避免把上一次残留的低电平和旧零值当成本次结果。

## PWM 与仅加热 PID

TIM17 使用 64 MHz 时钟、PSC=63、ARR=999，在 PD1/EN 输出 1 kHz 高有效 PWM。
每得到一轮新的四通道温度后更新一次 PID，默认反馈为四路平均温度。任一路温度
无效时，占空比立即归零并清除积分项。

PID 参数入口位于 `Core/Src/main.c`：

```c
temperature_control_config.setpoint_c = 25.0f;
temperature_control_config.pid.kp = 0.0f;
temperature_control_config.pid.ki = 0.0f;
temperature_control_config.pid.kd = 0.0f;
```

控制误差固定为“设定温度 - 反馈温度”，输出固定限制在 0～100%，所以框架只会
加热，不会产生制冷方向。PD1/EN 的 0% 为持续低电平，100% 为持续高电平。反馈可选
平均值、最大值、最小值或指定通道，见 `TemperatureControl_FeedbackMode`。

## 硬件限制

最新版 `Sheet1.SchDoc` 中，ADS124S08 的 START/SYNC 通过 AGND 电源端口接地，
RESET# 通过 3V3 电源端口拉高，满足数据手册的软件命令控制要求。独立 DRDY#
未连接，固件按数据手册第 71 页使用 SPI MISO 上复用的 DOUT/DRDY 状态轮询。
## CH1 无传感器调试接法

原理图中的 CH1 是“采样端和激励端在 PT1000 焊盘处汇合”的三线结构。网表显示：

```text
AIN0 ─ 100 Ω ─┐
              ├─ U1（PT1000 焊盘）
AIN11 ────────┘      ┌─ U1 ───── REFP0
AIN1 ─ 100 Ω ───────┘
```

因此，替代 U1 做测试时，应把 1 kΩ 电阻接在 **U1 的两个 PT1000 焊盘**，
也就是 AIN11 一侧和 REFP0 一侧（可直接在对应焊盘上跨接）。不能只把电阻
接在 AIN0 与 AIN1：这两个是经过 100 Ω 引线电阻后的采样节点，U1 焊盘
未接通时，AIN11 的 IDAC 电流没有回路。

250 µA IDAC、1 kΩ 被测电阻和 3 kΩ R17 时，预期：

```text
VRTD ≈ 0.25 V
VREF ≈ 0.75 V
ADC code ≈ 8388608 × 1000 / 3000 ≈ 2796203
```

DBG 帧中的 `ST=81` 表示 `FL_POR=1` 且 `FL_REF_L0=1`，即参考电压低于
0.3 V；在 IDAC 回路断开时这是预期现象。接好并确认回路导通后该状态应变为
`ST=80`（首次上电 POR 标志仍可能保持）或最低位清零，`B=xx:xx:xx` 应不再是
`00:00:00`。
