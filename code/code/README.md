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
