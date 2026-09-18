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

主循环还会调用 `BoardHeatingIndicator_Update()` 驱动 PA12/LED1：TIM17 实际
占空比大于 0 时，每 100 ms 翻转一次（完整周期 200 ms）；占空比为 0 时，
每 500 ms 翻转一次（完整周期 1 s）。闪烁使用 `HAL_GetTick()` 非阻塞计时，
不会影响温度采集和 PID 更新。

## 独立看门狗

固件在全部外设、采集应用和 PWM 输出初始化完成后启动 IWDG。看门狗使用独立
LSI 时钟、256 分频、重装值 499；按典型 32 kHz LSI 计算，超时时间约为 4 秒。
主循环只有在按键、四路采集、温控和 LED 指示任务全部返回后才喂狗。如果程序
卡死或任务长期不能返回，IWDG 将自动复位 MCU，并由上电安全逻辑先保持加热关闭。

## ADS124S08 配置

| 寄存器 | 值 | 含义 |
| --- | --- | --- |
| PGA (03h) | 08h | PGA 开启，增益 1 |
| DATARATE (04h) | 35h | 单次转换、低延迟滤波、50 SPS |
| REF (05h) | 52h | REFP0/REFN0，内部参考常开供 IDAC，并开启 0.3 V 参考监测 |
| IDACMAG (06h) | 04h | IDAC1/2 幅值 250 µA |
| SYS (09h) | 10h | 无 CRC、无状态前缀 |

通道顺序固定为：

1. IDAC1→AIN11，AIN0-AIN1。
2. IDAC1→AIN10，AIN2-AIN3。
3. IDAC1→AIN9，AIN4-AIN5。
4. IDAC1→AIN8，AIN6-AIN7。

IDAC2 始终断开；SPI 使用 Mode 1、8 MHz。每路单次转换的典型首次结果延迟
约为 26.504 ms，固件保守等待 35 ms；四路每 200 ms 采集一轮，即 5 Hz。

## 标定与串口输出

默认参考电阻为 3000.0 Ω，PT1000 标称电阻为 1000.0 Ω。调用
`PT1000_AppGetDefaultConfig()` 后，可修改参考电阻实测值以及各通道的
`resistance_gain`、`resistance_offset_ohm`。

USART2 在工作模式下每轮输出一行固定 18 列的纯数字 FireWater 数据，
当前输出频率与四路采集和 PID 更新频率一致，均为 5 Hz：

```text
25.01,25.03,24.99,25.02,43.00,25.03,17.97,3.000,0.025,0.000,53.91,0.00,0.00,30.00,30.00,0.20,4,1\n
```

列顺序固定为：

```text
T1,T2,T3,T4,SP,FB,ERR,KP,KI,KD,P,I,D,OUT,LIM,DT,VALID,STATE
```

`SP` 是目标温度，`FB` 是参与控制的合成反馈，`ERR=SP-FB`，
`P/I/D` 是三个实际输出分量，`OUT` 是限幅后的高有效 PWM 占空比，
`LIM` 是当前允许的占空比上限，`DT` 是本次计算间隔，`VALID` 是有效传感器数。
`KP/KI/KD` 固定显示三位小数，其余浮点列显示两位小数，因此 `Ki=0.025`
会原样输出，不再四舍五入显示为 `0.03`。
`STATE=1` 表示正在控制，`STATE=0` 表示传感器不足或控制被禁用，
`STATE=2` 表示任一有效通道达到 48 ℃ 后已锁定停机。状态 0 和 2 均强制
加热输出归零；过温锁定只能通过重新上电清除。失效温度列用 `0.00` 占位，
需结合 `VALID/STATE` 判断。ADC 原始码仍保存在内部快照中，但不输出到
FireWater，避免百万级码值把温度和 PID 曲线压缩在图表底部。

## PWM 与仅加热 PID

TIM17 使用 64 MHz 时钟、PSC=63、ARR=999，在 PD1/EN 输出 1 kHz 高有效 PWM。
每得到一轮新的四通道温度后更新一次 PID，当前反馈为所有有效通道中的最高温度。
至少两路传感器有效时允许控制；只剩一路或全部无效时，占空比立即归零
并清除积分项。

PID 参数入口位于 `Core/Src/main.c`：

```c
temperature_control_config.setpoint_c = 43.0f;
temperature_control_config.pid.kp = 3.0f;
temperature_control_config.pid.ki = 0.025f;
temperature_control_config.pid.kd = 0.0f;
temperature_control_config.pid.output_max = 30.0f;
temperature_control_config.pid.integral_max = 12.0f;
temperature_control_config.pid.integral_initial = 6.0f;
temperature_control_config.feedback_mode = TEMPERATURE_FEEDBACK_MAXIMUM;
temperature_control_config.overtemperature_c = 48.0f;
```

第一轮纯 P 数据显示最高温反馈稳定在约 40.63 ℃，稳态误差约 2.37 ℃，
维持输出约 7.12%。第二轮保持 `Kp=3.0 %/℃`，加入较缓慢的
`Ki=0.025 %/(℃·s)` 消除该稳态误差，`Kd` 暂时保持 0。积分项独立限制为
0～12%，并使用条件积分抗饱和：总输出到达上下限且误差会继续加重饱和时
暂停积分，误差反向时仍允许释放积分。根据 12 V、680 Ω 限流实测，系统维持
43 ℃ 约需 7.4% PWM，因此初始化时把积分项预置为 6%。这样首次接近目标时
输出不会因积分尚未建立而过早降到约 5%，可减轻 42～43 ℃ 区间的温度回落。
该预置值只在控制器初始化时装载；传感器故障、控制禁用或过温停机仍会清零积分。
当前 30% 限幅只限制占空比；TPS27S100 导通时的输出高电平仍接近 VIN。
本板未把 VIN 接入 MCU ADC，因此固件无法在 12～24 V 变化时自动换算功率限幅。

控制误差固定为“设定温度 - 反馈温度”，当前输出限制在 0～30%，所以框架只会
加热，不会产生制冷方向。PD1/EN 的 PWM 为高电平有效。反馈可选
平均值、最大值、最小值或指定通道，见 `TemperatureControl_FeedbackMode`。

### PWM 阶梯测试模式

`Core/Src/main.c` 中的 `PWM_RUN_MODE` 是唯一的模式选择宏。当前值为
`PWM_RUN_MODE_WORK`，固件使用四路温度反馈进行 PID 控制。需要验证硬件时，
可将该宏改为 `PWM_RUN_MODE_TEST`；固件将绕过 PID，让 PD1/EN 按
10 秒间隔循环输出 20%、40%、60%、80% 和 90% 高电平占空比。PWM 硬件
测试模式不执行 PID，因此不输出上述 FireWater 温控帧。

## 硬件限制

最新版 `Sheet1.SchDoc` 中，ADS124S08 的 START/SYNC 通过 AGND 电源端口接地，
RESET# 通过 3V3 电源端口拉高，满足数据手册的软件命令控制要求。独立 DRDY#
未连接，固件按数据手册第 71 页使用 SPI MISO 上复用的 DOUT/DRDY 状态轮询。
