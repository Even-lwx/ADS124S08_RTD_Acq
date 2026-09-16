# ADS124S08 四路 PT1000 采集库

## 硬件映射

| 功能 | STM32G031 | PCB 网络/器件 |
| --- | --- | --- |
| SPI SCLK | PA1 | ADS124S08 SCLK |
| SPI MISO | PA6 | ADS124S08 DOUT/DRDY# |
| SPI MOSI | PA7 | ADS124S08 DIN |
| SPI CS | PA5 | ADS124S08 CS# |
| Debug TX/RX | PA2/PA3, USART2 | H1-1/H1-2，115200 8N1 |
| 业务 TX/RX | PA9/PA10, USART1 | H3 跳线选择 RS485/TTL，9600 8N1 |
| RS485 方向 | PC6 | SN65HVD75 DE 与 RE#，低电平为接收 |

H3 短接 1-3、2-4 时 USART1 连接 RS485；短接 3-5、4-6 时连接 H4 TTL。

## ADS124S08 配置

| 寄存器 | 值 | 含义 |
| --- | --- | --- |
| PGA (03h) | 08h | PGA 开启，增益 1 |
| DATARATE (04h) | 34h | 单次转换、低延迟滤波、20 SPS |
| REF (05h) | 12h | REFP0/REFN0，内部参考常开供 IDAC 使用 |
| IDACMAG (06h) | 04h | IDAC1/2 幅值 250 uA |
| SYS (09h) | 10h | 无 CRC、无状态前缀 |

通道顺序固定为：

1. IDAC1→AIN11，AIN0-AIN1；
2. IDAC1→AIN10，AIN2-AIN3；
3. IDAC1→AIN9，AIN4-AIN5；
4. IDAC1→AIN8，AIN6-AIN7。

IDAC2 始终断开。SPI 使用 Mode 1、8 MHz。

## 标定与输出

默认参考电阻为 3000.0 Ω，PT1000 标称电阻为 1000.0 Ω。可在
`PT1000_AppGetDefaultConfig()` 返回后修改参考电阻实测值，以及每通道
`resistance_gain`、`resistance_offset_ohm`。

USART2 每秒输出：

```text
raw1,temp1,raw2,temp2,raw3,temp3,raw4,temp4\r
```

异常温度字段输出 `nan`，USART2 不输出其他日志。

## 硬件限制

当前原理图中 ADS124S08 的 START/SYNC、RESET#、DRDY# 未连接。软件命令控制要求
START/SYNC 保持低电平、RESET# 保持高电平；若实际引脚悬空，软件只能报告超时，
无法保证采集稳定。
