# UART 配置与乱码排查指南

本文用于后续新增或修改 UART 时避免再次出现“反复猜波特率、反复乱码”的问题。

## 配置前先确认

配置 UART 前必须先确认四件事：

1. **确认 UART 外设实例**：例如 `UART0`、`UART1`，不要只看串口助手端口号。
2. **确认实际引脚**：明确 MCU 的 TX、RX 分别接到 USB-TTL/CH340 的哪根线。
3. **确认共地**：外接 USB-TTL 时必须 MCU GND 与 USB-TTL GND 共地。
4. **确认电平类型**：只能接 TTL 电平串口，不要接 RS232 反相电平模块。

TX/RX 接线原则：

- MCU TX 接 USB-TTL RX。
- MCU RX 接 USB-TTL TX。
- GND 必须共地。

## 时钟源选择原则

UART 乱码优先怀疑“实际波特率不对”，不要先怀疑文本编码。

后续配置 UART 时优先使用确定时钟源：

- 优先：`MFCLK 4MHz`，适合 9600、115200 等常规调试串口。
- 谨慎：`BUSCLK/ULPCLK`，只有在已经明确当前外设时钟频率时才使用。
- 避免：一边改主频/ULPCLK 分频，一边沿用旧 UART 分频值。

当前工程的稳定方案：

```c
DL_SYSCTL_enableMFCLK();

static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};
```

## 分频计算流程

UART 分频必须按真实 UART 时钟计算：

```text
UARTDIV = UART_CLK / (16 * baud)
IBRD    = integer(UARTDIV)
FBRD    = round((UARTDIV - IBRD) * 64)
```

当前 UART0 使用 `MFCLK=4MHz`、`baud=115200`：

```text
UARTDIV = 4000000 / (16 * 115200) = 2.1701
IBRD    = 2
FBRD    = round(0.1701 * 64) = 11
```

对应代码：

```c
DL_UART_Main_setOversampling(UART_0_INST, DL_UART_MAIN_OVERSAMPLING_RATE_16X);
DL_UART_Main_setBaudRateDivisor(UART_0_INST, 2U, 11U);
```

## 最小验证方法

UART 新配置完成后，不要直接发送复杂文本。先按下面顺序验证：

1. 串口助手设置为目标波特率、8N1、无流控。
2. 先连续发送 ASCII `U`，即 `0x55`。
3. 若有逻辑分析仪或示波器，测单 bit 宽。
4. 115200 baud 的单 bit 宽约为 `8.68 us`。
5. 9600 baud 的单 bit 宽约为 `104.17 us`。
6. `U` 正常后，再发送短 ASCII 文本，例如 `UART OK\r\n`。

如果 `U` 都不稳定，不要继续改字符串内容，应先回头检查时钟、分频、接线、电平。

## 乱码排查顺序

出现乱码时按这个顺序排查：

1. 确认串口助手波特率、数据位、停止位、校验位、流控设置。
2. 确认 UART 实际时钟源和代码中的分频值一致。
3. 确认 TX/RX 没接反，GND 已共地。
4. 确认没有使用 RS232 反相电平模块。
5. 确认 MCU 引脚复用只配置到一个目标引脚，避免无意义镜像输出。
6. 用 `U` 字符测 bit 宽，按实测结果反推真实 UART 时钟。
7. 只有在 bit 宽正确后，才排查文本编码或上位机显示问题。

## 多任务打印注意事项

- UART0 是多个任务共享的调试输出口，不能让不同任务逐字节抢占发送。
- `BspUart0_SendString()` 在 FreeRTOS 调度器运行后会临时挂起调度器，保证一整条字符串完整输出。
- 当前 `IMU100Hz` 会按 100Hz 输出姿态调试行，日志较密；若后续 UART0 还要承载控制命令或业务通信，应降低 IMU 打印频率或改为独立发送队列。
- 若后续把 UART0 用作高频业务通信，应改成专用 UART 发送任务、队列或 DMA，避免长字符串阻塞调度。

## 当前工程固定约定

- UART0 使用 `MFCLK 4MHz`。
- UART0 波特率为 `115200 8N1`。
- UART0 TX 使用 `PA10`。
- UART0 RX 使用 `PA11`。
- PA10/PA11 属于核心板特殊功能风险引脚，本次已按用户确认使用。
- 串口助手使用 `115200 8N1，无流控`。
- UART1（张大头 Emm42_V5.0 闭环步进驱动）：`MFCLK 4MHz` + 16x 过采样，`115200 8N1`，TX=`PA17`、RX=`PB5`，分频 `IBRD=2`/`FBRD=11`（与 UART0 同参数）。驱动器出厂默认即 115200；若被改成 38400，改用已算好的 `IBRD=6`/`FBRD=33`（见 `ti_msp_dl_config.h` 的 `UART_1_*_38400_MFCLK`），不要去动时钟源。
- UART2（激光测距1）：`MFCLK 4MHz` + **8x 过采样**，`230400 8N1`，TX=`PB15`、RX=`PB16`，分频 `IBRD=2`/`FBRD=11`。

## 不要再做的事

- 不要在没有确认 UART 时钟源的情况下反复试 2400、9600、115200。
- 不要把乱码优先归因于中文编码。
- 不要同时改时钟源、引脚、分频、发送内容，否则无法判断根因。
- 不要为了“看看哪根线有输出”长期保留 TX 镜像；诊断完成后必须删掉镜像配置。
- 不要让 Keil 构建流程重新调用缺失的 TI SysConfig 工具。
