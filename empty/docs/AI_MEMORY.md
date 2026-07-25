# AI 维护记忆

- 代码注释统一使用中文；关键逻辑、硬件操作、中断处理、任务入口、状态切换和安全保护要写必要注释。
- 每次完成代码修改后，要说明烧录或运行后应该观察到的现象，方便用户排查。
- MSPM0G3507 FreeRTOS 工程不要使用 `SYSCFG_DL_init()` 做总初始化，避免其中的 SysTick 初始化占用 FreeRTOS SysTick。
- 当前主频配置为 80 MHz；`CPUCLK_FREQ` 和 `configCPU_CLOCK_HZ` 都必须保持为 `80000000`。
- 【2026-07-14 时钟源改为外部晶振】80 MHz 现由核心板 40MHz 外部晶振 HFXT 经 SYSPLL 倍频锁定：`HFXT 40MHz -> PDIV=/2 -> 20MHz -> QDIV=8 -> VCO 160MHz -> CLK0=/2 -> 80MHz`（PLL 结构体 `sysPLLRef=REF_HFCLK`）。关键：`rDivClk0=0` 表示 CLK0=VCO/2，不是/1；VCO=参考频率×qDiv。
- HFXT 引脚 PA5=HFXIN(`IOMUX_PINCM10`)、PA6=HFXOUT(`IOMUX_PINCM11`)，必须在 GPIO_init 里 `DL_GPIO_initPeripheralAnalogFunction` 配为模拟功能，否则不起振。范围枚举用 `DL_SYSCTL_HFXT_RANGE_32_48_MHZ`。
- 【锁死坑】DriverLib `setHFCLKSourceHFXT`/`configSYSPLL` 内部会死等 HFCLK_GOOD/SYSPLL_GOOD；晶振不起振会永久卡死、串口无输出像变砖。本工程用 `SYSCFG_DL_tryStartHFXT()`（`monitor=false` 不进死等 + 自带超时轮询）规避，超时回退内部 SYSOSC 备用 PLL 配置（`PDIV=/2、QDIV=10`），保证仍 80MHz、串口可用。启动打印实际源，标志 `g_sysClockUsingHFXT`。
- 外设时钟（UART0/I2C1/步进定时器）仍走内部 MFCLK 4MHz，与主频时钟源切换解耦，UART 115200 的 IBRD=2/FBRD=11 不变，晶振只锁 CPU 主频、不动 FreeRTOS 节拍。
- 当前工程已与 TI SysConfig 生成流程解耦：Keil `BeforeMake` 不再调用 `syscfg.bat`，`empty.syscfg` 仅作历史参考，`ti_msp_dl_config.c/h` 手写维护。
- 尽量不要使用核心板特殊功能引脚：A23、A21、A20、A19、A18、A11、A10、A5、A6、A4、A3、A2；确需使用时必须先说明风险并等待人工确认。
- 硬件接线统一维护在 `docs/HARDWARE_WIRING.md`；修改 GPIO、外设复用或引脚时必须同步更新。
- 【2026-07-15 迁 v1.1】引脚以 `pcb引脚配置文档/v1.1/机器人控制板_接线说明.md` 为准，v1.0 扩展板配置废弃。本次把 LED1(PB22→**PB25**)、TMC 细分(PB8/PB9→**PB0/PB1**)、KEY4(PA17→**PA29**) 全部改到 v1.1，并新增 LED2=PA7、LED3=PB12、蜂鸣器=PA15、恢复板载 OLED(PB8/PB9)。
- 当前 LED1=**PB25**，v1.1 高电平点亮；`App_Init()` 启动 `LED1` 任务后每 300ms 翻转一次，用作 FreeRTOS 心跳灯。LED2=PA7、LED3=PB12 由 `PERIPH` 外设测试任务翻转。
- OLED：v1.1 板载 0.96 寸 OLED 恢复到 PB8/PB9 软件 I2C（约定 SCL=PB9/SDA=PB8），驱动只推挽输出不读 ACK；TMC 细分已改到 PB0/PB1，两者不再冲突。`OLED.c::IIC_delay` 位延时从 10us 降到 2us，减少全屏刷新对同优先级任务的忙等阻塞。由 `PERIPH` 任务刷屏显示调试测试数据。
- 蜂鸣器：PA15 有源蜂鸣器，高电平响，普通 GPIO（无需 PWM）；`bsp_buzzer.c` 提供 On/Off/Toggle；`PERIPH` 任务每 5s 通断一次做测试。
- 【MSPM0G3507 IOMUX PINCM 速查（2026-07-15 用权威头文件 `source/ti/devices/msp/m0p/mspm0g350x.h` 逐条核对）】PA7=12、PA13=35、PA15=37、PA16=38、PA17=39、PA28=3、PA29=4、PA30=5、PA31=6；PB0=13、PB1=14、PB2=15、PB3=16、PB8=25、PB9=26、PB10=27、PB11=28、PB12=29、PB22=50、**PB25=56**（此前误记为 58，58 实际是 PB27；代码里一直用对的 PINCM56）。⚠️ 查 PINCM 必须用 `mspm0g350x.h`（本工程芯片专属），不同系列头文件（如 `mspm0l122x.h`）的编号不通用。
- 任务配置集中放在 `common/app_config.h`；新增或修改任务后同步更新 `docs/FREERTOS_TASKS.md`。
- 串口乱码最终解决思路：不要继续猜文本编码或 BUSCLK 频率，改用确定的 MFCLK 4MHz 作为 UART0 时钟源；115200 使用 16x 过采样，`IBRD=2`、`FBRD=11`。
- 后续新增或修改 UART 时必须先按 `docs/UART_DEBUG_GUIDE.md` 的流程确认实例、引脚、共地、电平、时钟源、分频和 bit 宽，不要直接反复试波特率。
- 当前 UART0 使用 PA10=TX、PA11=RX；PA10/PA11 属于核心板特殊功能风险引脚，本次已按用户确认使用；串口助手设置为 115200 8N1，无流控。
- 当前 UART0 接收测试不再控制 LED1；收到任意非换行字符后回显 `UART RX OK`，LED1(PB25) 固定作为心跳灯使用。
- ATK-MS6DSV 当前接线：SCL=PB2/B02，SDA=PB3/B03，INT=PA16/A16，SA0 接地，LSM6DSV16X 7bit I2C 地址为 `0x6A`。
- ATK-MS6DSV 当前 SCL/SDA 已外接上拉，代码仍启用 MCU 内部上拉；2026-06-18 曾改为 PB2/PB3 GPIO 软件 I2C，对齐正点原子官方例程，显式执行最后一字节 NACK 和 STOP，避免硬件 I2C 读事务后 SDA 被拉低。
- 【2026-07-14 确定使用软件 I2C】端口层 `bsp_imu_port.c` 使用纯 GPIO 软件 I2C（开漏模拟），对齐参考项目 v1.3 的读取方式。首次访问时关闭 I2C1 硬件控制器、PB2/PB3 切为 GPIO 模式；此后所有 I2C 通信通过 GPIO 位操作实现。重复起始读时序：写寄存器地址→重复起始→读 N 字节，末字节 NACK 后 STOP。
- 上层 module/imu 只用 `BspImuPort_WriteReg/ReadReg/ProbeAddress` 三个接口，底层为纯 GPIO 软件 I2C 实现；总线诊断/引脚测试函数基于 GPIO 位操作。
- 上层 module/imu 只用 `BspImuPort_WriteReg/ReadReg/ProbeAddress` 三个接口，换 I2C 内核（硬件/软件）时上层零改动；总线诊断/引脚测试函数始终用 GPIO 位操作，硬件模式下调用后下次读写由 `UseHwI2cPins` 重置回 I2C 复用。
- 2026-06-18 软件 I2C 修改后，用户实测 IMU 已能连续输出姿态；`FIFO=0/1/2` 小范围跳动正常，因为 SFLP 为 120Hz、任务读取为 100Hz。
- 2026-06-18 用户日志出现 `IMU INIT FAIL:2 LAST_ID=0x70`，说明 WHO_AM_I 已读通，后续排查重点应放在 reset/config 阶段和 SDA 被拉低；代码已增加 `STEP=...` 初始化步骤输出。
- 2026-06-18 用户进一步定位到 `STEP=RESET_SET LAST_ID=0x70` 后 SDA 被拉低；当前 `ATK_MS6DSV_USE_BOOT_RESET` 默认为 0，跳过 ST 驱动 `RESTORE_CTRL_REGS`/boot reset，直接配置传感器。
- TI DriverLib 的 I2C `ADDR_ACK`/`DATA_ACK` 状态位表示已经 ACK，不是错误位；I2C 轮询错误判断只应把 `ERROR` 和 `ARBITRATION_LOST` 当作失败。
- `APP_IMU_I2C_PIN_TEST_ENABLE` 是临时硬件排查宏；置 1 时 IMU 任务只用开漏模拟方式翻转 PB2/PB3 并读取 GPIO DIN 输出 SET/READ，不会读取 IMU。2026-06-18 用户实测 PB2/PB3 SET/READ 一致，MCU 端 GPIO 读写正常；当前宏已改回 0。
- MSPM0G3507 上 PB2/PB3 的 I2C1 复用为 `IOMUX_PINCM15_PF_I2C1_SCL` 和 `IOMUX_PINCM16_PF_I2C1_SDA`；用户接线表里的 U2.15/U2.17 是板口/封装编号，不要误把 SDA 配成 `PINCM17`。
- `IMU100Hz` 任务周期 10ms：融合欧拉角(FIFO/SFLP)每 10ms 读一次；UART0 整行输出经 `APP_IMU_PRINT_DIVIDER`(默认 5) 节流到 20Hz。LSM6DSV16X 内部 ODR 无精确 100Hz 档位，加速度、陀螺仪和 SFLP 使用 120Hz。
- 【2026-07-14 IMU 新增原始数据输出】除融合欧拉角外，`AtkMs6dsv_ReadImuRaw()`（`module/imu`）用 `lsm6dsv16x_acceleration_raw_get`/`angular_rate_raw_get` 寄存器直读三轴加速度(mg，±2g)和三轴角速度(mdps，±125dps)，串口整行追加 `AX/AY/AZ`(mg) 和 `GX/GY/GZ`(mdps)，供后续算法用。量程若不够(快速转向易超 125dps、冲击易超 2g)可在 `AtkMs6dsv_ConfigureSensorFusion` 改 FS 并换对应换算宏。
- 【软件 I2C 读取开销实测账】当前软件位操作 I2C：延时常量 `BSP_IMU_I2C_DELAY_CYCLES=CPUCLK_FREQ/200000`=每次延时 5µs(与频率无关)，发送位≈3延时=15µs、读位≈2延时=10µs，`DL_Common_delayCycles` 全程忙等不让出 CPU。一次 6 字节寄存器读≈1.0ms，两组(加速度+陀螺)≈2.0ms/周期。若按 100Hz 读，仅这两组就吃≈20% CPU。
- 【欧拉角数值范围与环绕】串口 `R/P/Y` 是融合欧拉角(度)：`P`(pitch)/`Y`(yaw) 范围 -180~+180，`R`(roll) 由 `asin` 解算只有 -90~+90；都在 ±180 处环绕跳变(+179°→-179°)。`GX/GY/GZ` 是角速度(mdps，±125000)不是角度，勿混。⚠️ 后续用 yaw 做累计转角/PID/积分必须先 unwrap 或取最短差值(把 `target-current` 规范到 ±180)，否则跳变点控制量瞬间打满。
- 【2026-07-14 方案A：加速度/陀螺按打印节拍读】原始加速度/角速度当前只用于串口显示、无 100Hz 消费者，故 `ReadImuRaw` 只在「要打印那一拍」(20Hz) 执行，不再每周期读，读取开销从 ~2ms/10ms(20%) 降到 ~0.4ms/10ms(4%)。欧拉角仍每周期读以排空 FIFO。将来若有姿态融合/卡尔曼等算法需要 100Hz 原始数据，再把 `ReadImuRaw` 移回每周期，并优先恢复硬件 I2C(TPR=0=400kHz)把单次读从 ~1ms 压到 ~0.2ms。
- 工程已迁到 v1.1 机器人主控板（四路 TMC2209 步进 + 舵机 + 循迹 + 激光测距 + 板载 OLED/IMU），引脚以 `pcb引脚配置文档/v1.1/机器人控制板_接线说明.md` 为准（v1.0 配置废弃）；OLED 恢复板载 PB8/PB9，TMC 细分改到 PB0/PB1。
- 电机1驱动：STEP=PB10(TIMG0_CCP0 硬件定时器)、DIR=PB11、ENN=PA13(低有效，四路共用)、MS1=**PB0**/MS2=**PB1**(四路共用细分)；STEP 定时器源 MFCLK 4MHz、预分频 ÷1（物理最小，prescale=0）→ 定时器时钟=4MHz；步频 = 4MHz / period，period 由梯形加减速动态调整（起步 4000→1kHz，巡航由参数决定，慢 625→6.4kHz、快 125→32kHz）。黄排针模块细分译码：LL=1/8、HH=1/16、HL=1/32、LH=1/64。
- 步进电机"原地剧烈抖动、不转"的首要根因是相线圈配对错(把两个线圈各掏一根凑成一对)，不是方向反(方向反只会反转不抖)；断电量电阻找两组导通对即可定位。区分共振/丢步：降到 200Hz 还抖就是配对错。本工程用户改对线序后电机正常旋转。
- TMC2209 不接 UART 时电流由 VREF 电位器标定，VREF 设每相 RMS 电流(`Irms≈VREF×0.71`，随模块 Rsense 变)；以温热不烫、捏轴有反抗力矩为准，用户已调到合适数值。
- 步进电机从静止**直接起高频会失步**：实测 20kHz(2500 全步/秒)直接起转，电机只抖 5~10° 不转(定时器照发完 1600 脉冲)；起转频率(本电机约 1kHz/125 全步每秒)远低于运动后可达的巡航频率。高速必须配加减速斜坡。
- 【2026-07-15 电机改连续旋转+在线调速调向】`bsp_motor.c` 由「定长旋转」改为「连续旋转」模型：`BspMotor1_RunContinuous(cruisePeriod)` 起转/在线设目标速度、`BspMotor1_SetSpeed()` 在线调速、`BspMotor1_RequestStop()` 请求平滑减速停、`BspMotor1_IsStopped()` 查停稳。TIMG0 ZERO 中断持续出脉冲不再倒计步数，ISR 每脉冲把当前周期朝目标(停止时朝起步 4000)逼近 `MOTOR_RAMP_DELTA=8`，减速到 4000 且已请求停止时停表。旧 `StartRotateSteps/IsRotateDone/StartStep` 已删除。
- 电机1现为四按键连续控制(`app_motor_test_task.c`，任务名 MOTOR1，状态机 M_STOPPED/RUNNING/STOPPING/REVERSING)：20ms 轮询 + 按下沿。**K1 启停 / K2 换向 / K3 加速 / K4 减速**。换向=运行中先 RequestStop 停稳(M_REVERSING)再按新方向重启，避免带速换向失步。速度 5 档周期 `625/400/250/175/125`(≈1/1.6/2.5/3.6/5 圈每秒，1/32 细分)。`MOTOR_AUTO_START_ON_BOOT`=1 上电自动正向最慢档起转。诊断结构 `AppMotorDiag_t` 改为 `running/dirForward/speedLevel` 三字段供 OLED 显示。
- 【2026-07-15 UART0 去卡顿】UART0 锁由 `vTaskSuspendAll`(冻结整个调度器 ~10ms/行) 改为**递归互斥量**(`bsp_uart.c`，`BspUart0_Init` 在 `BspBoard_Init` 调度器启动前建)：发送整行只独占串口、不冻结调度器，其余任务照常时间片轮转，从根上消除电机+IMU+舵机+OLED 并行时的全局卡顿。`SendString` 单调用自动加锁即原子，多段拼接用 `Lock/Unlock`(递归可嵌套)。
- 四个功能按键接线（v1.1）：KEY1=PA28/PINCM3、KEY2=PA31/PINCM6、KEY3=PA30/PINCM5、KEY4=**PA29/PINCM4**，一端接 GND、内部上拉，按下为低；都不在核心板慎用引脚列表内。`bsp_key.c` 提供 `BspKey_IsPressed()` 读瞬时电平。
- 【踩坑】全部设备上电状态下烧录后，IMU（LSM6DSV16X）必定读不出来——即使代码完全正确。根因：烧录期间 IMU 一直带电，芯片内部状态机未经历上电复位（POR），软件 boot/reset 序列无法将其从残留状态中恢复正常。**解决：烧录后必须拔掉 Type-C 数据线或电源彻底断电，再重新上电**，让 IMU 经历完整的 POR 周期。断电→重上电后即可正常工作，不需要改代码。⚠️ v1.1 机器人板上 IMU 的 3V3 来自开发板 LDO、由 **+5V** 供，而 +5V 有 **XL4015(电池) 和 Type-C 两路**（接线图风险 R3）；只掉一路电、只按 RST、只重烧都**不算真断电**，两路都拔掉等几秒才让 IMU 经历 POR。
- 【踩坑 2026-07-16 换模块才好·区分"临时锁死"vs"永久损坏"】某次起 IMU 一直 `INIT FAIL:2 STEP=WHOAMI`、`0x6A/0x6B=ERR`、`SCAN none`、总线 `SCL=1 SDA=1`（上拉正常、器件对自己地址完全不 ACK）。逐一排除代码/引脚(v1.1 逐网络核对无误)/时钟后，**彻底断电重启也无效，换一颗新的 MS6DSV 模块立刻恢复正常**——是老模块**硬件损坏**（芯片在总线上变哑），非软件问题。关键区分：① **临时锁死**（芯片是好的，如上一条 POR 未复位）→ 全断电几秒可恢复；② **永久损坏**（芯片坏了）→ 断电也救不回，**只能换模块**。软件无法救活一颗连地址都不 ACK 的从机（复位/切模式/改寄存器都要先被 ACK 才发得进去，`BspImuPort_RecoverBus` 的恢复时钟+STOP 也试过无效）。**IMU 读不出的排查顺序**：先按上条两路电全拔彻底断电重启一次（治临时锁死）→ 仍不行且引脚/供电都确认无误 → **直接换一颗新模块验证**，别再纠结改代码。
- 【IMU 保护·勿回退】`SYSCFG_DL_I2C_1_init()` 现**不调用** `DL_I2C_enableController()`：上电到 IMU 任务接管软件 I2C 的空窗期，若硬件 I2C 接管 PB2/PB3 可能在总线上打 glitch，把健康传感器推入锁死态（反复冲击疑似会加速把老模块搞到永久损坏）。软件 I2C 全程由 GPIO 位操作控制，此修复务必保留，不要为了"兼容旧接口"把 `enableController` 加回去。

- 【舵机1 调试记录 2026-07-15】：
  - 硬件：SERVO1=PA8(PINCM19)，TIMA0_CCP0，PF=5。TIMA0 为 GPTIMER_Regs* 类型，可复用 DL_Timer 通用 API。
  - **排查过程**：
    1. 初版用 DriverLib 配 TIMA0 50Hz PWM，舵机不响应（有电但可拧动）。
    2. 加裸机诊断：步骤A→PA8 切 GPIO 闪 5 次，验证引脚通路 OK。
    3. 步骤B→裸寄存器写 TIMA0，万用表测不到 PWM 输出→**漏了 CCACT（比较动作）寄存器**，CCP 不会翻转。
    4. 改为 DriverLib 函数 + 关 GPIO DOE 后重试，万用表测到 **67.4Hz** 波形→PWM 已产生！
    5. **67.4Hz≠50Hz 根因**：MSPM0G 的 CPS 预分频编码是 `divider = CPS + 1`（**非** `2^CPS`）。CPS=2 → ÷3 → 4MHz/3≈1.33MHz → period=20000 → 66.7Hz≈67.4Hz。
    6. **修复**：CPS 2→3（÷4 → 1MHz → 50Hz），见 `ti_msp_dl_config.h:SERVO_TIMER_PRESCALE`。
    7. **当前状态（2026-07-15）**：CPS=3 后 PA8 实测 **50.4Hz** 3.3V 方波，PWM 频率已正确。**但舵机仍不响应**（有电、可拧动）。PA8 直流档测到约 3V，远高于 1.5ms/20ms=7.5% 应有的 0.25V→**疑似 PWM 极性反了**（OCTL INV 位或 CCACT 动作极性导致输出 18.5ms 高/1.5ms 低而非 1.5ms 高/18.5ms 低）。待下一轮排查。
  - **TIMG 步进已验证 CPS=0→÷1(4MHz→20kHz)，与 CPS+1 编码一致**：0+1=1，÷1=4MHz，period=200→20kHz ✓。
  - bsp_servo：`BspServo_SetPulseUs()` 写 TIMA0 CC_01，`BspServo_Start/Stop()` 启停计数器。测试任务 `SERVO1` 20ms 周期、10us 步长在 600~2400us 间往复，单程约 3.6 秒。
  - 舵机引脚全貌：SERVO1=PA8/PINCM19/CCP0(PF=5)、SERVO2=PA9/PINCM20/CCP1(PF=5)、SERVO3=PB4/PINCM17/CCP2(PF=5)、SERVO4=PA12/PINCM34/CCP3(PF=6)。
  - **【2026-07-15 已解决·用户实测舵机正常摆动】** 根因：TIMA0 用 DriverLib `EDGE_ALIGN`(向下计数)，CCP 输出恒为「周期起点置高、向下计到 CC 时置低」→ 高电平 = `period - CC`，写入的 CC 实际是**低电平宽度**；CC=1500 得到 18.5ms 高 / 1.5ms 低，极性反，舵机识别不了、PA8 直流被拉到 ~3V。曾把 `pwmMode` 改成 `EDGE_ALIGN_UP` 想让 CC==高电平脉宽，**实测无效**（PA8 仍 ~3V，底层原因未深究）。**最终修复**：保持 `EDGE_ALIGN`，在 `bsp_servo.c::BspServo_SetPulseUs()` 写入 `(SERVO_TIMER_PERIOD - pulseUs)` 做极性补偿，使 PA8 实际高脉冲宽度 == pulseUs；用户实测舵机正常来回摆动。教训：舵机对脉冲极性敏感，而电机 STEP 只要方波、占空比反了仍能转——所以同样 `EDGE_ALIGN` 下电机能转、舵机不动。

- 【2026-07-16 激光测距1接入·UART2】测距模块1(单点激光)走 **UART2**：MCU 视角 **PB15=TX(PINCM32) / PB16=RX(PINCM33)**，复用功能 PF2，v1.1 排针 **H21**。协议移植自参考工程 `26RuiKang-AppleRobot-STM32-5.1` 的 `lidar_manager/ld14`：**195 字节定长帧**，帧头 `0xAA x4` + 命令字 `0x02` + 12 点(每点 15B，偏移 10 起) + 时间戳 + 校验和(前 194 字节累加取低 8 位，兼容含/不含帧头两种口径)；解析取 12 点非零距离**平均**为单值 mm。新增纯软件解析模块 `module/laser/laser_ld14.c`(`LaserLd14_FeedByte` / `LaserLd14_GetLatest`)。**波特率 230400**（依据参考工程 SC16 通道A 配置推定；实物若不符只改 `ti_msp_dl_config.h` 的 `UART_2_BAUD_RATE` 及分频即可。MFCLK 4MHz + 8x 过采样，IBRD=2/FBRD=11，误差约 -0.08 百分点，与 UART0 115200@16x 同一分频）。**关键：230400 连续流必须走 UART2 RX 中断**(`UART2_IRQHandler` in bsp_uart.c，逐字节喂解析器，ISR 内不调用 FreeRTOS API)——RX FIFO 仅几字节深，任务轮询必丢字节；激光不是 FreeRTOS 任务。距离由 `IMU100Hz` 任务在打印整行时追加 `D1=<mm>mm`，与陀螺仪同一行输出，打印节流从 20Hz 降到 **5Hz**(`APP_IMU_PRINT_DIVIDER` 5→20)便于阅读。电平风险 R2：激光 TX 若为 5V 而 PB16 非 5V 容忍，接线前先量 TX 电平。命令行编译 0 Error / 0 Warning。

- 【2026-07-16 舵机1/2/3不动·根因=setCCPDirection坑】现象：4 路舵机只有舵机4(PA12/CCP3)能动，舵机1/2/3(PA8/PA9/PB4=CCP0/1/2)信号线恒 ~0.3V 不动。**根因**：`DL_Timer_setCCPDirection(gptimer, cfg)` 内部是 `gptimer->CCPD = cfg`（**整体覆盖**整个 CCPD 寄存器，不是或入一位）。原 servo 初始化分 4 次单独调用 `setCCPDirection(CC0_OUTPUT)`…`(CC3_OUTPUT)`，结果只有最后一次(CC3=0x8)留下，CCP0/1/2 退回输入态、无 PWM 输出。**修复**：四路方向必须**一次调用**把 CC0/1/2/3_OUTPUT(0x1|0x2|0x4|0x8=0xF) 全或在一起写入。单通道的电机 STEP 定时器只调一次 CC0_OUTPUT 所以没中招。教训：凡是「整体写寄存器」的 DL_setXxx，多通道/多位一定要一次性或好再写，别分多次调用互相覆盖。

- 【2026-07-16 舵机脉宽范围收窄 + 独立控制确认】
  - **范围**：安全脉宽从 600~2400us 收到 **800~2200us**（中心 1500 不变），见 bsp_servo.h。原因：常规舵机在 ~600/~2400us 两端常顶机械限位，命令了也转不到或堵转发抖发热（用户观测到极值不灵）。800~2200 留余量，仍有 ±700us≈±63° 行程，SetPulseUs 自动限幅到此区间。注意这不是 MCU 侧问题：极性补偿(period-pulseUs)在任何脉宽都精确，极值无效是舵机机械限位所致。若某舵机行程不同按需改这两个宏。
  - **独立控制**：4 路舵机在 TIMA0 上各有独立比较寄存器(CCP0/1/2/3)，只共用 50Hz 时基，脉宽互不影响，`BspServo_SetPulseUs(BSP_SERVO_x, us)` 可单独控制任意一路、其它不动，本来就完全独立。app_servo_test_task 现改为四路错相独立摆动来演示（每秒串口打印 S1/S2/S3/S4 脉宽，数值各不相同即证明独立）。

- 【2026-07-25 继电器接入 + 门控解耦】继电器 RELAY=**PA24**(PINCM54，接口 P1-3)，普通 GPIO 推挽驱动大电流电磁铁负载，封装 `bsp/bsp_relay.c`(`On/Off/Set/Toggle/IsOn`)。**极性已上板实测确认：高电平=吸合、低电平=断开**(`BSP_RELAY_ACTIVE_LOW=0`；换低电平触发模块改 1 反相)；上电默认断开(PA24 下拉+清零 + `BspRelay_Init` 收敛)。OLED 底部状态栏最前显示 `R:ON `/`R:OFF`，**等宽坑**：`ON`(2字符)/`OFF`(3字符)不等宽会让后面 Yaw/距离左右跳，已给 `ON` 补一空格成 `ON `(3字符)固定，状态栏 buf 24→32。**门控解耦（本次）**：`APP_FEATURE_RELAY`(默认1)=继电器功能(板级初始化 + OLED 状态显示 + 对外接口 `BspRelay_*` 可调用)、`APP_FEATURE_RELAY_SELFTEST`(默认0)=每 2s 自动切换的自检任务 `RELAYTEST`。此前二者由同一个 `APP_FEATURE_RELAY` 门控(默认1即自动切换)，已改为解耦。**正常运行不自动切换**，继电器由业务代码 `#include "bsp_relay.h"` 调 `BspRelay_On/Off/Set` 按需控制；想恢复上电自检把 `APP_FEATURE_RELAY_SELFTEST` 置 1。接线风险 R7：PA24 上电前高阻可能误吸合，硬件建议加 10kΩ 下拉+100Ω 限流；代码侧内部下拉+上电清零只压低概率、不能替代外部下拉。命令行编译 0 Error/0 Warning。
