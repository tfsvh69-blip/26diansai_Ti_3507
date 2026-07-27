# 仓库协作指南

## 首读文件与同步规则

每次阅读、分析或修改本仓库前，必须先阅读根目录的 `CLAUDE.md`，再阅读本文件。`CLAUDE.md` 保存详细工程约定，本文件提供简明执行指南；二者共同生效。

凡是修改编码、构建、硬件、任务、测试或协作流程规则，必须在同一次改动中同步更新 `CLAUDE.md` 和 `AGENTS.md`，确保内容不矛盾。新增或修改的代码注释、README、`docs/` 文档、协作指南及其他开发说明统一使用中文；命令、文件路径、代码标识符、协议字段和必要专有名词除外。

## 项目结构

工程主体在 `empty/`：`app/` 放任务、业务流程和题目状态机，题目 N 使用 `app/tasks/taskN.c`；题目一 `DIR TEST`（M1→M4 单轮正方向测试）、题目二 `ARC TEST`（固定半径差速圆弧测试）、题目三 `GYRO 90L`（陀螺仪 PID 闭环左转 90°，✅ 冒烟测试已通过），菜单任一按键短促嘀声（~2ms），M1/M2 已在 BSP 全局取反标定。步进电机命令统一直接下发、不做加减速。`algo/` 放可跨题复用的纯算法（PID 控制器、角度差值/归一化），任务层只保留可调参数宏。`bsp/` 放板级与外设驱动，手写 DriverLib 初始化位于 `bsp/board/ti_msp_dl_config.c`；`module/` 放可复用设备/协议模块；`common/` 放功能开关、FreeRTOS 配置和共享消息；`docs/` 放任务、接线、UART 和架构记录。

`source/ti/` 与 `empty/third_party/` 属于 SDK 或第三方代码，除非任务明确涉及 SDK 或 FreeRTOS 移植，否则不要修改。

## 构建、烧录与验证

用 Keil uVision 打开 `empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`，按 `Build (F7)` 编译，再用 `Download` 烧录。也可在 `empty/keil/` 目录执行：

```powershell
UV4.exe -b empty_LP_MSPM0G3507_nortos_keil.uvprojx `
  -t empty_LP_MSPM0G3507_nortos_keil -o build.log
```

以 `build.log` 出现 `0 Error(s)` 为编译通过。SysConfig 已停用，`empty.syscfg` 仅作历史参考。仓库没有自动化测试框架；固件改动除编译外还应实机验证 UART0 的两条 `BOOT:` 信息、PB25 每 300 ms 心跳，以及受影响外设的实际行为。

## 代码与硬件约束

沿用现有 C 风格：四空格缩进、既有大括号格式；板级接口使用 `Bsp*`，应用入口使用 `App_*`，赛题状态机使用 `TaskN_OnEnter/OnLoop/OnExit`，常量和功能开关使用大写蛇形命名，例如 `APP_FEATURE_IMU`。关键硬件操作、中断、状态切换和安全保护应写必要的中文注释；中断保持短小，涉及 FreeRTOS 时使用 `FromISR` 接口。

修改引脚分配或外设复用前，必须查阅 `pcb引脚配置文档/v1.1/机器人控制板_接线说明.md` 及其网表；它们的优先级高于代码注释和 Git 历史。行为、接线、任务或报文变化时，同步更新对应的 `empty/docs/` 文档。

## 提交与合并请求

提交标题使用简短中文，可带版本或范围前缀，例如 `v1.9.3：修复电机急停状态`、`docs：更新 UART2 接线说明`。保持一次提交只处理一个主题。合并请求应说明目的、影响的硬件/模块、构建结果、实机验证结果和配置变更；界面或 OLED 行为变化时附上可见证据。
