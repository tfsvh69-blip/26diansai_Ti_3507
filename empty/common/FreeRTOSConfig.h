/*
 * FreeRTOS Kernel V11.3.0
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*
 * MCU：Texas Instruments MSPM0G3507，Cortex-M0+。
 * 当前板级时钟：SYSOSC 32MHz -> SYSPLL -> MCLK/CPUCLK 80MHz。
 */

/*========== 基础配置 ==========*/
#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
/*
 * 栈溢出检测：2=每次任务切换检查栈顶魔术字与指针越界，命中即调 vApplicationStackOverflowHook。
 * 之前未定义(默认0)，钩子形同虚设；开启后 IMU 等栈紧任务一旦溢出能立刻捕获（钩子里会闪 LED+打串口）。
 * 可配合 uxTaskGetStackHighWaterMark 查各任务余量。
 */
#define configCHECK_FOR_STACK_OVERFLOW          2
/* FreeRTOS SysTick 使用 CPUCLK，必须和板级 SYSPLL 主频保持一致。 */
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 80000000 )
/* FreeRTOS tick 频率为 1000Hz，即 1ms 调度节拍。 */
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 5 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 )
/*
 * 堆 16KB（heap_4）。当前全部任务栈 + TCB 实际用约 6.5~7KB（关软件定时器后又省 ~1KB），
 * 16KB 堆有近半空闲，headroom 充足——之前担心的"余量偏紧"其实是把整块 16KB 堆算进了
 * ZI(总 RW+ZI≈26KB/32KB)，堆本身只用了一半。功能开关关掉外设会进一步释放堆。
 * 若后续需要更多静态 RAM（如大 DMA 缓冲），可把堆调到 12KB 腾出 4KB；当前保持 16KB 留扩展余量。
 */
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 16 * 1024 ) )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_TRACE_FACILITY                1
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_QUEUE_SETS                    0
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_TIME_SLICING                  1

/*========== 内存管理 ==========*/
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configAPPLICATION_ALLOCATED_HEAP        0

/*========== 软件定时器 ==========*/
/*
 * 当前工程未使用任何软件定时器(xTimerCreate)，故关闭：不再创建 Timer 服务任务，
 * 省下约 1KB 栈 + 队列 + TCB。将来要用软件定时器时改回 1 并同步开 INCLUDE_xTimerPendFunctionCall。
 */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/*========== 协程配置：Cortex-M0+ 项目不使用协程 ==========*/
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/*========== 断言 ==========*/
/*
 * 断言失败不再静默死循环：转调 vAssertCalled()（在 main.c 实现），
 * 关中断后闪烁 LED1 并尝试打印一行原因，便于现场判断"是卡死了还是断言挂了"。
 */
#ifndef __ASSEMBLER__
extern void vAssertCalled(const char *file, unsigned long line);
#endif
#define configASSERT( x )                       if( ( x ) == 0 ) { vAssertCalled( __FILE__, __LINE__ ); }

/*========== 可选 API ==========*/
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xSemaphoreGetMutexHolder        1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          0   /* 依赖软件定时器，已随 configUSE_TIMERS=0 关闭 */

/*========== 中断优先级 ==========*/
/*
 * Cortex-M0+ 当前按 2 位优先级配置。
 * 在 ISR 中调用 FreeRTOS API 时，必须使用 FromISR 版本。
 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         3
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    1
#define configKERNEL_INTERRUPT_PRIORITY                 ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - 2 ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY            ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - 2 ) )

/*========== Tickless 模式 ==========*/
#define configUSE_TICKLESS_IDLE                         0

/*
 * 【勿删】本 SDK 的 ARM_CM0 移植层是"统一 MPU 支持"版本，portmacro.h/port.c 会 #error
 * 强制要求定义下面这几个宏（即使 M0+ 无 MPU/TrustZone）。都保持 0/1 的现值即可：
 * configENABLE_MPU=0(无 MPU)、configENABLE_TRUSTZONE=0(无 TZ)、
 * configRUN_FREERTOS_SECURE_ONLY=1(无 TrustZone 的单映像构建，此值对 M0+ 正确)。
 * 曾误当作"CMSIS-RTOS2 残留"删除，结果 portmacro.h 直接 #error，故恢复保留。
 */
#define configENABLE_FPB                               0
#define configENABLE_MPU                               0
#define configENABLE_TRUSTZONE                         0
#define configRUN_FREERTOS_SECURE_ONLY                 1

#endif /* FREERTOS_CONFIG_H */
