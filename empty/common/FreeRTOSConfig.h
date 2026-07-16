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
/* FreeRTOS SysTick 使用 CPUCLK，必须和板级 SYSPLL 主频保持一致。 */
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 80000000 )
/* FreeRTOS tick 频率为 1000Hz，即 1ms 调度节拍。 */
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 5 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 )
/*
 * 堆大小 8KB→16KB：新增 PERIPH 任务后，全部 7 个任务(含 idle/timer)栈 + TCB 共约 7.9KB，
 * 8KB 堆会在 vTaskStartScheduler 创建 idle/timer 时耗尽、调度器返回卡死。
 * RAM 区 32KB，16KB 堆后总 RW+ZI 约 26KB，留有余量。
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
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/*========== 协程配置：Cortex-M0+ 项目不使用协程 ==========*/
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/*========== 断言 ==========*/
#define configASSERT( x )                       if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

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
#define INCLUDE_xTimerPendFunctionCall          1

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

/*========== CMSIS-RTOS V2 兼容配置 ==========*/
#define configENABLE_FPB                               0
#define configENABLE_MPU                               0
#define configENABLE_TRUSTZONE                         0
#define configRUN_FREERTOS_SECURE_ONLY                 1

#endif /* FREERTOS_CONFIG_H */
