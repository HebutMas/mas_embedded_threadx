/***************************************************************************
 * Copyright (c) 2024 Microsoft Corporation
 * Copyright (c) 2026-present Eclipse ThreadX contributors
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 **************************************************************************/

/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** ThreadX Component                                                     */
/**                                                                       */
/**   User Specific                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/
/*                                                                        */
/*  PORT SPECIFIC C INFORMATION                            RELEASE        */
/*                                                                        */
/*    tx_user.h                                           PORTABLE C      */
/*                                                           6.3.0        */
/*                                                                        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Microsoft Corporation                             */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains user defines for configuring ThreadX in specific */
/*    ways. This file will have an effect only if the application and     */
/*    ThreadX library are built with TX_INCLUDE_USER_DEFINE_FILE defined. */
/*    Note that all the defines in this file may also be made on the      */
/*    command line when building ThreadX library and application objects. */
/*                                                                        */
/**************************************************************************/

#ifndef TX_USER_H
#define TX_USER_H

/* 为 ThreadX 端口定义各种编译选项。应用程序可通过注释/取消注释这些条件编译宏来修改配置，
   也可通过编译器的 -D 等效选项传入这些宏定义。

   若追求最大运行速度，应定义以下宏：

        TX_MAX_PRIORITIES                       32
        TX_DISABLE_PREEMPTION_THRESHOLD
        TX_DISABLE_REDUNDANT_CLEARING
        TX_DISABLE_NOTIFY_CALLBACKS
        TX_NOT_INTERRUPTABLE
        TX_TIMER_PROCESS_IN_ISR
        TX_REACTIVATE_INLINE
        TX_DISABLE_STACK_FILLING
        TX_INLINE_THREAD_RESUME_SUSPEND

   若追求最小代码体积，应定义以下宏：

        TX_MAX_PRIORITIES                       32
        TX_DISABLE_PREEMPTION_THRESHOLD
        TX_DISABLE_REDUNDANT_CLEARING
        TX_DISABLE_NOTIFY_CALLBACKS
        TX_NO_FILEX_POINTER
        TX_NOT_INTERRUPTABLE
        TX_TIMER_PROCESS_IN_ISR

   当然，这些宏中的许多会降低功能完整性，或改变系统行为，其取舍需根据实际场景评估。
   例如，TX_TIMER_PROCESS_IN_ISR 可使代码更快、体积更小，但会增加中断服务程序（ISR）中的处理量；
   此外，定时器中部分可使用的服务在 ISR 中不可用，启用此选项后调用这些服务会返回错误。
   需根据具体应用场景判断是否可接受此行为。 */

/* 覆盖 tx_port.h 中已分配默认值的各种选项。有关每个选项的详细说明，也可参考 tx_port.h。 */

#define TX_MAX_PRIORITIES         32
/*
#define TX_MINIMUM_STACK           256     // 需用户自定义值
#define TX_THREAD_USER_EXTENSION   ? ? ? ? // 需用户自定义值
#define TX_TIMER_THREAD_STACK_SIZE 1024    // 需用户自定义值
#define TX_TIMER_THREAD_PRIORITY   (TX_MAX_PRIORITIES - 1)
*/

/* 定义队列中单个消息的最大尺寸。默认值为 TX_ULONG_16，
   新值必须是 ULONG 类型的倍数。 */

/*
#define TX_QUEUE_MESSAGE_MAX_SIZE              TX_ULONG_16
*/

/* 为其他中间件组件定义通用的定时器滴答基准值。默认值为 10 毫秒（即 100 个滴答，定义在 tx_api.h 中），
   但可通过 tx_port.h 中的端口特定版本或在此处重定义。
   注意：实际硬件定时器的配置值可能需要修改（通常在 tx_initialize_low_level 函数中）。 */

#define TX_TIMER_TICKS_PER_SECOND (1000UL)

/* 决定线程控制块中是否保留 FileX 指针。
   默认情况下，为了向下兼容/遗留支持，该指针会保留。
   使用 FileX 的应用程序也必须保留此指针。
   定义此宏可节省线程控制块的内存空间。
*/

#define TX_NO_FILEX_POINTER

/* 决定定时器到期处理（应用层定时器、超时、tx_thread_sleep 调用）
   应在系统定时器线程中执行，还是直接在定时器 ISR 中执行。
   默认使用定时器线程处理。当定义以下宏时，定时器到期处理
   直接在定时器 ISR 中完成，从而省去定时器线程的控制块、栈空间
   以及激活线程的上下文切换开销。 */

#define TX_TIMER_PROCESS_IN_ISR

/* 决定在定时器到期处理中是否使用内联方式重新激活定时器。
   默认禁用此功能，使用函数调用方式。当定义以下宏时，
   重新激活操作以内联方式执行，可提升定时器处理速度，但会略微增加代码体积。 */

#define TX_REACTIVATE_INLINE

/* 决定是否启用栈填充功能。默认情况下，ThreadX 启用栈填充，
   会在每个线程栈的每个字节中填充 0xEF 模式。此功能供支持 ThreadX 的调试器
   以及 ThreadX 运行时栈检查功能使用。 */

#define TX_DISABLE_STACK_FILLING

/* 决定是否启用栈检查功能。默认情况下，ThreadX 禁用栈检查。
   当定义以下宏时，ThreadX 线程栈检查功能被启用。若启用栈检查
   （定义 TX_ENABLE_STACK_CHECKING），则 TX_DISABLE_STACK_FILLING 宏会被否定，
   从而强制启用栈填充（栈检查逻辑依赖栈填充功能）。 */


#define TX_ENABLE_STACK_CHECKING

/* 决定栈填充是否使用随机数。默认情况下，ThreadX 使用固定模式填充栈。
   当定义以下宏时，ThreadX 使用随机数填充栈。此功能仅在定义了
   TX_ENABLE_STACK_CHECKING 时生效。 */

/*
#define TX_ENABLE_RANDOM_NUMBER_STACK_FILLING
*/

/* 决定是否禁用抢占阈值功能。默认情况下，抢占阈值功能启用。
   若应用程序不使用抢占阈值，可禁用此功能以减小代码体积并提升性能。 */

#define TX_DISABLE_PREEMPTION_THRESHOLD

/* 决定是否清除 ThreadX 全局变量。若编译器启动代码在 ThreadX 运行前已清除
   .bss 段，则可定义此宏以消除 ThreadX 全局变量的冗余清除操作。 */

/*
#define TX_DISABLE_REDUNDANT_CLEARING
*/

/* 决定是否禁用定时器处理功能。此选项可在不需要定时器时移除相关处理逻辑。
   用户还需注释掉对 tx_timer_interrupt 的调用（通常在 tx_initialize_low_level
   的汇编代码中）。注意：若使用 TX_NO_TIMER，必须同时定义 TX_TIMER_PROCESS_IN_ISR，
   且需从 ThreadX 库中移除 tx_timer_initialize 函数。 */

/*
#define TX_NO_TIMER
#ifndef TX_TIMER_PROCESS_IN_ISR
#define TX_TIMER_PROCESS_IN_ISR
#endif
*/

/* 决定是否禁用通知回调功能。默认情况下，通知回调功能启用。
   若应用程序不使用通知回调，可禁用此功能以减小代码体积并提升性能。 */

#define TX_DISABLE_NOTIFY_CALLBACKS

/* 决定 tx_thread_resume 和 tx_thread_suspend 服务的内部代码是否以内联方式实现。
   此方式会增加镜像文件体积，但可提升线程挂起/恢复服务的性能。 */

#define TX_INLINE_THREAD_RESUME_SUSPEND

/* 决定 ThreadX 内部代码是否不可中断。此方式可减小代码体积并降低处理开销，
   但会增加中断锁定时间。 */

/* #define TX_NOT_INTERRUPTABLE */

/* 决定是否启用跟踪事件日志功能。此功能会略微增加代码体积和运行开销，
   但可生成系统跟踪信息，供 TraceX 工具查看。 */

/*
#define TX_ENABLE_EVENT_TRACE
*/

/* 决定应用程序是否需要收集块池性能信息。定义以下宏后，
   ThreadX 会收集各类块池性能数据。 */

/*
#define TX_BLOCK_POOL_ENABLE_PERFORMANCE_INFO
*/

/* 决定应用程序是否需要收集字节池性能信息。定义以下宏后，
   ThreadX 会收集各类字节池性能数据。 */

/*
#define TX_BYTE_POOL_ENABLE_PERFORMANCE_INFO
*/

/* 决定应用程序是否需要收集事件标志组性能信息。定义以下宏后，
   ThreadX 会收集各类事件标志组性能数据。 */

/*
#define TX_EVENT_FLAGS_ENABLE_PERFORMANCE_INFO
*/

/* 决定应用程序是否需要收集互斥锁性能信息。定义以下宏后，
   ThreadX 会收集各类互斥锁性能数据。 */

/*
#define TX_MUTEX_ENABLE_PERFORMANCE_INFO
*/

/* 决定应用程序是否需要收集队列性能信息。定义以下宏后，
   ThreadX 会收集各类队列性能数据。 */

/*
#define TX_QUEUE_ENABLE_PERFORMANCE_INFO
*/

/* 决定应用程序是否需要收集信号量性能信息。定义以下宏后，
   ThreadX 会收集各类信号量性能数据。 */

/*
#define TX_SEMAPHORE_ENABLE_PERFORMANCE_INFO
*/

/* 决定应用程序是否需要收集线程性能信息。定义以下宏后，
   ThreadX 会收集各类线程性能数据。 */

/*
#define TX_THREAD_ENABLE_PERFORMANCE_INFO
*/

/* 决定应用程序是否需要收集定时器性能信息。定义以下宏后，
   ThreadX 会收集各类定时器性能数据。 */

/*
#define TX_TIMER_ENABLE_PERFORMANCE_INFO
*/

/*  字节池多块搜索的覆盖选项。 */

/*
#define TX_BYTE_POOL_MULTIPLE_BLOCK_SEARCH    20
*/

/*  字节池搜索延迟的覆盖选项（用于避免抖动）。 */

/*
#define TX_BYTE_POOL_DELAY_VALUE              3
*/

#endif
