cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

project(threadx
    LANGUAGES C ASM
)

# ---------------------------------------------------------------------------
# Validate required variables
# ---------------------------------------------------------------------------
if(NOT DEFINED THREADX_ARCH)
  message(FATAL_ERROR "THREADX_ARCH is not defined. "
        "Supported values: cortex_m4, cortex_m7")
endif()

set(_THREADX_SUPPORTED_ARCHS cortex_m3 cortex_m4 cortex_m7)
if(NOT THREADX_ARCH IN_LIST _THREADX_SUPPORTED_ARCHS)
  message(FATAL_ERROR "Unsupported THREADX_ARCH: '${THREADX_ARCH}'. "
        "Supported values: ${_THREADX_SUPPORTED_ARCHS}")
endif()

message(STATUS "ThreadX: building for arch '${THREADX_ARCH}'")

# ---------------------------------------------------------------------------
# Convenience path variables
# ---------------------------------------------------------------------------
set(_TX_COMMON_SRC  ${CMAKE_CURRENT_LIST_DIR}/common/src)
set(_TX_COMMON_INC  ${CMAKE_CURRENT_LIST_DIR}/common/inc)
set(_TX_PORT_DIR    ${CMAKE_CURRENT_LIST_DIR}/ports/${THREADX_ARCH})
set(_TX_PORT_SRC    ${_TX_PORT_DIR}/src)
set(_TX_PORT_INC    ${_TX_PORT_DIR}/inc)

# A place for the copied tx_user.h so it does not pollute the source tree
set(_TX_CUSTOM_INC  ${CMAKE_CURRENT_BINARY_DIR}/custom_inc)

# ---------------------------------------------------------------------------
# Static library target + alias
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} STATIC)
add_library("azrtos::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Common C sources
# Note: tx_misra.c provides weak stubs; the port's tx_misra.S is intentionally
#       excluded here to avoid duplicate-symbol linker errors when the C stubs
#       are sufficient.  Override by setting TX_SRC_OVERRIDES if needed.
# ---------------------------------------------------------------------------
target_sources(${PROJECT_NAME}
    PRIVATE
    # Initialization
    ${_TX_COMMON_SRC}/tx_initialize_high_level.c
    ${_TX_COMMON_SRC}/tx_initialize_kernel_enter.c
    ${_TX_COMMON_SRC}/tx_initialize_kernel_setup.c

    # Block pool
    ${_TX_COMMON_SRC}/tx_block_allocate.c
    ${_TX_COMMON_SRC}/tx_block_pool_cleanup.c
    ${_TX_COMMON_SRC}/tx_block_pool_create.c
    ${_TX_COMMON_SRC}/tx_block_pool_delete.c
    ${_TX_COMMON_SRC}/tx_block_pool_info_get.c
    ${_TX_COMMON_SRC}/tx_block_pool_initialize.c
    ${_TX_COMMON_SRC}/tx_block_pool_performance_info_get.c
    ${_TX_COMMON_SRC}/tx_block_pool_performance_system_info_get.c
    ${_TX_COMMON_SRC}/tx_block_pool_prioritize.c
    ${_TX_COMMON_SRC}/tx_block_release.c

    # Byte pool
    ${_TX_COMMON_SRC}/tx_byte_allocate.c
    ${_TX_COMMON_SRC}/tx_byte_pool_cleanup.c
    ${_TX_COMMON_SRC}/tx_byte_pool_create.c
    ${_TX_COMMON_SRC}/tx_byte_pool_delete.c
    ${_TX_COMMON_SRC}/tx_byte_pool_info_get.c
    ${_TX_COMMON_SRC}/tx_byte_pool_initialize.c
    ${_TX_COMMON_SRC}/tx_byte_pool_performance_info_get.c
    ${_TX_COMMON_SRC}/tx_byte_pool_performance_system_info_get.c
    ${_TX_COMMON_SRC}/tx_byte_pool_prioritize.c
    ${_TX_COMMON_SRC}/tx_byte_pool_search.c
    ${_TX_COMMON_SRC}/tx_byte_release.c

    # Event flags
    ${_TX_COMMON_SRC}/tx_event_flags_cleanup.c
    ${_TX_COMMON_SRC}/tx_event_flags_create.c
    ${_TX_COMMON_SRC}/tx_event_flags_delete.c
    ${_TX_COMMON_SRC}/tx_event_flags_get.c
    ${_TX_COMMON_SRC}/tx_event_flags_info_get.c
    ${_TX_COMMON_SRC}/tx_event_flags_initialize.c
    ${_TX_COMMON_SRC}/tx_event_flags_performance_info_get.c
    ${_TX_COMMON_SRC}/tx_event_flags_performance_system_info_get.c
    ${_TX_COMMON_SRC}/tx_event_flags_set.c
    ${_TX_COMMON_SRC}/tx_event_flags_set_notify.c

    # Mutex
    ${_TX_COMMON_SRC}/tx_mutex_cleanup.c
    ${_TX_COMMON_SRC}/tx_mutex_create.c
    ${_TX_COMMON_SRC}/tx_mutex_delete.c
    ${_TX_COMMON_SRC}/tx_mutex_get.c
    ${_TX_COMMON_SRC}/tx_mutex_info_get.c
    ${_TX_COMMON_SRC}/tx_mutex_initialize.c
    ${_TX_COMMON_SRC}/tx_mutex_performance_info_get.c
    ${_TX_COMMON_SRC}/tx_mutex_performance_system_info_get.c
    ${_TX_COMMON_SRC}/tx_mutex_prioritize.c
    ${_TX_COMMON_SRC}/tx_mutex_priority_change.c
    ${_TX_COMMON_SRC}/tx_mutex_put.c

    # Queue
    ${_TX_COMMON_SRC}/tx_queue_cleanup.c
    ${_TX_COMMON_SRC}/tx_queue_create.c
    ${_TX_COMMON_SRC}/tx_queue_delete.c
    ${_TX_COMMON_SRC}/tx_queue_flush.c
    ${_TX_COMMON_SRC}/tx_queue_front_send.c
    ${_TX_COMMON_SRC}/tx_queue_info_get.c
    ${_TX_COMMON_SRC}/tx_queue_initialize.c
    ${_TX_COMMON_SRC}/tx_queue_performance_info_get.c
    ${_TX_COMMON_SRC}/tx_queue_performance_system_info_get.c
    ${_TX_COMMON_SRC}/tx_queue_prioritize.c
    ${_TX_COMMON_SRC}/tx_queue_receive.c
    ${_TX_COMMON_SRC}/tx_queue_send.c
    ${_TX_COMMON_SRC}/tx_queue_send_notify.c

    # Semaphore
    ${_TX_COMMON_SRC}/tx_semaphore_ceiling_put.c
    ${_TX_COMMON_SRC}/tx_semaphore_cleanup.c
    ${_TX_COMMON_SRC}/tx_semaphore_create.c
    ${_TX_COMMON_SRC}/tx_semaphore_delete.c
    ${_TX_COMMON_SRC}/tx_semaphore_get.c
    ${_TX_COMMON_SRC}/tx_semaphore_info_get.c
    ${_TX_COMMON_SRC}/tx_semaphore_initialize.c
    ${_TX_COMMON_SRC}/tx_semaphore_performance_info_get.c
    ${_TX_COMMON_SRC}/tx_semaphore_performance_system_info_get.c
    ${_TX_COMMON_SRC}/tx_semaphore_prioritize.c
    ${_TX_COMMON_SRC}/tx_semaphore_put.c
    ${_TX_COMMON_SRC}/tx_semaphore_put_notify.c

    # Thread
    ${_TX_COMMON_SRC}/tx_thread_create.c
    ${_TX_COMMON_SRC}/tx_thread_delete.c
    ${_TX_COMMON_SRC}/tx_thread_entry_exit_notify.c
    ${_TX_COMMON_SRC}/tx_thread_identify.c
    ${_TX_COMMON_SRC}/tx_thread_info_get.c
    ${_TX_COMMON_SRC}/tx_thread_initialize.c
    ${_TX_COMMON_SRC}/tx_thread_performance_info_get.c
    ${_TX_COMMON_SRC}/tx_thread_performance_system_info_get.c
    ${_TX_COMMON_SRC}/tx_thread_preemption_change.c
    ${_TX_COMMON_SRC}/tx_thread_priority_change.c
    ${_TX_COMMON_SRC}/tx_thread_relinquish.c
    ${_TX_COMMON_SRC}/tx_thread_reset.c
    ${_TX_COMMON_SRC}/tx_thread_resume.c
    ${_TX_COMMON_SRC}/tx_thread_shell_entry.c
    ${_TX_COMMON_SRC}/tx_thread_sleep.c
    ${_TX_COMMON_SRC}/tx_thread_stack_analyze.c
    ${_TX_COMMON_SRC}/tx_thread_stack_error_handler.c
    ${_TX_COMMON_SRC}/tx_thread_stack_error_notify.c
    ${_TX_COMMON_SRC}/tx_thread_suspend.c
    ${_TX_COMMON_SRC}/tx_thread_system_preempt_check.c
    ${_TX_COMMON_SRC}/tx_thread_system_resume.c
    ${_TX_COMMON_SRC}/tx_thread_system_suspend.c
    ${_TX_COMMON_SRC}/tx_thread_terminate.c
    ${_TX_COMMON_SRC}/tx_thread_time_slice.c
    ${_TX_COMMON_SRC}/tx_thread_time_slice_change.c
    ${_TX_COMMON_SRC}/tx_thread_timeout.c
    ${_TX_COMMON_SRC}/tx_thread_wait_abort.c

    # Time
    ${_TX_COMMON_SRC}/tx_time_get.c
    ${_TX_COMMON_SRC}/tx_time_set.c

    # Timer
    ${_TX_COMMON_SRC}/tx_timer_activate.c
    ${_TX_COMMON_SRC}/tx_timer_change.c
    ${_TX_COMMON_SRC}/tx_timer_create.c
    ${_TX_COMMON_SRC}/tx_timer_deactivate.c
    ${_TX_COMMON_SRC}/tx_timer_delete.c
    ${_TX_COMMON_SRC}/tx_timer_expiration_process.c
    ${_TX_COMMON_SRC}/tx_timer_info_get.c
    ${_TX_COMMON_SRC}/tx_timer_initialize.c
    ${_TX_COMMON_SRC}/tx_timer_performance_info_get.c
    ${_TX_COMMON_SRC}/tx_timer_performance_system_info_get.c
    ${_TX_COMMON_SRC}/tx_timer_system_activate.c
    ${_TX_COMMON_SRC}/tx_timer_system_deactivate.c
    ${_TX_COMMON_SRC}/tx_timer_thread_entry.c

    # Trace
    ${_TX_COMMON_SRC}/tx_trace_buffer_full_notify.c
    ${_TX_COMMON_SRC}/tx_trace_disable.c
    ${_TX_COMMON_SRC}/tx_trace_enable.c
    ${_TX_COMMON_SRC}/tx_trace_event_filter.c
    ${_TX_COMMON_SRC}/tx_trace_event_unfilter.c
    ${_TX_COMMON_SRC}/tx_trace_initialize.c
    ${_TX_COMMON_SRC}/tx_trace_interrupt_control.c
    ${_TX_COMMON_SRC}/tx_trace_isr_enter_insert.c
    ${_TX_COMMON_SRC}/tx_trace_isr_exit_insert.c
    ${_TX_COMMON_SRC}/tx_trace_object_register.c
    ${_TX_COMMON_SRC}/tx_trace_object_unregister.c
    ${_TX_COMMON_SRC}/tx_trace_user_event_insert.c

    # Error-checking wrappers (txe_*)
    ${_TX_COMMON_SRC}/txe_block_allocate.c
    ${_TX_COMMON_SRC}/txe_block_pool_create.c
    ${_TX_COMMON_SRC}/txe_block_pool_delete.c
    ${_TX_COMMON_SRC}/txe_block_pool_info_get.c
    ${_TX_COMMON_SRC}/txe_block_pool_prioritize.c
    ${_TX_COMMON_SRC}/txe_block_release.c
    ${_TX_COMMON_SRC}/txe_byte_allocate.c
    ${_TX_COMMON_SRC}/txe_byte_pool_create.c
    ${_TX_COMMON_SRC}/txe_byte_pool_delete.c
    ${_TX_COMMON_SRC}/txe_byte_pool_info_get.c
    ${_TX_COMMON_SRC}/txe_byte_pool_prioritize.c
    ${_TX_COMMON_SRC}/txe_byte_release.c
    ${_TX_COMMON_SRC}/txe_event_flags_create.c
    ${_TX_COMMON_SRC}/txe_event_flags_delete.c
    ${_TX_COMMON_SRC}/txe_event_flags_get.c
    ${_TX_COMMON_SRC}/txe_event_flags_info_get.c
    ${_TX_COMMON_SRC}/txe_event_flags_set.c
    ${_TX_COMMON_SRC}/txe_event_flags_set_notify.c
    ${_TX_COMMON_SRC}/txe_mutex_create.c
    ${_TX_COMMON_SRC}/txe_mutex_delete.c
    ${_TX_COMMON_SRC}/txe_mutex_get.c
    ${_TX_COMMON_SRC}/txe_mutex_info_get.c
    ${_TX_COMMON_SRC}/txe_mutex_prioritize.c
    ${_TX_COMMON_SRC}/txe_mutex_put.c
    ${_TX_COMMON_SRC}/txe_queue_create.c
    ${_TX_COMMON_SRC}/txe_queue_delete.c
    ${_TX_COMMON_SRC}/txe_queue_flush.c
    ${_TX_COMMON_SRC}/txe_queue_front_send.c
    ${_TX_COMMON_SRC}/txe_queue_info_get.c
    ${_TX_COMMON_SRC}/txe_queue_prioritize.c
    ${_TX_COMMON_SRC}/txe_queue_receive.c
    ${_TX_COMMON_SRC}/txe_queue_send.c
    ${_TX_COMMON_SRC}/txe_queue_send_notify.c
    ${_TX_COMMON_SRC}/txe_semaphore_ceiling_put.c
    ${_TX_COMMON_SRC}/txe_semaphore_create.c
    ${_TX_COMMON_SRC}/txe_semaphore_delete.c
    ${_TX_COMMON_SRC}/txe_semaphore_get.c
    ${_TX_COMMON_SRC}/txe_semaphore_info_get.c
    ${_TX_COMMON_SRC}/txe_semaphore_prioritize.c
    ${_TX_COMMON_SRC}/txe_semaphore_put.c
    ${_TX_COMMON_SRC}/txe_semaphore_put_notify.c
    ${_TX_COMMON_SRC}/txe_thread_create.c
    ${_TX_COMMON_SRC}/txe_thread_delete.c
    ${_TX_COMMON_SRC}/txe_thread_entry_exit_notify.c
    ${_TX_COMMON_SRC}/txe_thread_info_get.c
    ${_TX_COMMON_SRC}/txe_thread_preemption_change.c
    ${_TX_COMMON_SRC}/txe_thread_priority_change.c
    ${_TX_COMMON_SRC}/txe_thread_relinquish.c
    ${_TX_COMMON_SRC}/txe_thread_reset.c
    ${_TX_COMMON_SRC}/txe_thread_resume.c
    ${_TX_COMMON_SRC}/txe_thread_suspend.c
    ${_TX_COMMON_SRC}/txe_thread_terminate.c
    ${_TX_COMMON_SRC}/txe_thread_time_slice_change.c
    ${_TX_COMMON_SRC}/txe_thread_wait_abort.c
    ${_TX_COMMON_SRC}/txe_timer_activate.c
    ${_TX_COMMON_SRC}/txe_timer_change.c
    ${_TX_COMMON_SRC}/txe_timer_create.c
    ${_TX_COMMON_SRC}/txe_timer_deactivate.c
    ${_TX_COMMON_SRC}/txe_timer_delete.c
    ${_TX_COMMON_SRC}/txe_timer_info_get.c
)

# ---------------------------------------------------------------------------
# Port-specific ASM sources (selected by THREADX_ARCH)
#
# tx_misra.S            is intentionally omitted — common/src/tx_misra.c
#                       already provides (weak) implementations; include
#                       tx_misra.S manually from your top-level project if
#                       you need the optimised ASM stubs.
#
# tx_initialize_low_level.S lives in the port root (_TX_PORT_DIR) and has
#                       been adapted for this board, so it is included here.
# ---------------------------------------------------------------------------
target_sources(${PROJECT_NAME}
    PRIVATE
    ${_TX_PORT_DIR}/tx_initialize_low_level.S
    ${_TX_PORT_SRC}/tx_thread_context_restore.S
    ${_TX_PORT_SRC}/tx_thread_context_save.S
    ${_TX_PORT_SRC}/tx_thread_interrupt_control.S
    ${_TX_PORT_SRC}/tx_thread_interrupt_disable.S
    ${_TX_PORT_SRC}/tx_thread_interrupt_restore.S
    ${_TX_PORT_SRC}/tx_thread_schedule.S
    ${_TX_PORT_SRC}/tx_thread_stack_build.S
    ${_TX_PORT_SRC}/tx_thread_system_return.S
    ${_TX_PORT_SRC}/tx_timer_interrupt.S
)

# ---------------------------------------------------------------------------
# Public include directories
#   - common/inc    : tx_api.h and friends
#   - port/inc      : tx_port.h for the selected arch
#   - custom_inc    : tx_user.h (copied from project root at configure time)
# ---------------------------------------------------------------------------
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/tx_user.h
    ${_TX_CUSTOM_INC}/tx_user.h
    COPYONLY
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${_TX_COMMON_INC}
        ${_TX_PORT_INC}
        ${_TX_CUSTOM_INC}
)

# ---------------------------------------------------------------------------
# Compile definitions propagated to consumers
# ---------------------------------------------------------------------------
target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        TX_INCLUDE_USER_DEFINE_FILE
        $<$<STREQUAL:${THREADX_ARCH},cortex_m4>:TX_ENABLE_FPU_SUPPORT>
        $<$<STREQUAL:${THREADX_ARCH},cortex_m7>:TX_ENABLE_FPU_SUPPORT>
)

# ---------------------------------------------------------------------------
# Maximum performance compiler optimization flags
# ---------------------------------------------------------------------------
target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>,$<C_COMPILER_ID:AppleClang>>:
            -O3
            -ffast-math
            -fno-math-errno
            -funroll-loops
            -fomit-frame-pointer
        >
)
