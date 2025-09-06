/**
 * SPDX-License-Identifier: Apache-2.0 
 * Copyright (c) Bao Project and Contributors. All rights reserved
 */

#ifndef __MEM_THROT_H__
#define __MEM_THROT_H__

#include <timer.h>
#include <events.h>
#include <bitmap.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct mem_throt_info {
    bool is_initialized;
    bool throttled;             
    size_t counter_id;
    size_t period_us;
    size_t period_counts;
    size_t budget; 
    int64_t budget_left;
    size_t assign_ratio;
    size_t c_vm;
} mem_throt_t;

extern const size_t DT[100];

/* Global counters: the total bus accesses and four critical counters */
extern volatile size_t total_bus_access;
extern volatile size_t c1_bus_access;
extern volatile size_t c2_bus_access;
extern volatile size_t c3_bus_access;
extern volatile size_t c4_bus_access;

extern size_t global_num_ticket_hypervisor;

void mem_throt_config(size_t period_us, size_t vm_budget, size_t* cpu_ratio, size_t asil_criticality);
void mem_throt_init();
// void mem_throt_period_timer_callback_c(irqid_t);
void mem_throt_period_timer_callback_nc(irqid_t);

/* Callback for when the PMU detects an event overflow */
void mem_throt_event_overflow_callback(irqid_t); 
void mem_throt_process_overflow(void);

void mem_throt_timer_init(irq_handler_t handler);
void mem_throt_events_init(events_enum event, unsigned long budget, irq_handler_t handler);
void mem_throt_budget_change(uint64_t budget);
void perf_monitor_setup_event_counters(size_t counter_id);

/* 
 * Optimized atomic operations using ARMv8 instructions.
 * Uses relaxed memory ordering for better pipeline performance.
 */

/* Relaxed version without acquire semantics */
static inline uint64_t atomic_load64_acquire(const volatile uint64_t *addr)
{
    uint64_t val;
    __asm__ volatile (
        "ldar %0, [%1]"    /* Acquire load for proper ordering */
        : "=r" (val)
        : "r" (addr)
        : "memory"
    );
    return val;
}

/* Proper atomic store with release semantics */
static inline void atomic_store64_release(volatile uint64_t *addr, uint64_t val)
{
    __asm__ volatile (
        "stlr %0, [%1]"    /* Release store ensures prior accesses complete first */
        :
        : "r" (val), "r" (addr)
        : "memory"
    );
}

/* Relaxed fetch-add with better pipelining */
static inline uint64_t atomic_fetch_add64(volatile uint64_t *ptr, uint64_t add)
{
    uint64_t old, new_val;
    uint32_t res;
    
    __asm__ volatile (
        "1: ldxr    %0, [%2]       \n"  /* Use ldxr instead of ldaxr */
        "   add     %1, %0, %4     \n"
        "   stxr    %w3, %1, [%2]  \n"  /* Use stxr instead of stlxr */
        "   cbnz    %w3, 1b        \n"
        : "=&r" (old), "=&r" (new_val), "+r" (ptr), "=&r" (res)
        : "r" (add)
    );
    return old;
}

/* Relaxed exchange for better pipelining */
static inline uint64_t atomic_exchange64(volatile uint64_t *addr, uint64_t new_val)
{
    uint64_t old;
    uint32_t res;
    
    __asm__ volatile (
        "1: ldxr    %0, [%1]       \n"  /* Use ldxr instead of ldaxr */
        "   stxr    %w2, %3, [%1]  \n"  /* Use stxr instead of stlxr */
        "   cbnz    %w2, 1b        \n"
        : "=&r" (old), "+r" (addr), "=&r" (res)
        : "r" (new_val)
    );
    return old;
}

#endif /* __MEM_THROT_H__ */
