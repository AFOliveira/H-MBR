/**
 * SPDX-License-Identifier: Apache-2.0 
 * Copyright (c) Bao Project and Contributors. All rights reserved
 */

#include <mem_throt.h>
#include <cpu.h>
#include <vm.h>
#include <spinlock.h>


#define LOWER_BOUND 400ULL
#define UPPER_BOUND 100000ULL
#define BUCKETS 10
#define RANGE (UPPER_BOUND - LOWER_BOUND)  // 99600
/* Precompute MULTIPLIER = (9 << 32) / RANGE to map [LOWER_BOUND+1, UPPER_BOUND-1] uniformly
 * into indices 0..9. */
#define MULTIPLIER 388003ULL  // Approximately

spinlock_t lock;

const size_t DT[10] = {
    100000,         // index 0
    50000,
    25000,
    10000,
    5000,
    1000,
    750,
    500,
    250,
    100
};

/* Define the global counters. */
volatile size_t total_bus_access = 0;
volatile size_t qm_bus_access = 0;  /* Added QM bus access counter */
volatile size_t c1_bus_access = 0;
volatile size_t c2_bus_access = 0;
volatile size_t c3_bus_access = 0;
volatile size_t c4_bus_access = 0;

/* Array of pointers to the critical counters for fast index‐based access. */
static volatile size_t * const crit_counters[5] = {
    &qm_bus_access, &c1_bus_access, &c2_bus_access, &c3_bus_access, &c4_bus_access
};

/*
 * Helper function: updates a critical counter atomically.
 * It performs an atomic exchange on the chosen counter (thereby reading and clearing it),
 * computes the difference between the new and old value, and adjusts the global total accordingly.
 */
static inline void update_critical_counter(volatile size_t *counter, uint64_t new_val) {
    uint64_t old_val = atomic_exchange64(counter, new_val);
    int64_t delta = new_val - old_val;
    atomic_fetch_add64(&total_bus_access, delta);
}

/* Selects the appropriate DT value based on 'val'. */
static inline size_t process_event(uint64_t val) {
    /* Generate branchless masks:
     * When the condition is true, the expression (val <= LOWER_BOUND) returns 1, and its negative becomes all 1s;
     * otherwise, it becomes 0.
     */
    uint64_t low_mask  = -(uint64_t)(val <= LOWER_BOUND);
    uint64_t high_mask = -(uint64_t)(val >= UPPER_BOUND);
    uint64_t normal_mask = ~(low_mask | high_mask);

    /* For in-range values, compute the normalized difference.
     * Out-of-range values will have normal_mask == 0.
     */
    uint64_t normalized = (val - LOWER_BOUND) & normal_mask;
    size_t computed_index = (normalized * MULTIPLIER) >> 32;

    /* Combine the cases:
     * - If low_mask is active, force index 0.
     * - If high_mask is active, force index BUCKETS - 1.
     * - Otherwise, use computed_index.
     */
    size_t index = (low_mask  & 0) |
                   (high_mask & (BUCKETS - 1)) |
                   (normal_mask & computed_index);
    return DT[index];
}


#define QM     0
#define ASIL_A 1
#define ASIL_B 2
#define ASIL_C 3
#define ASIL_D 4

/* Q8 fixed-point scaling factors:
 * Index 0: QM     = 0.25
 * Index 1: ASIL_A = 0.5 
 * Index 2: ASIL_B = 0.75 
 * Index 3: ASIL_C = 1 
 * Index 4: ASIL_D (not used here)
 */
static const uint32_t scaling[5] = { 64, 128, 192, 256 };

void mem_throt_period_timer_callback_nc(irqid_t int_id) {
    timer_disable();

    /* Modified to include QM VMs */
        uint64_t new_val = events_get_cntr_value(cpu()->vcpu->vm->mem_throt.counter_id);
        /* For QM use index 0, for others, use their numeric value as index */
        size_t idx = cpu()->vcpu->vm->mem_throt.c_vm;
        update_critical_counter(crit_counters[idx], new_val);

        pmu_reset_event_counters();
        events_clear_cntr_ovs(cpu()->vcpu->vm->mem_throt.counter_id);
        events_arch_cntr_enable(cpu()->vcpu->vm->mem_throt.counter_id);
        events_cntr_set(cpu()->vcpu->vm->mem_throt.counter_id, 0);
   

    if (cpu()->vcpu->vm->mem_throt.c_vm != ASIL_D) {
        events_cntr_disable(cpu()->vcpu->vm->mem_throt.counter_id);

        if (cpu()->vcpu->mem_throt.throttled) {
            events_cntr_irq_enable(cpu()->vcpu->vm->mem_throt.counter_id);
            cpu()->vcpu->mem_throt.throttled = false;
        }
        events_cntr_enable(cpu()->vcpu->vm->mem_throt.counter_id);

        if (cpu()->vcpu->vm->master)
            cpu()->vcpu->vm->mem_throt.budget_left = cpu()->vcpu->vm->mem_throt.budget;

        uint64_t total_val = atomic_load64_acquire(&total_bus_access);
        size_t new_budget = process_event(total_val);


        uint32_t crit = cpu()->vcpu->vm->mem_throt.c_vm;
        new_budget = (new_budget * scaling[crit]) >> 8;  // Multiply then shift to divide by 256

#ifdef DEBUG
        console_printk(" %d, %d\n", new_budget, total_val);
#endif
        events_cntr_set(cpu()->vcpu->vm->mem_throt.counter_id, new_budget);
    }

    timer_reschedule_interrupt(cpu()->vcpu->vm->mem_throt.period_counts);
    timer_enable();
}


void mem_throt_event_overflow_callback(irqid_t int_id) {
    events_clear_cntr_ovs(cpu()->vcpu->vm->mem_throt.counter_id);
    events_cntr_disable(cpu()->vcpu->vm->mem_throt.counter_id);
    events_cntr_irq_disable(cpu()->vcpu->vm->mem_throt.counter_id);

    cpu()->vcpu->mem_throt.throttled = true;  
    cpu_standby();
}

void mem_throt_timer_init(irq_handler_t handler) {
    timer_define_irq_callback(handler);
    cpu()->vcpu->vm->mem_throt.period_counts = timer_init(cpu()->vcpu->vm->mem_throt.period_us);
}

void mem_throt_events_init(events_enum event, unsigned long budget, irq_handler_t handler) {
    if ((cpu()->vcpu->vm->mem_throt.counter_id = events_cntr_alloc()) == ERROR_NO_MORE_EVENT_COUNTERS) {
        ERROR("No more event counters!");
    }

    events_set_evtyper(cpu()->vcpu->vm->mem_throt.counter_id, event);
    events_cntr_set(cpu()->vcpu->vm->mem_throt.counter_id, budget);
    events_cntr_set_irq_callback(handler, cpu()->vcpu->vm->mem_throt.counter_id);
    events_clear_cntr_ovs(cpu()->vcpu->vm->mem_throt.counter_id);
    events_interrupt_enable(cpu()->id);
    events_cntr_irq_enable(cpu()->vcpu->vm->mem_throt.counter_id);
    events_enable();
    events_cntr_enable(cpu()->vcpu->vm->mem_throt.counter_id);
}

inline void mem_throt_budget_change(size_t budget) {
    events_cntr_set(cpu()->vcpu->vm->mem_throt.counter_id, cpu()->vcpu->vm->mem_throt.budget);
    events_cntr_enable(cpu()->vcpu->vm->mem_throt.counter_id);
    events_cntr_irq_enable(cpu()->vcpu->vm->mem_throt.counter_id);
}

void perf_monitor_setup_event_counters(size_t counter_id) {
    events_cntr_set(counter_id, 0);
    events_enable();
    events_set_evtyper(counter_id, bus_access);
    events_clear_cntr_ovs(counter_id);
    events_cntr_enable(counter_id);
}

void mem_throt_config(size_t period_us, size_t vm_budget, size_t* cpu_ratio, size_t asil_criticality) {
    cpu()->vcpu->vm->mem_throt.c_vm = 0;
    if (period_us == 0) return;

    if (cpu()->id == cpu()->vcpu->vm->master) {   
        vm_budget = vm_budget / cpu()->vcpu->vm->cpu_num;
        cpu()->vcpu->vm->mem_throt.throttled = false;
        cpu()->vcpu->vm->mem_throt.period_us = period_us;
        cpu()->vcpu->vm->mem_throt.budget = vm_budget * cpu()->vcpu->vm->cpu_num;
        cpu()->vcpu->vm->mem_throt.budget_left = cpu()->vcpu->vm->mem_throt.budget;
        cpu()->vcpu->vm->mem_throt.is_initialized = true;
    }
    cpu()->vcpu->vm->mem_throt.c_vm = asil_criticality;
    console_printk("Criticality: %d\n", cpu()->vcpu->vm->mem_throt.c_vm);

    while(cpu()->vcpu->vm->mem_throt.is_initialized != true);

    spin_lock(&lock);

    if (cpu_ratio[cpu()->vcpu->id] == 0) {
        cpu_ratio[cpu()->vcpu->id] = cpu()->vcpu->vm->mem_throt.budget / cpu()->vcpu->vm->cpu_num;
    }
    
    cpu()->vcpu->mem_throt.assign_ratio = cpu_ratio[cpu()->vcpu->id]; 
    cpu()->vcpu->mem_throt.budget = cpu()->vcpu->vm->mem_throt.budget * (cpu()->vcpu->mem_throt.assign_ratio) / 100;
    cpu()->vcpu->vm->mem_throt.budget -= cpu()->vcpu->mem_throt.budget;
    cpu()->vcpu->vm->mem_throt.budget_left -= cpu()->vcpu->mem_throt.budget;
    cpu()->vcpu->mem_throt.assign_ratio += cpu()->vcpu->mem_throt.assign_ratio;

    spin_unlock(&lock);

    if (cpu()->vcpu->mem_throt.assign_ratio > 100) {
        ERROR("The sum of the ratios is greater than 100");
    }
}

void mem_throt_init() {
    if (cpu()->vcpu->vm->mem_throt.is_initialized != true) return;

    if (cpu()->vcpu->vm->mem_throt.c_vm != ASIL_D) {
        mem_throt_events_init(bus_access, cpu()->vcpu->mem_throt.budget, mem_throt_event_overflow_callback);
    } else {
        perf_monitor_setup_event_counters(cpu()->vcpu->vm->mem_throt.counter_id);
    }
    mem_throt_timer_init(mem_throt_period_timer_callback_nc);
}