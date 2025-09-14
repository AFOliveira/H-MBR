/**
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) Bao Project and Contributors. All rights reserved.
 * 
 * PMU instrumentation for MBR overhead measurement
 */

#ifndef PMU_MBR_H
#define PMU_MBR_H

#include <types.h>
#include <arch/pmu.h>

/* PMU cycle counter for ARMv8 */
#define PMCCNTR_EL0         S3_3_C9_C13_0
#define PMCNTENSET_EL0      S3_3_C9_C12_1
#define PMCR_EL0            S3_3_C9_C12_0

/* MBR profiling regions */
typedef enum {
    MBR_REGION_ENTRY,           // Entry to MBR handler
    MBR_REGION_CRITICALITY,     // ASIL criticality computation
    MBR_REGION_UPDATE_COUNTER,  // Counter updates
    MBR_REGION_DT_LOOKUP,       // Delay table lookup
    MBR_REGION_DELAY,           // Actual delay execution
    MBR_REGION_EXIT,            // Exit from MBR handler
    MBR_REGION_COUNT
} mbr_region_t;

/* Per-CPU MBR profiling data */
struct mbr_profile {
    uint64_t cycles[MBR_REGION_COUNT];     // Cycle counts for each region
    uint64_t invocations[MBR_REGION_COUNT]; // Number of invocations
    uint64_t total_cycles;                  // Total cycles in MBR
    uint64_t total_invocations;             // Total MBR invocations
    uint64_t last_timestamp;                // Last PMU timestamp
};

/* Global profiling enable flag */
extern bool mbr_profiling_enabled;

/* Per-CPU profiling data */
extern struct mbr_profile mbr_profiles[MAX_CPUS];

/* Initialize PMU for MBR profiling */
static inline void pmu_mbr_init(void) {
    uint64_t val;
    
    /* Enable cycle counter */
    asm volatile("msr pmcr_el0, %0" :: "r" ((uint64_t)(1 << 0) | (1 << 2))); // Enable and reset
    asm volatile("msr pmcntenset_el0, %0" :: "r" ((uint64_t)(1 << 31))); // Enable cycle counter
    
    mbr_profiling_enabled = true;
}

/* Read cycle counter */
static inline uint64_t pmu_read_cycles(void) {
    uint64_t cycles;
    asm volatile("mrs %0, pmccntr_el0" : "=r" (cycles));
    return cycles;
}

/* Start timing a region */
static inline uint64_t pmu_mbr_start(void) {
    if (!mbr_profiling_enabled) return 0;
    return pmu_read_cycles();
}

/* End timing a region and accumulate */
static inline void pmu_mbr_end(mbr_region_t region, uint64_t start_cycles) {
    if (!mbr_profiling_enabled) return;
    
    uint64_t end_cycles = pmu_read_cycles();
    uint64_t delta = end_cycles - start_cycles;
    
    size_t cpu_id = cpu_id_safe();
    if (cpu_id < MAX_CPUS) {
        mbr_profiles[cpu_id].cycles[region] += delta;
        mbr_profiles[cpu_id].invocations[region]++;
        mbr_profiles[cpu_id].total_cycles += delta;
    }
}

/* Record MBR invocation */
static inline void pmu_mbr_record_invocation(void) {
    if (!mbr_profiling_enabled) return;
    
    size_t cpu_id = cpu_id_safe();
    if (cpu_id < MAX_CPUS) {
        mbr_profiles[cpu_id].total_invocations++;
    }
}

/* Print profiling results */
void pmu_mbr_print_stats(void);

/* Reset profiling counters */
void pmu_mbr_reset(void);

#endif /* PMU_MBR_H */