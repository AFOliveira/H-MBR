/**
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) Bao Project and Contributors. All rights reserved.
 */

#include <bao.h>

#include <cpu.h>
#include <mem.h>
#include <interrupts.h>
#include <console.h>
#include <printk.h>
#include <platform.h>
#include <vmm.h>

void init(cpuid_t cpu_id, paddr_t load_addr)
{
    /**
     * These initializations must be executed first and in fixed order.
     */

    cpu_init(cpu_id, load_addr);
    mem_init(load_addr);

    /* -------------------------------------------------------------- */

    console_init();

    if (cpu_is_master()) {
        console_printk ("[START] Profilling Started\n");
        console_printk("Bao Hypervisor\n\r");
    }

    interrupts_init();

    vmm_init();

    /* Should never reach here */
    while (1) { }
}
