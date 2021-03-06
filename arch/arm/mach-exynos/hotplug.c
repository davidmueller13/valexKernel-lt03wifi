/* linux arch/arm/mach-exynos4/hotplug.c
 *
 *  Cloned from linux/arch/arm/mach-realview/hotplug.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/cp15.h>
#include <asm/smp_plat.h>

#include <plat/cpu.h>
#include <mach/regs-pmu.h>

#include "common.h"

extern void change_power_base(unsigned int cpu, void __iomem *base);

static inline void cpu_leave_lowpower(void)
{
	unsigned int v, u;

	asm volatile(
	"mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	orr	%0, %0, %3\n"
	"	ldr	%1, [%4]\n"
	"	and	%1, %1, #0\n"
	"	orr	%0, %0, %1\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	  : "=&r" (v), "=&r" (u)
	  : "Ir" (CR_C), "Ir" (0x40), "Ir" (EXYNOS_INFORM0)
	  : "cc");
}

static void exynos_power_down_cpu(unsigned int cpu)
{
	void __iomem *power_base;
	unsigned int pwr_offset = 0;

	set_boot_flag(cpu, HOTPLUG);

	if (soc_is_exynos5410()) {
		int cluster_id = read_cpuid_mpidr() & 0x100;
		if (samsung_rev() < EXYNOS5410_REV_1_0) {
			if (cluster_id == 0)
				pwr_offset = 4;
		} else {
			if (cluster_id != 0)
				pwr_offset = 4;
		}
	} else if (soc_is_exynos5420()) {
		int cluster_id;
		asm ("mrc\tp15, 0, %0, c0, c0, 5\n":"=r"(cluster_id));
		cluster_id = (cluster_id >> 8) & 0xf;
		if (cluster_id)
			pwr_offset = 4;
	}

	power_base = EXYNOS_ARM_CORE_CONFIGURATION(cpu + pwr_offset);
#ifdef CONFIG_EXYNOS5_CCI
	change_power_base(cpu, power_base);
#endif
	__raw_writel(0, power_base);

	return;
}

static inline void platform_do_lowpower(unsigned int cpu, int *spurious)
{
	for (;;) {

		/* make secondary cpus to be turned off at next WFI command */
		exynos_power_down_cpu(cpu_logical_map(cpu));

		/*
		 * here's the WFI
		 */
		asm(".word	0xe320f003\n"
		    :
		    :
		    : "memory", "cc");

		if (pen_release == cpu_logical_map(cpu)) {
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		}

		/*
		 * Getting here, means that we have come out of WFI without
		 * having been woken up - this shouldn't happen
		 *
		 * Just note it happening - when we're woken, we can report
		 * its occurrence.
		 */
		(*spurious)++;
	}
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void exynos_cpu_die(unsigned int cpu)
{
	int spurious = 0;

	v7_exit_coherency_flush(louis);

	/*
	 * bring this CPU back into the world of cache
	 * coherency, and then restore interrupts
	 */
	cpu_leave_lowpower();

	if (spurious)
		pr_warn("CPU%u: %u spurious wakeup calls\n", cpu, spurious);
}
