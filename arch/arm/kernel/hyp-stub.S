/*
 * Copyright (c) 2012 Linaro Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/virt.h>

/*
 * For the kernel proper, we need to find out the CPU boot mode long after
 * boot, so we need to store it in a writable variable.
 *
 * This is not in .bss, because we set it sufficiently early that the boot-time
 * zeroing of .bss would clobber it.
 */
.data
ENTRY(__boot_cpu_mode)
	.long	0
.text

	/*
	 * Save the primary CPU boot mode. Requires 3 scratch registers.
	 */
	.macro	store_primary_cpu_mode	reg1, reg2, reg3
	mrs	\reg1, cpsr
	and	\reg1, \reg1, #MODE_MASK
	adr	\reg2, .L__boot_cpu_mode_offset
	ldr	\reg3, [\reg2]
	str	\reg1, [\reg2, \reg3]
	.endm

	/*
	 * Compare the current mode with the one saved on the primary CPU.
	 * If they don't match, record that fact. The Z bit indicates
	 * if there's a match or not.
	 * Requires 3 additionnal scratch registers.
	 */
	.macro	compare_cpu_mode_with_primary mode, reg1, reg2, reg3
	adr	\reg2, .L__boot_cpu_mode_offset
	ldr	\reg3, [\reg2]
	ldr	\reg1, [\reg2, \reg3]
	cmp	\mode, \reg1		@ matches primary CPU boot mode?
	orrne	r7, r7, #BOOT_CPU_MODE_MISMATCH
	strne	r7, [r5, r6]		@ record what happened and give up
	.endm

/*
 * Hypervisor stub installation functions.
 *
 * These must be called with the MMU and D-cache off.
 * They are not ABI compliant and are only intended to be called from the kernel
 * entry points in head.S.
 */
@ Call this from the primary CPU
ENTRY(__hyp_stub_install)
	store_primary_cpu_mode	r4, r5, r6
ENDPROC(__hyp_stub_install)

	@ fall through...

@ Secondary CPUs should call here
ENTRY(__hyp_stub_install_secondary)
	mrs	r4, cpsr
	and	r4, r4, #MODE_MASK

	/*
	 * If the secondary has booted with a different mode, give up
	 * immediately.
	 */
	compare_cpu_mode_with_primary	r4, r5, r6, r7
	bxne	lr

	/*
	 * Once we have given up on one CPU, we do not try to install the
	 * stub hypervisor on the remaining ones: because the saved boot mode
	 * is modified, it can't compare equal to the CPSR mode field any
	 * more.
	 *
	 * Otherwise...
	 */

	cmp	r4, #HYP_MODE
	bxne	lr			@ give up if the CPU is not in HYP mode

/*
 * Configure HSCTLR to set correct exception endianness/instruction set
 * state etc.
 * Turn off all traps
 * Eventually, CPU-specific code might be needed -- assume not for now
 *
 * This code relies on the "eret" instruction to synchronize the
 * various coprocessor accesses.
 */
	@ Now install the hypervisor stub:
	adr	r7, __hyp_stub_vectors
	mcr	p15, 4, r7, c12, c0, 0	@ set hypervisor vector base (HVBAR)

	@ Disable all traps, so we don't get any nasty surprise
	mov	r7, #0
	mcr	p15, 4, r7, c1, c1, 0	@ HCR
	mcr	p15, 4, r7, c1, c1, 2	@ HCPTR
	mcr	p15, 4, r7, c1, c1, 3	@ HSTR

THUMB(	orr	r7, #(1 << 30)	)	@ HSCTLR.TE
#ifdef CONFIG_CPU_BIG_ENDIAN
	orr	r7, #(1 << 9)		@ HSCTLR.EE
#endif
	mcr	p15, 4, r7, c1, c0, 0	@ HSCTLR

	mrc	p15, 4, r7, c1, c1, 1	@ HDCR
	and	r7, #0x1f		@ Preserve HPMN
	mcr	p15, 4, r7, c1, c1, 1	@ HDCR

	bic	r7, r4, #MODE_MASK
	orr	r7, r7, #SVC_MODE
THUMB(	orr	r7, r7, #PSR_T_BIT	)
	msr	spsr_cxsf, r7		@ This is SPSR_hyp.

	__MSR_ELR_HYP(14)		@ msr elr_hyp, lr
	__ERET				@ return, switching to SVC mode
					@ The boot CPU mode is left in r4.
ENDPROC(__hyp_stub_install_secondary)

__hyp_stub_do_trap:
	cmp	r0, #-1
	mrceq	p15, 4, r0, c12, c0, 0	@ get HVBAR
	mcrne	p15, 4, r0, c12, c0, 0	@ set HVBAR
	__ERET
ENDPROC(__hyp_stub_do_trap)

/*
 * __hyp_set_vectors: Call this after boot to set the initial hypervisor
 * vectors as part of hypervisor installation.  On an SMP system, this should
 * be called on each CPU.
 *
 * r0 must be the physical address of the new vector table (which must lie in
 * the bottom 4GB of physical address space.
 *
 * r0 must be 32-byte aligned.
 *
 * Before calling this, you must check that the stub hypervisor is installed
 * everywhere, by waiting for any secondary CPUs to be brought up and then
 * checking that BOOT_CPU_MODE_HAVE_HYP(__boot_cpu_mode) is true.
 *
 * If not, there is a pre-existing hypervisor, some CPUs failed to boot, or
 * something else went wrong... in such cases, trying to install a new
 * hypervisor is unlikely to work as desired.
 *
 * When you call into your shiny new hypervisor, sp_hyp will contain junk,
 * so you will need to set that to something sensible at the new hypervisor's
 * initialisation entry point.
 */
ENTRY(__hyp_get_vectors)
	mov	r0, #-1
ENDPROC(__hyp_get_vectors)
	@ fall through
ENTRY(__hyp_set_vectors)
	__HVC(0)
	bx	lr
ENDPROC(__hyp_set_vectors)

.align 2
.L__boot_cpu_mode_offset:
	.long	__boot_cpu_mode - .

.align 5
__hyp_stub_vectors:
__hyp_stub_reset:	W(b)	.
__hyp_stub_und:		W(b)	.
__hyp_stub_svc:		W(b)	.
__hyp_stub_pabort:	W(b)	.
__hyp_stub_dabort:	W(b)	.
__hyp_stub_trap:	W(b)	__hyp_stub_do_trap
__hyp_stub_irq:		W(b)	.
__hyp_stub_fiq:		W(b)	.
ENDPROC(__hyp_stub_vectors)

