/*
 * Hibernation support specific for ARM
 *
 * Derived from work on ARM hibernation support by:
 *
 * Ubuntu project, hibernation support for mach-dove
 * Copyright (C) 2010 Nokia Corporation (Hiroshi Doyu)
 * Copyright (C) 2010 Texas Instruments, Inc. (Teerth Reddy et al.)
 *	https://lkml.org/lkml/2010/6/18/4
 *	https://lists.linux-foundation.org/pipermail/linux-pm/2010-June/027422.html
 *	https://patchwork.kernel.org/patch/96442/
 *
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <asm/tlbflush.h>

extern const void __nosave_begin, __nosave_end;

int pfn_is_nosave(unsigned long pfn)
{
	unsigned long nosave_begin_pfn = __pa_symbol(&__nosave_begin) >> PAGE_SHIFT;
	unsigned long nosave_end_pfn = PAGE_ALIGN(__pa_symbol(&__nosave_end)) >> PAGE_SHIFT;

	return (pfn >= nosave_begin_pfn) && (pfn < nosave_end_pfn);
}

void save_processor_state(void)
{
	flush_thread();
}

void restore_processor_state(void)
{
	local_flush_tlb_all();
}

u8 __swsusp_arch_ctx[PAGE_SIZE] __attribute__((aligned(PAGE_SIZE)));
u8 __swsusp_resume_stk[PAGE_SIZE/2] __nosavedata;

/*
 * The framework loads the hibernation image into this linked list,
 * for swsusp_arch_resume() to copy back to the proper destinations.
 *
 * To make this work if resume is triggered from initramfs, the
 * pagetables need to be switched to allow writes to kernel mem.
 */
void notrace __swsusp_arch_restore_prepare(void)
{
	cpu_switch_mm(__virt_to_phys(swapper_pg_dir), current->active_mm);
}

void notrace __swsusp_arch_restore_image(void)
{
	extern struct pbe *restore_pblist;
	struct pbe *pbe;

	for (pbe = restore_pblist; pbe; pbe = pbe->next)
		copy_page(pbe->orig_address, pbe->address);
}
