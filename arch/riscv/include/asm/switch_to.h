/*
 * Copyright (C) 2012 Regents of the University of California
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

#ifndef _ASM_RISCV_SWITCH_TO_H
#define _ASM_RISCV_SWITCH_TO_H

#include <asm/processor.h>
#include <asm/ptrace.h>
#include <asm/csr.h>

#ifdef CONFIG_FPU
extern void __fstate_save(struct task_struct *save_to);
extern void __fstate_restore(struct task_struct *restore_from);

static inline void __fstate_clean(struct pt_regs *regs)
{
	regs->sstatus |= (regs->sstatus & ~(SR_FS)) | SR_FS_CLEAN;
}

static inline void fstate_save(struct task_struct *task,
			       struct pt_regs *regs)
{
	if ((regs->sstatus & SR_FS) == SR_FS_DIRTY) {
		__fstate_save(task);
		__fstate_clean(regs);
	}
}

static inline void fstate_restore(struct task_struct *task,
				  struct pt_regs *regs)
{
	if ((regs->sstatus & SR_FS) != SR_FS_OFF) {
		__fstate_restore(task);
		__fstate_clean(regs);
	}
}

extern bool has_fpu;
#else
#define has_fpu false
#define fstate_save(task, regs) do { } while (0)
#define fstate_restore(task, regs) do { } while (0)
#endif

#ifdef CONFIG_DSP
static inline void dspstate_save(struct task_struct *task)
{
	task->thread.dspstate.ucode = csr_read(ucode);
}
static inline void dspstate_restore(struct task_struct *task)
{
	csr_write(ucode, task->thread.dspstate.ucode);
}
extern bool has_dsp;
#else
#define has_dsp false
#define dspstate_save(task, regs) do { } while (0)
#define dspstate_restore(task, regs) do { } while (0)
#endif

extern struct task_struct *__switch_to(struct task_struct *,
				       struct task_struct *);

static inline void __switch_to_aux(struct task_struct *prev,
				   struct task_struct *next)
{
#ifdef CONFIG_FPU
	struct pt_regs *regs;
	regs = task_pt_regs(prev);
	if (unlikely(regs->sstatus & SR_SD))
		fstate_save(prev, regs);
	fstate_restore(next, task_pt_regs(next));
#endif
#ifdef CONFIG_DSP
	if (likely(has_dsp)) {
		dspstate_save(prev);
		dspstate_restore(next);
	}
#endif
}

#define switch_to(prev, next, last)			\
do {							\
	struct task_struct *__prev = (prev);		\
	struct task_struct *__next = (next);		\
	if (has_fpu || has_dsp)				\
		__switch_to_aux(__prev, __next);	\
	((last) = __switch_to(__prev, __next));		\
} while (0)

#endif /* _ASM_RISCV_SWITCH_TO_H */
