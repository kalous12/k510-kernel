#
# Makefile for the RISC-V Linux kernel
#

ifdef CONFIG_FTRACE
CFLAGS_REMOVE_ftrace.o = -pg
CFLAGS_REMOVE_setup.o = -pg
endif

extra-y += head.o
extra-y += vmlinux.lds

obj-y	+= cpu.o
obj-y	+= cpufeature.o
obj-y	+= entry.o
ifdef CONFIG_ARCH_BINFMT_ELF_STATE
obj-y	+= elf.o
endif
obj-y	+= irq.o
obj-y	+= process.o
obj-y	+= ptrace.o
obj-y	+= reset.o
obj-y	+= setup.o
obj-y	+= signal.o
obj-y	+= syscall_table.o
obj-y	+= sys_riscv.o
obj-y	+= time.o
obj-y	+= traps.o
obj-y	+= riscv_ksyms.o
obj-y	+= stacktrace.o
obj-y	+= vdso.o
obj-y	+= cacheinfo.o
obj-y	+= plmt.o
obj-y	+= vdso/

CFLAGS_setup.o := -mcmodel=medany

obj-$(CONFIG_FPU)		+= fpu.o
obj-$(CONFIG_SMP)		+= smpboot.o
obj-$(CONFIG_SMP)		+= smp.o
obj-$(CONFIG_MODULES)		+= module.o
obj-$(CONFIG_MODULE_SECTIONS)	+= module-sections.o

obj-$(CONFIG_FUNCTION_TRACER)	+= mcount.o ftrace.o
obj-$(CONFIG_DYNAMIC_FTRACE)	+= mcount-dyn.o

obj-$(CONFIG_PERF_EVENTS)      += perf_event.o

obj-$(CONFIG_PM)		+= pm.o sleep.o

clean:
