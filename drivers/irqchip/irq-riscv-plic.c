/*
 * Copyright (C) 2017 SiFive
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

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

/*
 * From the RISC-V Privlidged Spec v1.10:
 *
 * Global interrupt sources are assigned small unsigned integer identifiers,
 * beginning at the value 1.  An interrupt ID of 0 is reserved to mean “no
 * interrupt”.  Interrupt identifiers are also used to break ties when two or
 * more interrupt sources have the same assigned priority. Smaller values of
 * interrupt ID take precedence over larger values of interrupt ID.
 *
 * While the RISC-V supervisor spec doesn't define the maximum number of
 * devices supported by the PLIC, the largest number supported by devices
 * marked as 'riscv,plic0' (which is the only device type this driver supports,
 * and is the only extant PLIC as of now) is 1024.  As mentioned above, device
 * 0 is defined to be non-existant so this device really only supports 1023
 * devices.
 */
#define MAX_DEVICES	1024
#define MAX_USE_REGS	(MAX_DEVICES / 32)
#define MAX_CONTEXTS	15872

/*
 * The PLIC consists of memory-mapped control registers, with a memory map as
 * follows:
 *
 * base + 0x000000: Reserved (interrupt source 0 does not exist)
 * base + 0x000004: Interrupt source 1 priority
 * base + 0x000008: Interrupt source 2 priority
 * ...
 * base + 0x000FFC: Interrupt source 1023 priority
 * base + 0x001000: Pending 0
 * base + 0x001FFF: Pending
 * base + 0x002000: Enable bits for sources 0-31 on context 0
 * base + 0x002004: Enable bits for sources 32-63 on context 0
 * ...
 * base + 0x0020FC: Enable bits for sources 992-1023 on context 0
 * base + 0x002080: Enable bits for sources 0-31 on context 1
 * ...
 * base + 0x002100: Enable bits for sources 0-31 on context 2
 * ...
 * base + 0x1F1F80: Enable bits for sources 992-1023 on context 15871
 * base + 0x1F1F84: Reserved
 * ...              (higher context IDs would fit here, but wouldn't fit
 *                   inside the per-context priority vector)
 * base + 0x1FFFFC: Reserved
 * base + 0x200000: Priority threshold for context 0
 * base + 0x200004: Claim/complete for context 0
 * base + 0x200008: Reserved
 * ...
 * base + 0x200FFC: Reserved
 * base + 0x201000: Priority threshold for context 1
 * base + 0x201004: Claim/complete for context 1
 * ...
 * base + 0xFFE000: Priority threshold for context 15871
 * base + 0xFFE004: Claim/complete for context 15871
 * base + 0xFFE008: Reserved
 * ...
 * base + 0xFFFFFC: Reserved
 *
 * V5 Support:
 *
 * base + 0x000000: V5 Feature enable register (vectored/preempt)
 *
 * base + 0x001000: V5 Interrupt pending array
 * base + 0x001080: V5 Trigger type array (read-only)
 * base + 0x001100: V5 Number of interrupts and targets (read-only)
 * base + 0x001104: V5 Version and max priority register (read-only)
 *
 * base + 0x200400: V5 Preempted priority stack for context 0
 * ...
 * base + 0x20041F: V5 Preempted priority stack for context 0
 * base + 0x201400: V5 Preempted priority stack for context 1
 * base + 0xFFE400: V5 Preempted priority stack for context 15871
 */

/* Each interrupt source has a priority register associated with it. */
#define PRIORITY_BASE		0
#define PRIORITY_PER_ID		4

/*
 * Each hart context has a vector of interupt enable bits associated with it.
 * There's one bit for each interrupt source.
 */
#define ENABLE_BASE		0x2000
#define ENABLE_PER_HART		0x80

/*
 * Each hart context has a set of control registers associated with it.  Right
 * now there's only two: a source priority threshold over which the hart will
 * take an interrupt, and a register to claim interrupts.
 */
#define CONTEXT_BASE		0x200000
#define CONTEXT_PER_HART	0x1000
#define CONTEXT_THRESHOLD	0
#define CONTEXT_CLAIM		4

/* V5 Support */
#ifdef CONFIG_PLIC_NCEPLIC100
#define FEATURE_ENABLE_BASE	0
#define FEATURE_PREEMPT		0
#define FEATURE_VECTORED	1

#define INFO_BASE		0x1000
#define INFO_PENDING_RW		0x0
#define INFO_TRIGGER_TYPE	0x80
#define INFO_NUMBER		0x100
#define INFO_VER_AND_MAX	0x104

#define CONTEXT_PREEMPT		0x400	// base on CONTEXT_BASE
#endif

/*
 * PLIC devices are named like 'riscv,plic0,%llx', this is enough space to
 * store that name.
 */
#define PLIC_DATA_NAME_SIZE 30

struct plic_handler {
	bool			present;
	int			contextid;
	struct plic_data	*data;
};

struct plic_data {
	struct irq_chip		chip;
	struct irq_domain	*domain;
	u32			ndev;
	void __iomem		*reg;
	int			handlers;
	struct plic_handler	*handler;
	char			name[PLIC_DATA_NAME_SIZE];
	spinlock_t		lock;
};

/* Addressing helper functions. */
static inline
u32 __iomem *plic_enable_vector(struct plic_data *data, int contextid)
{
	return data->reg + ENABLE_BASE + contextid * ENABLE_PER_HART;
}

static inline
u32 __iomem *plic_priority(struct plic_data *data, int hwirq)
{
	return data->reg + PRIORITY_BASE + hwirq * PRIORITY_PER_ID;
}

static inline
u32 __iomem *plic_hart_threshold(struct plic_data *data, int contextid)
{
	return data->reg + CONTEXT_BASE + CONTEXT_PER_HART * contextid + CONTEXT_THRESHOLD;
}

static inline
u32 __iomem *plic_hart_claim(struct plic_data *data, int contextid)
{
	return data->reg + CONTEXT_BASE + CONTEXT_PER_HART * contextid + CONTEXT_CLAIM;
}

/* V5 support */
#ifdef CONFIG_PLIC_NCEPLIC100
static inline
u32 __iomem *plic_hart_feature_enable(struct plic_data *data)
{
	return data->reg + FEATURE_ENABLE_BASE;
}

static inline
u32 __iomem *plic_hart_pending(struct plic_data *data)
{
	return data->reg + INFO_BASE + INFO_PENDING_RW;
}

static inline
u32 __iomem *plic_hart_trigger_type(struct plic_data *data)
{
	return data->reg + INFO_BASE + INFO_TRIGGER_TYPE;
}

static inline
u32 __iomem *plic_hart_number(struct plic_data *data)
{
	return data->reg + INFO_BASE + INFO_NUMBER;
}

static inline
u32 __iomem *plic_hart_version_and_max(struct plic_data *data)
{
	return data->reg + INFO_BASE + INFO_VER_AND_MAX;
}

static inline
u32 __iomem *plic_hart_preempt(struct plic_data *data, int contextid)
{
	return data->reg +
	       CONTEXT_BASE + CONTEXT_PER_HART * contextid + CONTEXT_PREEMPT;
}
#endif

/*
 * Handling an interrupt is a two-step process: first you claim the interrupt
 * by reading the claim register, then you complete the interrupt by writing
 * that source ID back to the same claim register.  This automatically enables
 * and disables the interrupt, so there's nothing else to do.
 */
static inline
u32 plic_claim(struct plic_data *data, int contextid)
{
	return readl(plic_hart_claim(data, contextid));
}

static inline
void plic_complete(struct plic_data *data, int contextid, u32 claim)
{
	writel(claim, plic_hart_claim(data, contextid));
}

/* Explicit interrupt masking. */
static void plic_disable(struct plic_data *data, int contextid, int hwirq)
{
	u32 __iomem *reg = plic_enable_vector(data, contextid) + (hwirq / 32);
	u32 mask = ~(1 << (hwirq % 32));

	spin_lock(&data->lock);
	writel(readl(reg) & mask, reg);
	spin_unlock(&data->lock);
}

static void plic_enable(struct plic_data *data, int contextid, int hwirq)
{
	u32 __iomem *reg = plic_enable_vector(data, contextid) + (hwirq / 32);
	u32 bit = 1 << (hwirq % 32);

	spin_lock(&data->lock);
	writel(readl(reg) | bit, reg);
	spin_unlock(&data->lock);
}

/*
 * There is no need to mask/unmask PLIC interrupts
 * They are "masked" by reading claim and "unmasked" when writing it back.
 */
static void plic_irq_mask(struct irq_data *d) { }
static void plic_irq_unmask(struct irq_data *d) { }

static void plic_irq_enable(struct irq_data *d)
{
	struct plic_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *priority = plic_priority(data, d->hwirq);
	int i;

	writel(1, priority);
	for (i = 0; i < data->handlers; ++i)
		if (data->handler[i].present)
			plic_enable(data, i, d->hwirq);
}

static void plic_irq_disable(struct irq_data *d)
{
	struct plic_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *priority = plic_priority(data, d->hwirq);
	int i;

	writel(0, priority);
	for (i = 0; i < data->handlers; ++i)
		if (data->handler[i].present)
			plic_disable(data, i, d->hwirq);
}

static void plic_irq_eoi(struct irq_data *d)
{
	/* FIXME: I can't figure out what's going on here: when I set the PLIC
	 * driver to use hadle_fasteoi_irq I end up with !handler here.  The
	 * difference is that here we get an irq_data and use
	 * common.handler_data, while in the working version we get an
	 * irq_desc and use irq_common_data.handler_data. */
	struct plic_handler *handler = d->common->handler_data;
	BUG_ON(!handler);
	plic_complete(handler->data, handler->contextid, d->hwirq);
}

static int plic_irqdomain_map(struct irq_domain *d, unsigned int irq,
			      irq_hw_number_t hwirq)
{
	struct plic_data *data = d->host_data;

	/* FIXME: ... I want to set handle_fasteoi_irq here. */
	irq_set_chip_and_handler(irq, &data->chip, handle_simple_irq);
	irq_set_chip_data(irq, data);
	irq_set_noprobe(irq);

	return 0;
}

/* V5 Support */
#ifdef CONFIG_PLIC_NCEPLIC100
static void plic_enable_preempt(struct plic_data *data)
{
	u32 __iomem *reg = plic_hart_feature_enable(data);
	u32 mask = 1 << FEATURE_PREEMPT;

	spin_lock(&data->lock);
	writel(readl(reg) | mask, reg);
	spin_unlock(&data->lock);
}

static void plic_disable_preempt(struct plic_data *data)
{
	u32 __iomem *reg = plic_hart_feature_enable(data);
	u32 mask = ~(1 << FEATURE_PREEMPT);

	spin_lock(&data->lock);
	writel(readl(reg) & mask, reg);
	spin_unlock(&data->lock);
}

static void plic_enable_pending(struct plic_data *data, int hwirq, int val)
{
	u32 __iomem *reg = plic_hart_pending(data) + (hwirq / 32);
	u32 bit = 1 << (hwirq % 32);

	spin_lock(&data->lock);
	writel(readl(reg) | bit, reg);
	spin_unlock(&data->lock);
}

static void plic_disable_pending(struct plic_data *data, int hwirq, int val)
{
	u32 __iomem *reg = plic_hart_pending(data) + (hwirq / 32);
	u32 mask = ~(1 << (hwirq % 32));

	spin_lock(&data->lock);
	writel(readl(reg) & mask, reg);
	spin_unlock(&data->lock);
}

static u32 plic_get_pending(struct plic_data *data, int hwirq)
{
	u32 __iomem *reg = plic_hart_pending(data) + (hwirq / 32);
	u32 mask = 1 << (hwirq % 32);

	return (readl(reg) & mask) >> (hwirq % 32);
}

static u32 plic_get_trigger_type(struct plic_data *data, int hwirq)
{
	u32 __iomem *reg = plic_hart_trigger_type(data) + (hwirq / 32);
	u32 mask = 1 << (hwirq % 32);

	return (readl(reg) & mask) >> (hwirq % 32);
}

static u32 plic_get_target_total(struct plic_data *data)
{
	return readl(plic_hart_number(data)) & 0x0000ffff;
}

static u32 plic_get_interrupt_total(struct plic_data *data)
{
	return readl(plic_hart_number(data)) >> 16;
}

static u32 plic_get_version(struct plic_data *data)
{
	return readl(plic_hart_version_and_max(data)) & 0x0000ffff;
}

static u32 plic_get_max_priority(struct plic_data *data)
{
	return readl(plic_hart_version_and_max(data)) >> 16;
}
#endif

static const struct irq_domain_ops plic_irqdomain_ops = {
	.map	= plic_irqdomain_map,
	.xlate	= irq_domain_xlate_onecell,
};

u32 __iomem *plic_base;
unsigned int *wake_mask;
unsigned int *orig;

static int plic_set_wake(struct irq_data *d, unsigned int on)
{
	struct plic_data *data = irq_data_get_irq_chip_data(d);
	u32 bit[MAX_USE_REGS];
	u32 offset = d->hwirq / 32;
	int i;

	plic_base = data->reg;
	bit[offset] = 1 << (d->hwirq % 32);

	if (on)
		__assign_bit(d->hwirq, (unsigned long *)wake_mask, true);
	else
		__assign_bit(d->hwirq, (unsigned long *)wake_mask, false);

	for(i = 0; i < data->handlers; ++i) {
		if (data->handler[i].present) {
			unsigned int int_mask[MAX_USE_REGS];
			u32 __iomem *reg = plic_enable_vector(data,
						data->handler[i].contextid)
						+ offset;
			u32 target_area = i * MAX_USE_REGS;

			int_mask[offset] = readl(reg);
			if (on) {
				if (int_mask[offset] & bit[offset])
					__assign_bit(d->hwirq,
						    (unsigned long *)&orig[target_area],
						    true);
				else
					__assign_bit(d->hwirq,
						    (unsigned long *)&orig[target_area],
						    false);

				__assign_bit(d->hwirq, (unsigned long *)int_mask, true);
			} else {
				if (!(orig[target_area] & bit[offset]))
					__assign_bit(d->hwirq,
						(unsigned long *)int_mask, false);

				__assign_bit(d->hwirq, (unsigned long *)&orig[target_area],
					    false);
			}
			writel(int_mask[offset], reg);
		}
	}
	return 0;
}

static void plic_chained_handle_irq(struct irq_desc *desc)
{
	struct plic_handler *handler = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_domain *domain = handler->data->domain;
	u32 what;

	chained_irq_enter(chip, desc);

	while ((what = plic_claim(handler->data, handler->contextid))) {
		int irq = irq_find_mapping(domain, what);

		if (irq > 0)
			generic_handle_irq(irq);
		else
			handle_bad_irq(desc);
		plic_complete(handler->data, handler->contextid, what);
	}

	chained_irq_exit(chip, desc);
}

static int plic_init(struct device_node *node, struct device_node *parent)
{
	struct plic_data *data;
	struct resource resource;
	int i, ok = 0;
	int out = -1;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (WARN_ON(!data))
		return -ENOMEM;

	spin_lock_init(&data->lock);

	data->reg = of_iomap(node, 0);
	if (WARN_ON(!data->reg)) {
		out = -EIO;
		goto free_data;
	}

	of_property_read_u32(node, "riscv,ndev", &data->ndev);
	if (WARN_ON(!data->ndev)) {
		out = -EINVAL;
		goto free_reg;
	}

	data->handlers = of_irq_count(node);
	if (WARN_ON(!data->handlers)) {
		out = -EINVAL;
		goto free_reg;
	}

	data->handler =
		kcalloc(data->handlers, sizeof(*data->handler), GFP_KERNEL);
	if (WARN_ON(!data->handler)) {
		out = -ENOMEM;
		goto free_reg;
	}

	data->domain = irq_domain_add_linear(node, data->ndev+1, &plic_irqdomain_ops, data);
	if (WARN_ON(!data->domain)) {
		out = -ENOMEM;
		goto free_handler;
	}

	wake_mask = kzalloc(MAX_USE_REGS * sizeof(u32), GFP_KERNEL);
	if (WARN_ON(!wake_mask))
		return -ENOMEM;

	orig = kzalloc(data->handlers * MAX_USE_REGS * sizeof(u32), GFP_KERNEL);
	if (WARN_ON(!orig))
		return -ENOMEM;

	of_address_to_resource(node, 0, &resource);
	snprintf(data->name, sizeof(data->name),
		 "riscv,plic0,%llx", resource.start);
	data->chip.name = data->name;
	data->chip.irq_mask = plic_irq_mask;
	data->chip.irq_unmask = plic_irq_unmask;
	data->chip.irq_enable = plic_irq_enable;
	data->chip.irq_disable = plic_irq_disable;
	data->chip.irq_eoi = plic_irq_eoi;
	data->chip.irq_set_wake = plic_set_wake;

	for (i = 0; i < data->handlers; ++i) {
		struct plic_handler *handler = &data->handler[i];
		struct of_phandle_args parent;
		int parent_irq, hwirq;

		handler->present = false;

		if (of_irq_parse_one(node, i, &parent))
			continue;
		/* skip context holes */
		if (parent.args[0] == -1)
			continue;

		/* skip any contexts that lead to inactive harts */
		if (of_device_is_compatible(parent.np, "riscv,cpu-intc") &&
		    parent.np->parent &&
		    riscv_of_processor_hart(parent.np->parent) < 0)
			continue;

		parent_irq = irq_create_of_mapping(&parent);
		if (!parent_irq)
			continue;

		handler->present = true;
		handler->contextid = i;
		handler->data = data;
		/* hwirq prio must be > this to trigger an interrupt */
		writel(0, plic_hart_threshold(data, i));

		for (hwirq = 1; hwirq <= data->ndev; ++hwirq)
			plic_disable(data, i, hwirq);
		irq_set_chained_handler_and_data(parent_irq, plic_chained_handle_irq, handler);
		++ok;
	}

	pr_info("%s: mapped %d interrupts to %d/%d handlers\n",
	        data->name, data->ndev, ok, data->handlers);
	WARN_ON(!ok);
	return 0;

free_handler:
	kfree(data->handler);
free_reg:
	iounmap(data->reg);
free_data:
	kfree(data);
	kfree(wake_mask);
	kfree(orig);
	return out;
}

IRQCHIP_DECLARE(plic0, "riscv,plic0", plic_init);
