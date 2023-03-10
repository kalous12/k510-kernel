// SPDX-License-Identifier: GPL-2.0

#include <linux/pagewalk.h>
#include <linux/ptdump.h>
#include <linux/kasan.h>

#ifdef CONFIG_KASAN
/*
 * This is an optimization for KASAN=y case. Since all kasan page tables
 * eventually point to the kasan_early_shadow_page we could call note_page()
 * right away without walking through lower level page tables. This saves
 * us dozens of seconds (minutes for 5-level config) while checking for
 * W+X mapping or reading kernel_page_tables debugfs file.
 */
static inline int note_kasan_page_table(struct mm_walk *walk,
					unsigned long addr)
{
	struct ptdump_state *st = walk->private;

	st->note_page(st, addr, 4, pte_val(kasan_early_shadow_pte[0]));

	walk->action = ACTION_CONTINUE;

	return 0;
}
#endif

static int ptdump_pgd_entry(pgd_t *pgd, unsigned long addr,
			    unsigned long next, struct mm_walk *walk)
{
	struct ptdump_state *st = walk->private;
	pgd_t val = READ_ONCE(*pgd);

#if CONFIG_PGTABLE_LEVELS > 4 && defined(CONFIG_KASAN)
	if (pgd_page(val) == virt_to_page(lm_alias(kasan_early_shadow_p4d)))
		return note_kasan_page_table(walk, addr);
#endif

	if (pgd_leaf(val))
		st->note_page(st, addr, 0, pgd_val(val));

	return 0;
}

static int ptdump_p4d_entry(p4d_t *p4d, unsigned long addr,
			    unsigned long next, struct mm_walk *walk)
{
	struct ptdump_state *st = walk->private;
	p4d_t val = READ_ONCE(*p4d);

#if CONFIG_PGTABLE_LEVELS > 3 && defined(CONFIG_KASAN)
	if (p4d_page(val) == virt_to_page(lm_alias(kasan_early_shadow_pud)))
		return note_kasan_page_table(walk, addr);
#endif

	if (p4d_leaf(val))
		st->note_page(st, addr, 1, p4d_val(val));

	return 0;
}

static int ptdump_pud_entry(pud_t *pud, unsigned long addr,
			    unsigned long next, struct mm_walk *walk)
{
	struct ptdump_state *st = walk->private;
	pud_t val = READ_ONCE(*pud);

#if CONFIG_PGTABLE_LEVELS > 2 && defined(CONFIG_KASAN)
	if (pud_page(val) == virt_to_page(lm_alias(kasan_early_shadow_pmd)))
		return note_kasan_page_table(walk, addr);
#endif

	if (pud_leaf(val))
		st->note_page(st, addr, 2, pud_val(val));

	return 0;
}

static int ptdump_pmd_entry(pmd_t *pmd, unsigned long addr,
			    unsigned long next, struct mm_walk *walk)
{
	struct ptdump_state *st = walk->private;
//	printk("ptdump_pmd_entry: pmd=%016lx \n", pmd);
	pmd_t val = READ_ONCE(*pmd);
//	printk("ptdump_pmd_entry: val=%016lx \n", val);

#if defined(CONFIG_KASAN)
	if (pmd_page(val) == virt_to_page(lm_alias(kasan_early_shadow_pte)))
		return note_kasan_page_table(walk, addr);
#endif

	if (pmd_leaf(val))
		st->note_page(st, addr, 3, pmd_val(val));

	return 0;
}

static int ptdump_pte_entry(pte_t *pte, unsigned long addr,
			    unsigned long next, struct mm_walk *walk)
{
	struct ptdump_state *st = walk->private;

	st->note_page(st, addr, 4, pte_val(READ_ONCE(*pte)));

	return 0;
}

static int ptdump_hole(unsigned long addr, unsigned long next,
		       int depth, struct mm_walk *walk)
{
	struct ptdump_state *st = walk->private;

	st->note_page(st, addr, depth, 0);

	return 0;
}

/*
static const struct mm_walk_ops ptdump_ops = {
	.pgd_entry	= ptdump_pgd_entry,
	.p4d_entry	= ptdump_p4d_entry,
	.pud_entry	= ptdump_pud_entry,
	.pmd_entry	= ptdump_pmd_entry,
	.pte_entry	= ptdump_pte_entry,
	.pte_hole	= ptdump_hole,
};
*/

void ptdump_walk_pgd(struct ptdump_state *st, struct mm_struct *mm, pgd_t *pgd)
{
	struct mm_walk walk = {
		.pgd_entry      = ptdump_pgd_entry,
		.p4d_entry      = ptdump_p4d_entry,
		.pud_entry      = ptdump_pud_entry,
		.pmd_entry      = ptdump_pmd_entry,
		.pte_entry      = ptdump_pte_entry,
		.pte_hole       = ptdump_hole,
                .mm             = mm,
                .pgd            = pgd,
                .private        = st,
                .no_vma         = true
	};
	const struct ptdump_range *range = st->range;

	down_read(&mm->mmap_sem);
	while (range->start != range->end) {
		walk_page_range_novma(range->start, range->end,
				      &walk);
		range++;
	}
	up_read(&mm->mmap_sem);

	/* Flush out the last page */
	st->note_page(st, 0, -1, 0);
}
