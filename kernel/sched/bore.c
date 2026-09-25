// SPDX-License-Identifier: GPL-2.0
/*
 * kernel/sched/bore.c
 *
 * Burst-Oriented Response Enhancer (BORE) CPU Scheduler
 * Copyright (C) 2021-2025 Masahito Suzuki <firelzrd@gmail.com>
 */
#include "sched.h"

#include <linux/math64.h>
#include <linux/sysctl.h>
#include <linux/sched/bore.h>

#ifdef CONFIG_SCHED_BORE
DEFINE_STATIC_KEY_TRUE(sched_bore_key);
DEFINE_STATIC_KEY_TRUE(sched_burst_inherit_key);
DEFINE_STATIC_KEY_TRUE(sched_burst_ancestor_key);

u8   __read_mostly sched_bore                   = 1;
u8   __read_mostly sched_burst_inherit_type     = 2;
u8   __read_mostly sched_burst_smoothness       = 1;
u8   __read_mostly sched_burst_penalty_offset   = 24;
uint __read_mostly sched_burst_penalty_scale    = 1536;
uint __read_mostly sched_burst_cache_lifetime   = 75000000;
static int __maybe_unused maxval_prio    =   39;
static int __maybe_unused maxval_6_bits  =   63;
static int __maybe_unused maxval_12_bits = 4095;
static int three                         =    3;

#define MAX_BURST_PENALTY ((40U << 8) - 1)
#define BURST_CACHE_SAMPLE_LIMIT 63
#define BURST_CACHE_SCAN_LIMIT (BURST_CACHE_SAMPLE_LIMIT * 2)

static u32 bore_reciprocal_lut[BURST_CACHE_SAMPLE_LIMIT + 1];

static inline u32 log2p1_u64_u32fp(u64 v, u8 fp) {
	int clz, exponent;
	u32 mantissa;

	if (unlikely(!v)) return 0;
	clz = __builtin_clzll(v);
	exponent = 64 - clz;
	mantissa = (u32)((v << clz) << 1 >> (64 - fp));
	return exponent << fp | mantissa;
}

static inline u32 calc_burst_penalty(u64 burst_time) {
	u32 greed = log2p1_u64_u32fp(burst_time, 8),
		tolerance = sched_burst_penalty_offset << 8;
	s32 diff = (s32)(greed - tolerance);
	u32 penalty = diff & ~(diff >> 31);
	u32 scaled_penalty = penalty * sched_burst_penalty_scale >> 10;
	s32 overflow = scaled_penalty - MAX_BURST_PENALTY;
	return scaled_penalty - (overflow & ~(overflow >> 31));
}

static inline u32 binary_smooth(u32 new, u32 old) {
	u32 is_growing = (new > old);
	u32 increment = (new - old) * is_growing;
	u32 shift = sched_burst_smoothness;
	u32 smoothed = old + ((increment + (1U << shift) - 1) >> shift);
	return (new & ~(-is_growing)) | (smoothed & (-is_growing));
}

u8 effective_prio_bore(struct task_struct *p) {
	int prio = p->static_prio - MAX_RT_PRIO;
	s32 diff;

	if (static_branch_likely(&sched_bore_key))
		prio += bore_score(p);
	prio &= ~(prio >> 31);
	diff = prio - maxval_prio;
	prio -= (diff & ~(diff >> 31));
	return (u8)prio;
}

static void update_penalty(struct task_struct *p) {
	struct sched_entity *se = &p->se;
	u8 prev_prio, new_prio;
	s32 diff;
	u16 max_val;
	u32 is_kthread;

	prev_prio = effective_prio_bore(p);

	diff = (s32)se->curr_penalty - (s32)se->prev_penalty;
	max_val = se->curr_penalty - (diff & (diff >> 31));
	is_kthread = !!(p->flags & PF_KTHREAD);
	se->penalty = max_val & -(s32)(!is_kthread);

	new_prio = effective_prio_bore(p);
	if (new_prio != prev_prio)
		reweight_task_by_prio(p, new_prio);
}

void update_curr_bore(struct task_struct *p, u64 delta_exec) {
	struct sched_entity *se = &p->se;
	u32 curr_penalty;

	if (se->stop_update) return;

	se->burst_time += delta_exec;
	curr_penalty = se->curr_penalty = calc_burst_penalty(se->burst_time);

	if (curr_penalty <= se->prev_penalty) return;
	update_penalty(p);
}

void restart_burst_bore(struct task_struct *p) {
	struct sched_entity *se = &p->se;
	u32 new_penalty = binary_smooth(se->curr_penalty, se->prev_penalty);
	se->prev_penalty = new_penalty;
	se->curr_penalty = 0;
	se->burst_time = 0;
	update_penalty(p);
}

static inline bool task_is_bore_eligible(struct task_struct *p)
{ return p && p->sched_class == &fair_sched_class && !p->exit_state; }

#ifndef for_each_child_task
#define for_each_child_task(p, t) \
	list_for_each_entry(t, &(p)->children, sibling)
#endif

static inline u32 count_children_upto2(struct task_struct *p) {
	struct list_head *head = &p->children;
	struct list_head *first = READ_ONCE(head->next);
	struct list_head *second = READ_ONCE(first->next);
	return (first != head) + (second != head);
}

static inline bool burst_cache_expired(struct bore_bc *bc, u64 now) {
	struct bore_bc bc_val = { .value = READ_ONCE(bc->value) };
	u64 timestamp = (u64)bc_val.timestamp << BORE_BC_TIMESTAMP_SHIFT;
	return now - timestamp > (u64)sched_burst_cache_lifetime;
}

static void update_burst_cache(struct bore_bc *bc,
		struct task_struct *p, u32 count, u32 total, u64 now) {
	u32 average = (count == 1) ? total :
		(u32)(((u64)total * bore_reciprocal_lut[count]) >> 32);

	struct bore_bc new_bc = {
		.penalty = max(average, (u32)p->se.penalty),
		.timestamp = now >> BORE_BC_TIMESTAMP_SHIFT
	};
	WRITE_ONCE(bc->value, new_bc.value);
}

static u32 inherit_from_parent(struct task_struct *parent,
								u64 clone_flags, u64 now) {
	struct bore_bc bc_val;
	struct bore_bc *bc;

	if (clone_flags & CLONE_PARENT)
		parent = parent->real_parent;

	bc = &parent->se.subtree;

	if (burst_cache_expired(bc, now)) {
		struct task_struct *child;
		u32 count = 0, total = 0, scan_count = 0;
		for_each_child_task(parent, child) {
			if (count >= BURST_CACHE_SAMPLE_LIMIT) break;
			if (scan_count++ >= BURST_CACHE_SCAN_LIMIT) break;

			if (!task_is_bore_eligible(child)) continue;
			count++;
			total += child->se.penalty;
		}

		update_burst_cache(bc, parent, count, total, now);
	}

	bc_val.value = READ_ONCE(bc->value);
	return (u32)bc_val.penalty;
}

static u32 inherit_from_ancestor_hub(struct task_struct *parent,
									u64 clone_flags, u64 now) {
	struct bore_bc bc_val;
	struct bore_bc *bc;
	struct task_struct *ancestor = parent;
	struct task_struct *next;
	u32 sole_child_count = 0;

	if (clone_flags & CLONE_PARENT) {
		ancestor = ancestor->real_parent;
		sole_child_count = 1;
	}

	for (;
			(next = ancestor->real_parent) != ancestor &&
			count_children_upto2(ancestor) <= sole_child_count;
			ancestor = next, sole_child_count = 1) {}

	bc = &ancestor->se.subtree;

	if (burst_cache_expired(bc, now)) {
		struct task_struct *direct_child;
		struct task_struct *descendant, *next_descendant;
		u32 count = 0, total = 0, scan_count = 0;
		for_each_child_task(ancestor, direct_child) {
			if (count >= BURST_CACHE_SAMPLE_LIMIT) break;
			if (scan_count++ >= BURST_CACHE_SCAN_LIMIT) break;

			descendant = direct_child;
			while (count_children_upto2(descendant) == 1) {
				next_descendant =
					list_first_entry_or_null(&descendant->children,
											struct task_struct, sibling);
				if (!next_descendant) break;
				descendant = next_descendant;
			}

			if (!task_is_bore_eligible(descendant)) continue;
			count++;
			total += descendant->se.penalty;
		}

		update_burst_cache(bc, ancestor, count, total, now);
	}

	bc_val.value = READ_ONCE(bc->value);
	return (u32)bc_val.penalty;
}

static u32 inherit_from_thread_group(struct task_struct *p, u64 now) {
	struct bore_bc bc_val;
	struct task_struct *leader = p->group_leader;
	struct bore_bc *bc = &leader->se.group;

	if (burst_cache_expired(bc, now)) {
		struct task_struct *sibling;
		u32 count = 0, total = 0, scan_count = 0;

		rcu_read_lock();
		for_each_thread(leader, sibling) {
			if (count >= BURST_CACHE_SAMPLE_LIMIT) break;
			if (scan_count++ >= BURST_CACHE_SCAN_LIMIT) break;

			if (!task_is_bore_eligible(sibling)) continue;
			count++;
			total += sibling->se.penalty;
		}
		rcu_read_unlock();

		update_burst_cache(bc, leader, count, total, now);
	}

	bc_val.value = READ_ONCE(bc->value);
	return (u32)bc_val.penalty;
}

void task_fork_bore(struct task_struct *p,
			struct task_struct *parent, u64 clone_flags, u64 now) {
	struct sched_entity *se = &p->se;
	u32 inherited_penalty;

	if (!static_branch_likely(&sched_bore_key) || !task_is_bore_eligible(p))
		return;

	if (clone_flags & CLONE_THREAD) {
		inherited_penalty = inherit_from_thread_group(parent, now);
	} else {
		read_lock(&tasklist_lock);
		if (!static_branch_likely(&sched_burst_inherit_key))
			inherited_penalty = 0;
		else
			inherited_penalty = static_branch_likely(&sched_burst_ancestor_key)?
				inherit_from_ancestor_hub(parent, clone_flags, now):
				inherit_from_parent(parent, clone_flags, now);
		read_unlock(&tasklist_lock);
	}

	if (se->prev_penalty < inherited_penalty)
		se->prev_penalty = inherited_penalty;
	se->curr_penalty = 0;
	se->burst_time   = 0;
	se->stop_update  = false;
	update_penalty(p);
}

void reset_task_bore(struct task_struct *p) {
	struct sched_entity *se = &p->se;

	se->burst_time   = 0;
	se->prev_penalty = 0;
	se->curr_penalty = 0;
	se->penalty      = 0;
	se->stop_update  = false;
	se->subtree.value = 0;
	se->group.value   = 0;
}

static void update_inherit_type(void) {
	switch(sched_burst_inherit_type) {
	case 1:
		static_branch_enable(&sched_burst_inherit_key);
		static_branch_disable(&sched_burst_ancestor_key);
		break;
	case 2:
		static_branch_enable(&sched_burst_inherit_key);
		static_branch_enable(&sched_burst_ancestor_key);
		break;
	default:
		static_branch_disable(&sched_burst_inherit_key);
		break;
	}
}

void __init sched_init_bore(void) {
	int i;

	printk(KERN_INFO "%s %s by %s\n",
		SCHED_BORE_PROGNAME, SCHED_BORE_VERSION, SCHED_BORE_AUTHOR);

	for (i = 1; i <= BURST_CACHE_SAMPLE_LIMIT; i++)
		bore_reciprocal_lut[i] = (u32)div64_u64(0xffffffffULL + i, i);

	reset_task_bore(&init_task);
	update_inherit_type();
}

static void readjust_all_task_weights(void) {
	struct task_struct *task;
	struct rq *rq;
	struct rq_flags rf;

	write_lock_irq(&tasklist_lock);
	for_each_process(task) {
		if (!task_is_bore_eligible(task)) continue;
		rq = task_rq_lock(task, &rf);
		update_rq_clock(rq);
		reweight_task_by_prio(task, effective_prio_bore(task));
		task_rq_unlock(rq, task, &rf);
	}
	write_unlock_irq(&tasklist_lock);
}

int sched_bore_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos) {
	int ret = proc_dou8vec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	if (sched_bore)
		static_branch_enable(&sched_bore_key);
	else
		static_branch_disable(&sched_bore_key);

	readjust_all_task_weights();

	return 0;
}

int sched_burst_inherit_type_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos) {
	int ret = proc_dou8vec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	update_inherit_type();

	return 0;
}

#ifdef CONFIG_SYSCTL
static struct ctl_table sched_bore_sysctls[] = {
	{
		.procname	= "sched_bore",
		.data		= &sched_bore,
		.maxlen		= sizeof(u8),
		.mode		= 0644,
		.proc_handler = sched_bore_update_handler,
		.extra1		= SYSCTL_ZERO,
		.extra2		= SYSCTL_ONE,
	},
	{
		.procname	= "sched_burst_inherit_type",
		.data		= &sched_burst_inherit_type,
		.maxlen		= sizeof(u8),
		.mode		= 0644,
		.proc_handler = sched_burst_inherit_type_update_handler,
		.extra1		= SYSCTL_ZERO,
		.extra2		= SYSCTL_TWO,
	},
	{
		.procname	= "sched_burst_smoothness",
		.data		= &sched_burst_smoothness,
		.maxlen		= sizeof(u8),
		.mode		= 0644,
		.proc_handler = proc_dou8vec_minmax,
		.extra1		= SYSCTL_ZERO,
		.extra2		= &three,
	},
	{
		.procname	= "sched_burst_penalty_offset",
		.data		= &sched_burst_penalty_offset,
		.maxlen		= sizeof(u8),
		.mode		= 0644,
		.proc_handler = proc_dou8vec_minmax,
		.extra1		= SYSCTL_ZERO,
		.extra2		= &maxval_6_bits,
	},
	{
		.procname	= "sched_burst_penalty_scale",
		.data		= &sched_burst_penalty_scale,
		.maxlen		= sizeof(uint),
		.mode		= 0644,
		.proc_handler = proc_douintvec_minmax,
		.extra1		= SYSCTL_ZERO,
		.extra2		= &maxval_12_bits,
	},
	{
		.procname	= "sched_burst_cache_lifetime",
		.data		= &sched_burst_cache_lifetime,
		.maxlen		= sizeof(uint),
		.mode		= 0644,
		.proc_handler = proc_douintvec,
	},
};

static int __init sched_bore_sysctl_init(void) {
	register_sysctl_init("kernel", sched_bore_sysctls);
	return 0;
}
late_initcall(sched_bore_sysctl_init);

#endif // CONFIG_SYSCTL
#endif // CONFIG_SCHED_BORE
