// SPDX-License-Identifier: GPL-2.0
/*
 * kernel/sched/bore.c
 *
 * Burst-Oriented Response Enhancer (BORE) CPU Scheduler
 * Copyright (C) 2021-2024 Masahito Suzuki <firelzrd@gmail.com>
 */
#include "sched.h"

#include <linux/ktime.h>
#include <linux/sysctl.h>
#include <linux/sched/bore.h>

#ifdef CONFIG_SCHED_BORE
u8   __read_mostly sched_bore                   = 1;
u8   __read_mostly sched_burst_exclude_kthreads = 1;
u8   __read_mostly sched_burst_smoothness_long  = 1;
u8   __read_mostly sched_burst_smoothness_short = 0;
u8   __read_mostly sched_burst_fork_atavistic   = 2;
u8   __read_mostly sched_burst_penalty_offset   = 22;
uint __read_mostly sched_burst_penalty_scale    = 1280;
uint __read_mostly sched_burst_cache_lifetime   = 60000000;
#endif // CONFIG_SCHED_BORE

#ifdef CONFIG_SCHED_BORE
#define MAX_BURST_PENALTY (39U <<2)

static inline u32 log2plus1_u64_u32f8(u64 v) {
	u32 msb = fls64(v);
	u8 fractional = (v << (64 - msb) >> 55);
	return msb << 8 | fractional;
}

static inline u32 calc_burst_penalty(u64 burst_time) {
	u32 greed, tolerance, penalty, scaled_penalty;
	
	greed = log2plus1_u64_u32f8(burst_time);
	tolerance = sched_burst_penalty_offset << 8;
	penalty = max(0, (s32)(greed - tolerance));
	scaled_penalty = penalty * sched_burst_penalty_scale >> 16;

	return min(MAX_BURST_PENALTY, scaled_penalty);
}

u8 effective_prio_bore(struct task_struct *p) {
	u8 prio = p->static_prio - MAX_RT_PRIO;
	if (likely(sched_bore))
		prio += p->se.burst_score;
	return min(39, (int)prio);
}

void update_burst_penalty(struct sched_entity *se) {
	se->curr_burst_penalty = calc_burst_penalty(se->burst_time);
	se->burst_penalty = max(se->prev_burst_penalty, se->curr_burst_penalty);
	update_burst_score(se);
}

static inline u32 binary_smooth(u32 new, u32 old) {
	int increment = new - old;
		return (0 <= increment)?
	old + ( increment >> (int)sched_burst_smoothness_long):
	old - (-increment >> (int)sched_burst_smoothness_short);
}

void restart_burst(struct sched_entity *se) {
	se->burst_penalty = se->prev_burst_penalty =
		binary_smooth(se->curr_burst_penalty, se->prev_burst_penalty);
	se->curr_burst_penalty = 0;
	se->burst_time = 0;
	update_burst_score(se);
}

static void reset_task_weights_bore(void) {
	struct task_struct *task;
	struct rq *rq;
	struct rq_flags rf;

	write_lock_irq(&tasklist_lock);
	for_each_process(task) {
		rq = task_rq(task);
		rq_lock_irqsave(rq, &rf);
		reweight_task_by_prio(task, effective_prio_bore(task));
		rq_unlock_irqrestore(rq, &rf);
	}
	write_unlock_irq(&tasklist_lock);
}

int sched_bore_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos) {
	int ret = proc_dou8vec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	reset_task_weights_bore();

	return 0;
}
#endif // CONFIG_SCHED_BORE

#ifdef CONFIG_SCHED_BORE
extern u8   sched_burst_fork_atavistic;
extern uint sched_burst_cache_lifetime;

void __init sched_init_bore(void) {
	init_task.se.burst_time = 0;
	init_task.se.prev_burst_penalty = 0;
	init_task.se.curr_burst_penalty = 0;
	init_task.se.burst_penalty = 0;
	init_task.se.burst_score = 0;
	init_task.se.child_burst_last_cached = 0;
}

static u32 count_child_tasks(struct task_struct *p) {
	struct task_struct *child;
	u32 cnt = 0;
	list_for_each_entry(child, &p->children, sibling) {cnt++;}
	return cnt;
}

static inline bool task_burst_inheritable(struct task_struct *p) {
	return (p->sched_class == &fair_sched_class);
}

static inline bool child_burst_cache_expired(struct task_struct *p, u64 now) {
	u64 expiration_time =
		p->se.child_burst_last_cached + sched_burst_cache_lifetime;
	return ((s64)(expiration_time - now) < 0);
}

static void __update_child_burst_cache(
		struct task_struct *p, u32 cnt, u32 sum, u64 now) {
	u8 avg = 0;
	if (cnt) avg = sum / cnt;
	p->se.child_burst = max(avg, p->se.burst_penalty);
	p->se.child_burst_cnt = cnt;
	p->se.child_burst_last_cached = now;
}

static inline void update_child_burst_direct(struct task_struct *p, u64 now) {
	struct task_struct *child;
	u32 cnt = 0, sum = 0;

	list_for_each_entry(child, &p->children, sibling) {
		if (!task_burst_inheritable(child)) continue;
		cnt++;
		sum += child->se.burst_penalty;
	}

	__update_child_burst_cache(p, cnt, sum, now);
}

static inline u8 __inherit_burst_direct(struct task_struct *p, u64 now) {
	if (child_burst_cache_expired(p, now))
		update_child_burst_direct(p, now);

	return p->se.child_burst;
}

static void update_child_burst_topological(
	struct task_struct *p, u64 now, u32 depth, u32 *acnt, u32 *asum) {
	struct task_struct *child, *dec;
	u32 cnt = 0, dcnt = 0, sum = 0;

	list_for_each_entry(child, &p->children, sibling) {
		dec = child;
		while ((dcnt = count_child_tasks(dec)) == 1)
			dec = list_first_entry(&dec->children, struct task_struct, sibling);
		
		if (!dcnt || !depth) {
			if (!task_burst_inheritable(dec)) continue;
			cnt++;
			sum += dec->se.burst_penalty;
			continue;
		}
		if (!child_burst_cache_expired(dec, now)) {
			cnt += dec->se.child_burst_cnt;
			sum += (u32)dec->se.child_burst * dec->se.child_burst_cnt;
			continue;
		}
		update_child_burst_topological(dec, now, depth - 1, &cnt, &sum);
	}

	__update_child_burst_cache(p, cnt, sum, now);
	*acnt += cnt;
	*asum += sum;
}

static inline u8 __inherit_burst_topological(struct task_struct *p, u64 now) {
	struct task_struct *anc = p;
	u32 cnt = 0, sum = 0;

	while (anc->real_parent != anc && count_child_tasks(anc) == 1)
		anc = anc->real_parent;

	if (child_burst_cache_expired(anc, now))
		update_child_burst_topological(
			anc, now, sched_burst_fork_atavistic - 1, &cnt, &sum);

	return anc->se.child_burst;
}

static inline void inherit_burst(struct task_struct *p, struct task_struct *parent) {
	u8 burst_cache;
	u64 now = ktime_get_ns();

	read_lock(&tasklist_lock);
	burst_cache = likely(sched_burst_fork_atavistic)?
		__inherit_burst_topological(parent, now):
		__inherit_burst_direct(parent, now);
	read_unlock(&tasklist_lock);

	p->se.prev_burst_penalty = max(p->se.prev_burst_penalty, burst_cache);
}

static u8 inherit_from_thread_group(struct task_struct *p) {
	struct task_struct *leader = p->group_leader;
	struct task_struct *sibling;
	u32 cnt = 0, sum = 0;

	rcu_read_lock();
	for_each_thread(leader, sibling) {
		if (!task_burst_inheritable(sibling)) continue;
		cnt++;
		sum += sibling->se.burst_penalty;
	}
	rcu_read_unlock();

	return cnt ? sum / cnt : 0;
}

void sched_fork_bore(struct task_struct *p, struct task_struct *parent,
                     u64 clone_flags) {
	p->se.burst_time = 0;
	p->se.curr_burst_penalty = 0;
	p->se.child_burst_last_cached = 0;
	p->se.burst_score = 0;

	if (task_burst_inheritable(p)) {
		if (clone_flags & CLONE_THREAD) {
			u8 burst_cache = inherit_from_thread_group(p);
			p->se.prev_burst_penalty =
				max(p->se.prev_burst_penalty, burst_cache);
		} else {
			inherit_burst(p, parent);
		}
	}
	p->se.burst_penalty = p->se.prev_burst_penalty;
}
#endif // CONFIG_SCHED_BORE
