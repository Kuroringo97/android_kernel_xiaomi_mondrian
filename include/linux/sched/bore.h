/* SPDX-License-Identifier: GPL-2.0 */
/*
 * include/linux/sched/bore.h
 *
 * Burst-Oriented Response Enhancer (BORE) CPU Scheduler
 * Copyright (C) 2021-2025 Masahito Suzuki <firelzrd@gmail.com>
 */
#ifndef _LINUX_SCHED_BORE_H
#define _LINUX_SCHED_BORE_H

#include <linux/init.h>
#include <linux/types.h>
#include <linux/jump_label.h>
#include <linux/sched.h>

#define SCHED_BORE_AUTHOR   "Masahito Suzuki"
#define SCHED_BORE_PROGNAME "BORE CPU Scheduler modification"

#define SCHED_BORE_VERSION  "6.8.0"

struct ctl_table;
struct cfs_rq;
struct sched_entity;

extern u8   __read_mostly sched_bore;
DECLARE_STATIC_KEY_TRUE(sched_bore_key);
extern u8   __read_mostly sched_burst_inherit_type;
DECLARE_STATIC_KEY_TRUE(sched_burst_inherit_key);
DECLARE_STATIC_KEY_TRUE(sched_burst_ancestor_key);
extern u8   __read_mostly sched_burst_smoothness;
extern u8   __read_mostly sched_burst_penalty_offset;
extern uint __read_mostly sched_burst_penalty_scale;
extern uint __read_mostly sched_burst_cache_lifetime;

static inline u8 bore_score(struct task_struct *p)
{ return p->se.penalty >> 8; }

u8 effective_prio_bore(struct task_struct *p);
void update_curr_bore(struct task_struct *p, u64 delta_exec);
void restart_burst_bore(struct task_struct *p);
void task_fork_bore(struct task_struct *p, struct task_struct *parent,
		    u64 clone_flags, u64 now);
void sched_init_bore(void) __init;
void reset_task_bore(struct task_struct *p);
void reweight_task_by_prio(struct task_struct *p, int prio);
void reweight_entity(struct cfs_rq *cfs_rq, struct sched_entity *se,
		     unsigned long weight);

int sched_bore_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos);
int sched_burst_inherit_type_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos);

#endif /* _LINUX_SCHED_BORE_H */
