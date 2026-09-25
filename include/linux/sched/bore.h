/* SPDX-License-Identifier: GPL-2.0 */
/*
 * include/linux/sched/bore.h
 *
 * Burst-Oriented Response Enhancer (BORE) CPU Scheduler
 * Copyright (C) 2021-2024 Masahito Suzuki <firelzrd@gmail.com>
 */
#ifndef _LINUX_SCHED_BORE_H
#define _LINUX_SCHED_BORE_H

#include <linux/init.h>
#include <linux/types.h>

struct ctl_table;
struct cfs_rq;
struct sched_entity;
struct task_struct;

extern u8   sched_bore;
extern u8   sched_burst_exclude_kthreads;
extern u8   sched_burst_smoothness_long;
extern u8   sched_burst_smoothness_short;
extern u8   sched_burst_fork_atavistic;
extern u8   sched_burst_penalty_offset;
extern uint sched_burst_penalty_scale;
extern uint sched_burst_cache_lifetime;

void sched_init_bore(void) __init;

u8 effective_prio_bore(struct task_struct *p);
void reweight_task_by_prio(struct task_struct *p, int prio);
void reweight_entity(struct cfs_rq *cfs_rq, struct sched_entity *se,
		     unsigned long weight);
void update_burst_score(struct sched_entity *se);
void update_burst_penalty(struct sched_entity *se);
void restart_burst(struct sched_entity *se);

int sched_bore_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos);

#endif /* _LINUX_SCHED_BORE_H */
