/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BOSS: Burst Optimized Scenario Scheduler
 *
 * Header: tier definitions, penalty scale table, and inline helpers.
 * boss.c implements the tier map construction and placement update.
 * fair.c calls boss_scale_burst_score() inside update_burst_score().
 * cass.c calls boss_update_placement_tier() after CPU selection.
 */
#ifndef _SCHED_BOSS_H
#define _SCHED_BOSS_H

#ifdef CONFIG_SCHED_BOSS

/* ── Placement tiers ────────────────────────────────── */
#define BOSS_TIER_LITTLE   0
#define BOSS_TIER_BIG      1
#define BOSS_TIER_PRIME    2
#define BOSS_TIER_MAX      3

/*
 * Penalty scale per tier, out of 256 = 100%.
 *
 * MUST be u16: LITTLE scale is 256, which overflows u8 (max 255)
 * and would silently truncate to 0, zeroing all LITTLE-tier penalties.
 *
 * LITTLE: full BORE penalty  — bursting on E-core is genuinely greedy
 * BIG:    63% BORE penalty   — mid-tier burst is expected work
 * PRIME:  25% BORE penalty   — Prime core exists specifically for burst
 */
static const u16 boss_penalty_scale[BOSS_TIER_MAX] = {
	[BOSS_TIER_LITTLE] = 256,	/* 100% */
	[BOSS_TIER_BIG]    = 160,	/*  63% */
	[BOSS_TIER_PRIME]  = 64,	/*  25% */
};

extern bool boss_enabled;

/* Called from cass.c after cass_best_cpu() selects the target CPU */
void boss_update_placement_tier(struct task_struct *p,
				unsigned long cap_orig);

/*
 * Called from BORE's update_burst_score() to scale the raw burst
 * penalty by the task's placement tier before it is applied.
 *
 * One READ_ONCE on p->boss_placement_tier (u8, written on wakeup).
 * No locks, no branches beyond the enabled check.
 */
static inline u8 boss_scale_burst_score(struct task_struct *p, u8 raw_score)
{
	u8  tier;
	u16 scale;

	if (!boss_enabled)
		return raw_score;

	tier = READ_ONCE(p->boss_placement_tier);

	/* Bounds check: default to full penalty on unexpected value */
	if (unlikely(tier >= BOSS_TIER_MAX))
		tier = BOSS_TIER_LITTLE;

	scale = boss_penalty_scale[tier];

	return (u8)(((u32)raw_score * scale) >> 8);
}

#else /* !CONFIG_SCHED_BOSS */

static inline void boss_update_placement_tier(struct task_struct *p,
					      unsigned long cap_orig) {}

static inline u8 boss_scale_burst_score(struct task_struct *p, u8 raw_score)
{
	return raw_score;
}

#endif /* CONFIG_SCHED_BOSS */
#endif /* _SCHED_BOSS_H */
