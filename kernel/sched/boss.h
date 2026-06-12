/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BOSS: Burst Optimized Scenario Scheduler
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

/*
 * Thermal throttle threshold: if cap_max drops below 50% of cap_orig,
 * the tier is downgraded. Above 50%, penalty is interpolated.
 * Using fixed-point: threshold = cap_orig >> 1 (i.e. 50%).
 * Stored as SCHED_CAPACITY_SCALE (1024) based values.
 */
#define BOSS_THERMAL_DOWNGRADE_SHIFT	1	/* 50% threshold */

extern int boss_enabled;

/* Defined in boss.c - tier capacity thresholds */
extern unsigned long boss_tier_cap[BOSS_TIER_MAX];

/* Called from cass.c after cass_best_cpu() selects the target CPU */
void boss_update_placement_tier(struct task_struct *p,
				unsigned long cap_orig,
				unsigned long cap_max);

/*
 * boss_get_tier_cap_orig() - get original capacity for a given tier.
 * Returns the boot-time capacity threshold for the tier, clamped to u16.
 * This replaces the incorrect SCHED_CAPACITY_SCALE proxy that made
 * thermal scaling inaccurate for Little/Big cores.
 */
static inline u16 boss_get_tier_cap_orig(u8 tier)
{
	if (unlikely(tier >= BOSS_TIER_MAX))
		return SCHED_CAPACITY_SCALE;

	return (u16)min(boss_tier_cap[tier], (unsigned long)U16_MAX);
}

/*
 * boss_thermal_scale() - compute effective penalty scale with thermal
 * awareness.
 *
 * Hybrid approach:
 * If cap_max < 50% of cap_orig → downgrade tier (hard step).
 * Otherwise → interpolate between current tier scale and next
 *             lower tier scale, weighted by throttle severity.
 *
 * throttle_ratio = cap_max / cap_orig in [0, 256] fixed point.
 * At ratio=256 (no throttle): full current tier scale applies.
 * At ratio=128 (50% throttle): boundary — downgrade kicks in.
 * Between 128..256: linearly blend toward lower tier scale.
 */
static inline u16 boss_thermal_scale(u8 tier, u16 cap_orig, u16 cap_max)
{
	u16 scale_curr, scale_lower;
	u32 throttle_ratio;

	/* No throttle or cap_orig zero guard */
	if (!cap_orig || cap_max >= cap_orig)
		return boss_penalty_scale[tier];

	/* throttle_ratio in [0..256]: 256 = unthrottled, 0 = fully throttled */
	throttle_ratio = ((u32)cap_max << 8) / cap_orig;

	/* Severe throttle: downgrade tier */
	if (throttle_ratio < 128) {
		tier = (tier > BOSS_TIER_LITTLE) ? tier - 1 : BOSS_TIER_LITTLE;
		return boss_penalty_scale[tier];
	}

	/* Mild throttle: interpolate between current and lower tier */
	scale_curr  = boss_penalty_scale[tier];
	scale_lower = (tier > BOSS_TIER_LITTLE)
		? boss_penalty_scale[tier - 1]
		: boss_penalty_scale[BOSS_TIER_LITTLE];

	/*
	 * throttle_ratio is in [128..256].
	 * Remap to [0..128]: 0 = heavy throttle, 128 = no throttle.
	 * blend = (scale_lower * (128 - t) + scale_curr * t) / 128
	 * where t = throttle_ratio - 128.
	 */
	{
		u32 t = throttle_ratio - 128;
		return (u16)((scale_lower * (128 - t) + scale_curr * t) >> 7);
	}
}

/*
 * Called from BORE's update_burst_score() to scale the raw burst
 * penalty by the task's placement tier and thermal state.
 * Reads p->boss_placement_tier and p->boss_cap_max (both written on
 * wakeup by CASS via boss_update_placement_tier()).
 * No locks, no branches beyond the enabled check.
 */
static inline u8 boss_scale_burst_score(struct task_struct *p, u8 raw_score)
{
	u8  tier;
	u16 cap_max, cap_orig, scale;

	if (!boss_enabled)
		return raw_score;

	tier    = READ_ONCE(p->boss_placement_tier);
	cap_max = READ_ONCE(p->boss_cap_max);

	/* Bounds check: default to full penalty on unexpected value */
	if (unlikely(tier >= BOSS_TIER_MAX))
		tier = BOSS_TIER_LITTLE;

	/*
	 * FIX: Use actual tier capacity instead of SCHED_CAPACITY_SCALE proxy.
	 * Previously used 1024 for all tiers, making thermal scaling wrong
	 * for Little (~300) and Big (~700) cores. Now uses boot-time
	 * boss_tier_cap[] values for accurate throttle ratio calculation.
	 */
	cap_orig = boss_get_tier_cap_orig(tier);
	scale = boss_thermal_scale(tier, cap_orig, cap_max);

	return (u8)(((u32)raw_score * scale) >> 8);
}

#else /* !CONFIG_SCHED_BOSS */

static inline void boss_update_placement_tier(struct task_struct *p,
					      unsigned long cap_orig,
					      unsigned long cap_max) {}

static inline u8 boss_scale_burst_score(struct task_struct *p, u8 raw_score)
{
	return raw_score;
}

#endif /* CONFIG_SCHED_BOSS */
#endif /* _SCHED_BOSS_H */