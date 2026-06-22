// SPDX-License-Identifier: GPL-2.0
/*
 * BOSS: Burst Optimized Scenario Scheduler
 *
 * Bridges CASS placement awareness with BORE burst penalization.
 *
 * CASS selects the best CPU for a task and knows that CPU's raw
 * capacity (cap_orig). BOSS uses this to classify the placement
 * into a tier (LITTLE/BIG/PRIME) based on actual hardware topology
 * and stores it in task->boss_placement_tier.
 *
 * BORE's burst penalty is then scaled by this tier: tasks placed
 * on high-capacity cores are penalized less for bursting, since
 * bursting on a Prime core is expected and efficient behaviour.
 * Tasks placed on efficiency cores receive the full BORE penalty.
 *
 * CPU tiers are derived at boot from arch_scale_cpu_capacity() —
 * no hardcoded values, works on any SoC topology.
 *
 * Result: CASS and BORE become mutually aware. CPU placement
 * informs burst policy. Neither subsystem is modified beyond a
 * single read/write per scheduling event.
 */

#include <linux/sched.h>
#include <linux/sysctl.h>
#include "sched.h"
#include "boss.h"

/* ── Global state ───────────────────────────────────── */
bool boss_enabled = true;

/* Capacity thresholds derived at boot from CPU topology */
static unsigned long boss_tier_cap[BOSS_TIER_MAX];

/* ── Tier map construction ──────────────────────────── */
/*
 * Collect unique CPU capacity values, sort ascending, then map to tiers:
 *
 *   n=1: single cluster          → all LITTLE (degenerate)
 *   n=2: two clusters            → LITTLE + PRIME (no BIG)
 *   n=3: three clusters (SM8475) → LITTLE(A510) + BIG(A710) + PRIME(X3)
 *   n≥4: first→LITTLE, last→PRIME, all others→BIG
 *
 * Sorting uses a simple bubble sort — runs exactly once at boot,
 * operates on at most NR_CPUS entries (typically 3 on mobile SoCs).
 */
static void boss_build_tier_map(void)
{
	unsigned long sorted[NR_CPUS] = {};
	int cpu, n = 0, i, j;

	for_each_possible_cpu(cpu) {
		unsigned long c = arch_scale_cpu_capacity(cpu);
		bool dup = false;

		for (i = 0; i < n; i++) {
			if (sorted[i] == c) {
				dup = true;
				break;
			}
		}
		if (!dup)
			sorted[n++] = c;
	}

	/* Sort ascending */
	for (i = 0; i < n - 1; i++) {
		for (j = i + 1; j < n; j++) {
			if (sorted[i] > sorted[j])
				swap(sorted[i], sorted[j]);
		}
	}

	boss_tier_cap[BOSS_TIER_LITTLE] = sorted[0];
	boss_tier_cap[BOSS_TIER_BIG]    = (n >= 3) ? sorted[n - 2] : sorted[0];
	boss_tier_cap[BOSS_TIER_PRIME]  = sorted[n - 1];
}

static inline u8 boss_cap_to_tier(unsigned long cap_orig)
{
	if (cap_orig >= boss_tier_cap[BOSS_TIER_PRIME])
		return BOSS_TIER_PRIME;
	if (cap_orig >= boss_tier_cap[BOSS_TIER_BIG])
		return BOSS_TIER_BIG;
	return BOSS_TIER_LITTLE;
}

/* ── Placement update (called from cass.c) ──────────── */
/*
 * Called by CASS after selecting the target CPU for a task.
 * Classifies cap_orig into a tier and caches it on the task.
 * One WRITE_ONCE — zero cost on the read side in update_burst_score().
 */
void boss_update_placement_tier(struct task_struct *p,
				unsigned long cap_orig)
{
	WRITE_ONCE(p->boss_placement_tier, boss_cap_to_tier(cap_orig));
}

/* ── Sysctl ─────────────────────────────────────────── */
static struct ctl_table boss_table[] = {
	{
		.procname	= "sched_boss_enabled",
		.data		= &boss_enabled,
		.maxlen		= sizeof(bool),
		.mode		= 0644,
		.proc_handler	= proc_dobool,
	},
	{ }
};

/* ── Init ───────────────────────────────────────────── */
/*
 * late_initcall: CPU topology (arch_scale_cpu_capacity) is populated
 * by core_initcall. Using late_initcall guarantees valid capacity
 * values when boss_build_tier_map() runs.
 *
 * early_initcall would fire before topology is set up and return
 * wrong/zero capacities, building a broken tier map silently.
 */
static int __init boss_init(void)
{
	boss_build_tier_map();
	pr_info("BOSS (Burst Optimized Scenario Scheduler): "
		"LITTLE<=%lu BIG<=%lu PRIME<=%lu\n",
		boss_tier_cap[BOSS_TIER_LITTLE],
		boss_tier_cap[BOSS_TIER_BIG],
		boss_tier_cap[BOSS_TIER_PRIME]);
	register_sysctl("kernel", boss_table);
	return 0;
}
late_initcall(boss_init);
