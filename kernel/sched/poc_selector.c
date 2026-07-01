// SPDX-License-Identifier: GPL-2.0
/*
 * POC (Piece-Of-Cake) CPU Selector — 5.10 backport
 *
 * Fast idle CPU selector using per-LLC atomic64 bitmaps.
 * Replaces the linear scan in select_idle_cpu() with O(1) bitmap lookup.
 *
 * Inspired by firelzrd's poc-selector (github.com/firelzrd/poc-selector)
 * which was itself inspired by the scx_cake BPF scheduler.
 */

#include "sched.h"

#ifdef CONFIG_SCHED_POC_SELECTOR

int poc_selector_enabled __read_mostly = 1;

/*
 * Called from idle.c when a CPU enters the idle loop.
 * Set this CPU's bit in its LLC's idle bitmap.
 */
void poc_set_cpu_idle(int cpu, int state)
{
	struct sched_domain *sd;
	u64 mask;

	if (!poc_selector_enabled)
		return;

	rcu_read_lock_sched();

	sd = rcu_dereference_sched(*per_cpu_ptr(&sd_llc, cpu));
	if (!sd || !sd->shared || !sd->shared->poc_fast_eligible) {
		rcu_read_unlock_sched();
		return;
	}

	mask = 1ULL << (cpu - sd->shared->poc_cpu_base);
	if (state)
		atomic64_or(mask, &sd->shared->poc_idle_cpus);
	else
		atomic64_andnot(mask, &sd->shared->poc_idle_cpus);

	rcu_read_unlock_sched();
}

/*
 * Fast idle CPU lookup — O(1) via ctz of bitmap intersected with affinity.
 * Returns -1 if no idle CPU found, falls back to standard linear scan.
 */
int poc_select_idle_cpu(struct sched_domain *sd, struct cpumask *cpus)
{
	struct sched_domain_shared *sd_share = sd->shared;
	u64 bitmap, mask, lo, hi;
	int base, base_word, shift, cpu;

	if (!poc_selector_enabled)
		return -1;

	/* No shared domain state, or LLC too wide for the bitmap */
	if (!sd_share || !sd_share->poc_fast_eligible)
		return -1;

	/*
	 * Build affinity mask: task's cpus_ptr ∩ LLC span, realigned from
	 * absolute cpumask bit positions to LLC-relative bit positions
	 * (bit 0 == poc_cpu_base). poc_cpu_base is not guaranteed to be
	 * 64-aligned, so a plain word fetch is not enough — shift in the
	 * carry from the next word whenever base isn't word-aligned.
	 */
	base = sd_share->poc_cpu_base;
	base_word = base >> 6;
	shift = base & 63;

	lo = cpumask_bits(cpus)[base_word];
	if (shift) {
		hi = cpumask_bits(cpus)[base_word + 1];
		mask = (lo >> shift) | (hi << (64 - shift));
	} else {
		mask = lo;
	}

	/* Intersect with idle bitmap */
	bitmap = atomic64_read(&sd_share->poc_idle_cpus) & mask;
	if (!bitmap)
		return -1;

	/* Find first idle CPU in bitmap */
	cpu = base + __builtin_ctzll(bitmap);

	/* Verify the CPU is actually available */
	if (available_idle_cpu(cpu))
		return cpu;

	/* State changed between read and check — let caller fall back */
	return -1;
}

/*
 * Called from topology.c at boot to initialize per-LLC bitmap state.
 * sd_id is the first CPU in this LLC domain, sd_span is the LLC CPU mask.
 */
void poc_init_llc(struct sched_domain_shared *sd_share, int sd_id,
		  const struct cpumask *sd_span)
{
	int range = cpumask_last(sd_span) - sd_id + 1;

	sd_share->poc_cpu_base = sd_id;
	atomic64_set(&sd_share->poc_idle_cpus, 0);

	if (range <= 64) {
		sd_share->poc_fast_eligible = true;
	} else {
		sd_share->poc_fast_eligible = false;
	}

	pr_info("POC: LLC %d-%d %s\n", sd_id,
		cpumask_last(sd_span),
		sd_share->poc_fast_eligible ? "O(1) fast" : "fallback");
}

#endif /* CONFIG_SCHED_POC_SELECTOR */