// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/bitfield.h>
#include <linux/cpufreq.h>
#include <linux/cpu_cooling.h>
#include <linux/energy_model.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msm_rtb.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/qcom-cpufreq-hw.h>
#include <linux/topology.h>
#include <linux/fie.h>

#define CREATE_TRACE_POINTS
#include <trace/events/dcvsh.h>

#define LUT_MAX_ENTRIES			40U
#define LUT_SRC				GENMASK(31, 30)
#define LUT_L_VAL			GENMASK(7, 0)
#define LUT_CORE_COUNT			GENMASK(18, 16)
#define LUT_VOLT			GENMASK(11, 0)
#define LUT_ROW_SIZE			32
#define CLK_HW_DIV			2
#define GT_IRQ_STATUS			BIT(2)
#define MAX_FN_SIZE			20
#define LIMITS_POLLING_DELAY_MS		4

#define CYCLE_CNTR_OFFSET(core_id, m, acc_count)		\
			(acc_count ? ((core_id + 1) * 4) : 0)

enum {
	REG_ENABLE,
	REG_FREQ_LUT,
	REG_VOLT_LUT,
	REG_PERF_STATE,
	REG_CYCLE_CNTR,
	REG_DOMAIN_STATE,
	REG_INTR_EN,
	REG_INTR_CLR,
	REG_INTR_STATUS,

	REG_ARRAY_SIZE,
};

static unsigned long cpu_hw_rate, xo_rate;
static const u16 *offsets;
static unsigned int lut_row_size = LUT_ROW_SIZE;
static unsigned int lut_max_entries = LUT_MAX_ENTRIES;
static bool accumulative_counter;
static bool perf_lock_support;

struct cpufreq_qcom {
	struct cpufreq_frequency_table *table;
	void __iomem *base;
	void __iomem *pdmem_base;
	cpumask_t related_cpus;
	unsigned long dcvsh_freq_limit;
	struct delayed_work freq_poll_work;
	struct mutex dcvsh_lock;
	struct device_attribute freq_limit_attr;
	int dcvsh_irq;
	char dcvsh_irq_name[MAX_FN_SIZE];
	bool is_irq_enabled;
	bool is_irq_requested;
	bool exited;
};

struct cpufreq_counter {
	u64 total_cycle_counter;
	u32 prev_cycle_counter;
	spinlock_t lock;
};

static struct cpufreq_counter qcom_cpufreq_counter[NR_CPUS];

struct qcom_cpufreq_soc_data {
	u32 reg_enable;
	u32 reg_domain_state;
	u32 reg_freq_lut;
	u32 reg_volt_lut;
	u32 reg_current_vote;
	u32 reg_perf_state;
	u32 reg_cycle_cntr;
	u32 reg_intr_status;
	u32 reg_intr_clear;
	u32 throttle_freq_mask;
	u8 lut_row_size;
	u8 throttle_irq_bit;
	bool accumulative_counter;
	bool turbo_ind_support;
};

struct qcom_cpufreq_data {
	void __iomem *base;
	struct resource *res;
	const struct qcom_cpufreq_soc_data *soc_data;

	/*
	 * Mutex to synchronize between de-init sequence and re-starting LMh
	 * polling/interrupts
	 */
	struct mutex throttle_lock;
	int throttle_irq;
	char irq_name[15];
	bool cancel_throttle;
	bool is_irq_requested;
	struct delayed_work throttle_work;
	struct cpufreq_policy *policy;
	u32 cpu;
	unsigned long freq;
	unsigned long max_capacity, capacity;

	cpu = cpumask_first(&c->related_cpus);
	policy = cpufreq_cpu_get_raw(cpu);
	capacity = max_capacity = arch_scale_cpu_capacity(cpu);

	if (limit) {
		freq = readl_relaxed(c->base + offsets[REG_DOMAIN_STATE]) &
				GENMASK(7, 0);
		freq = DIV_ROUND_CLOSEST_ULL(freq * xo_rate, 1000);
		if (policy) {
			capacity = freq * max_capacity;
			capacity /= policy->cpuinfo.max_freq;
		}
	} else {
		if (!policy)
			freq = U32_MAX;
		else
			freq = policy->cpuinfo.max_freq;
	}

	arch_set_thermal_pressure(&c->related_cpus, max_t(unsigned long, 0,
				  max_capacity - capacity));
	trace_dcvsh_freq(cpumask_first(&c->related_cpus), freq);
	c->dcvsh_freq_limit = freq;

	return freq;
}

static void limits_dcvsh_poll(struct work_struct *work)
{
	struct cpufreq_qcom *c = container_of(work, struct cpufreq_qcom,
						freq_poll_work.work);
	unsigned long freq_limit, dcvsh_freq;
	u32 regval, cpu;

	mutex_lock(&c->dcvsh_lock);

	if (c->exited)
		goto out;

	cpu = cpumask_first(&c->related_cpus);

	freq_limit = limits_mitigation_notify(c, true);

	dcvsh_freq = qcom_cpufreq_hw_get(cpu);

	if (freq_limit < dcvsh_freq) {
		mod_delayed_work(system_highpri_wq, &c->freq_poll_work,
				msecs_to_jiffies(LIMITS_POLLING_DELAY_MS));
	} else {
		/* Update scheduler for throttle removal */
		limits_mitigation_notify(c, false);

		regval = readl_relaxed(c->base + offsets[REG_INTR_CLR]);
		regval |= GT_IRQ_STATUS;
		writel_relaxed(regval, c->base + offsets[REG_INTR_CLR]);

		c->is_irq_enabled = true;
		enable_irq(c->dcvsh_irq);
	}

out:
	mutex_unlock(&c->dcvsh_lock);
}

static irqreturn_t dcvsh_handle_isr(int irq, void *data)
{
	struct cpufreq_qcom *c = data;
	u32 regval;

	regval = readl_relaxed(c->base + offsets[REG_INTR_STATUS]);
	if (!(regval & GT_IRQ_STATUS))
		return IRQ_HANDLED;

	mutex_lock(&c->dcvsh_lock);

	if (c->is_irq_enabled) {
		c->is_irq_enabled = false;
		disable_irq_nosync(c->dcvsh_irq);
		limits_mitigation_notify(c, true);
		mod_delayed_work(system_highpri_wq, &c->freq_poll_work,
				msecs_to_jiffies(LIMITS_POLLING_DELAY_MS));

	}

	mutex_unlock(&c->dcvsh_lock);

	return IRQ_HANDLED;
}

u64 qcom_cpufreq_get_cpu_cycle_counter(int cpu)
{
	struct cpufreq_counter *cpu_counter;
	struct cpufreq_policy *policy;
	u64 cycle_counter_ret;
	unsigned long flags;
	u16 offset;
	u32 val;

	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy)
		return 0;

	cpu_counter = &qcom_cpufreq_counter[cpu];
	spin_lock_irqsave(&cpu_counter->lock, flags);

	offset = CYCLE_CNTR_OFFSET(topology_core_id(cpu), policy->related_cpus,
					accumulative_counter);
	val = readl_relaxed_no_log(policy->driver_data +
				    offsets[REG_CYCLE_CNTR] + offset);

	if (val < cpu_counter->prev_cycle_counter) {
		/* Handle counter overflow */
		cpu_counter->total_cycle_counter += UINT_MAX -
			cpu_counter->prev_cycle_counter + val;
		cpu_counter->prev_cycle_counter = val;
	} else {
		cpu_counter->total_cycle_counter += val -
			cpu_counter->prev_cycle_counter;
		cpu_counter->prev_cycle_counter = val;
	}
	cycle_counter_ret = cpu_counter->total_cycle_counter;
	spin_unlock_irqrestore(&cpu_counter->lock, flags);

	pr_debug("CPU %u, core-id 0x%x, offset %u\n", cpu, topology_core_id(cpu), offset);

	return cycle_counter_ret;
}
EXPORT_SYMBOL_GPL(qcom_cpufreq_get_cpu_cycle_counter);

static int
qcom_cpufreq_hw_target_index(struct cpufreq_policy *policy,
			     unsigned int index)
{
	struct qcom_cpufreq_data *data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = data->soc_data;
	unsigned long freq = policy->freq_table[index].frequency;
	unsigned long flags;

	/*
	 * Disable IRQs around the frequency set so that the timestamp
	 * recorded by FIE is as close to the actual MMIO write as possible.
	 */
	local_irq_save(flags);
	writel_relaxed(index, data->base + soc_data->reg_perf_state);
	fie_rate_set(policy->cpu, freq);
	local_irq_restore(flags);

	if (icc_scaling_enabled)
		qcom_cpufreq_set_bw(policy, freq);

	return 0;
}

static unsigned int qcom_cpufreq_hw_get(unsigned int cpu)
{
	struct cpufreq_policy *policy;
	unsigned int index;

	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy)
		return 0;

	index = readl_relaxed(policy->driver_data + offsets[REG_PERF_STATE]);
	index = min(index, lut_max_entries - 1);

	return policy->freq_table[index].frequency;
}

static unsigned int
qcom_cpufreq_hw_fast_switch(struct cpufreq_policy *policy,
			    unsigned int target_freq)
{
	struct qcom_cpufreq_data *data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = data->soc_data;
	unsigned int index;
	unsigned int freq;

	index = policy->cached_resolved_idx;
	freq = policy->freq_table[index].frequency;

	writel_relaxed(index, data->base + soc_data->reg_perf_state);
	fie_rate_set(policy->cpu, freq);

	return freq;
}

static int qcom_cpufreq_hw_cpu_init(struct cpufreq_policy *policy)
{
	struct cpufreq_qcom *c;
	struct device *cpu_dev;
	int ret;
	struct qcom_cpufreq_data *drv_data = policy->driver_data;
	const struct qcom_cpufreq_soc_data *soc_data = drv_data->soc_data;

	table = kcalloc(LUT_MAX_ENTRIES + 1, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (!ret) {
		/* Disable all opps and cross-validate against LUT later */
		icc_scaling_enabled = true;
		for (rate = 0; ; rate++) {
			opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
			if (IS_ERR(opp))
				break;

			dev_pm_opp_put(opp);
			dev_pm_opp_disable(cpu_dev, rate);
		}
	} else if (ret != -ENODEV) {
		dev_err(cpu_dev, "Invalid opp table in device tree\n");
		kfree(table);
		return ret;
	} else {
		policy->fast_switch_possible = true;
		icc_scaling_enabled = false;
	}

	for (i = 0; i < LUT_MAX_ENTRIES; i++) {
		data = readl_relaxed(drv_data->base + soc_data->reg_freq_lut +
				      i * soc_data->lut_row_size);
		src = FIELD_GET(LUT_SRC, data);
		lval = FIELD_GET(LUT_L_VAL, data);
		core_count = FIELD_GET(LUT_CORE_COUNT, data);

		if (i == 0)
			max_cc = core_count;

		data = readl_relaxed(drv_data->base + soc_data->reg_volt_lut +
				      i * soc_data->lut_row_size);
		volt = FIELD_GET(LUT_VOLT, data) * 1000;

		if (src)
			freq = xo_rate * lval / 1000;
		else
			freq = cpu_hw_rate / 1000;

		if (core_count == LUT_TURBO_IND && soc_data->turbo_ind_support)
			table[i].frequency = CPUFREQ_ENTRY_INVALID;
		else if (freq != prev_freq) {
			if (!qcom_cpufreq_update_opp(cpu_dev, freq, volt)) {
				table[i].frequency = freq;
				if (core_count < max_cc)
					table[i].flags = CPUFREQ_BOOST_FREQ;
				dev_dbg(cpu_dev, "index=%d freq=%d, core_count %d\n", i,
				freq, core_count);
			} else {
				dev_warn(cpu_dev, "failed to update OPP for freq=%d\n", freq);
				table[i].frequency = CPUFREQ_ENTRY_INVALID;
			}
		}

		/*
		 * Two of the same frequencies with the same core counts means
		 * end of table
		 */
		if (i > 0 && prev_freq == freq) {
			struct cpufreq_frequency_table *prev = &table[i - 1];

			/*
			 * Only treat the last frequency that might be a boost
			 * as the boost frequency
			 */
			if (prev->frequency == CPUFREQ_ENTRY_INVALID) {
				if (!qcom_cpufreq_update_opp(cpu_dev, prev_freq, volt)) {
					prev->frequency = prev_freq;
					prev->flags = CPUFREQ_BOOST_FREQ;
				} else {
					dev_warn(cpu_dev, "failed to update OPP for freq=%d\n",
						 freq);
				}
			}

			break;
		}

		prev_freq = freq;
	}

	table[i].frequency = CPUFREQ_TABLE_END;
	policy->freq_table = table;

	for_each_cpu(cpu, policy->cpus) {
		per_cpu(cpufreq_boost_pcpu, cpu).data = drv_data;
		per_cpu(cpufreq_boost_pcpu, cpu).max_index = i - 1;
	}

	for (i = 0; i < LUT_MAX_ENTRIES && table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (table[i].flags == CPUFREQ_BOOST_FREQ)
			break;

		drv_data->last_non_boost_freq = table[i].frequency;
	}

	dev_pm_opp_set_sharing_cpus(cpu_dev, policy->cpus);

	return 0;
}

static void qcom_get_related_cpus(int index, struct cpumask *m)
{
	struct device_node *cpu_np;
	struct of_phandle_args args;
	int cpu, ret;

	for_each_possible_cpu(cpu) {
		cpu_np = of_cpu_device_node_get(cpu);
		if (!cpu_np)
			continue;

		ret = of_parse_phandle_with_args(cpu_np, "qcom,freq-domain",
				"#freq-domain-cells", 0, &args);
		of_node_put(cpu_np);
		if (ret < 0)
			continue;

		if (index == args.args[0])
			cpumask_set_cpu(cpu, m);
	}
}

static ssize_t dcvsh_freq_limit_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct qcom_cpufreq_data *c = container_of(attr, struct qcom_cpufreq_data,
						   freq_limit_attr);
	return scnprintf(buf, PAGE_SIZE, "%lu\n", c->dcvsh_freq_limit);
}

static unsigned long qcom_lmh_get_throttle_freq(struct qcom_cpufreq_data *data)
{
	unsigned int lval;

	if (data->soc_data->reg_current_vote)
		lval = readl_relaxed(data->base + data->soc_data->reg_current_vote) & 0x3ff;
	else
		lval = readl_relaxed(data->base + data->soc_data->reg_domain_state) & 0xff;

	return lval * xo_rate;
}

static void qcom_lmh_dcvs_notify(struct qcom_cpufreq_data *data)
{
	const struct qcom_cpufreq_soc_data *soc_data = data->soc_data;
	struct cpufreq_policy *policy = data->policy;
	int cpu = cpumask_first(policy->related_cpus);
	struct device *dev = get_cpu_device(cpu);
	unsigned long freq_hz, throttled_freq, thermal_pressure;
	struct dev_pm_opp *opp;
	u32 val;

	if (!dev)
		return;

	/*
	 * Get the h/w throttled frequency, normalize it using the
	 * registered opp table and use it to calculate thermal pressure.
	 */
	freq_hz = qcom_lmh_get_throttle_freq(data);

	opp = dev_pm_opp_find_freq_floor(dev, &freq_hz);
	if (IS_ERR(opp) && PTR_ERR(opp) == -ERANGE)
		opp = dev_pm_opp_find_freq_ceil(dev, &freq_hz);

	if (!IS_ERR(opp))
		dev_pm_opp_put(opp);

	throttled_freq = thermal_pressure = freq_hz / HZ_PER_KHZ;

	/*
	 * In the unlikely case policy is unregistered do not enable
	 * polling or h/w interrupt
	 */
	mutex_lock(&data->throttle_lock);
	if (data->cancel_throttle)
		goto out;

	/*
	 * If h/w throttled frequency is higher than what cpufreq has requested
	 * for, then stop polling and switch back to interrupt mechanism.
	 */
	if (throttled_freq >= qcom_cpufreq_hw_get(cpu)) {
		thermal_pressure = policy->cpuinfo.max_freq;

		val = readl_relaxed(data->base + soc_data->reg_intr_clear);
		val |= BIT(soc_data->throttle_irq_bit);
		writel_relaxed(val, data->base + soc_data->reg_intr_clear);

		enable_irq(data->throttle_irq);
		trace_dcvsh_throttle(cpu, 0);
	} else {
		/*
		 * If the frequency is at least the highest, non-boost
		 * frequency, then the delta vs. what's requested is likely due
		 * to core-count boost limitations and shouldn't be
		 * communicated as thermal pressure.
		 */
		if (throttled_freq >= data->last_non_boost_freq)
			thermal_pressure = policy->cpuinfo.max_freq;

		mod_delayed_work(system_highpri_wq, &data->throttle_work,
				 msecs_to_jiffies(10));
	}

	/*
	 * Route the LMh-reported throttled frequency through FIE's thermal
	 * pressure aggregation. FIE combines this with its own measured HW
	 * throttle detection for a more accurate thermal pressure report.
	 */
	fie_cpufreq_pressure(cpu, thermal_pressure >= policy->cpuinfo.max_freq ?
			     UINT_MAX : thermal_pressure);

	trace_dcvsh_freq(cpu, qcom_cpufreq_hw_get(cpu), throttled_freq);
	arch_update_thermal_pressure(policy->related_cpus, thermal_pressure);
	data->dcvsh_freq_limit = thermal_pressure;

out:
	mutex_unlock(&data->throttle_lock);
}

static void qcom_lmh_dcvs_poll(struct work_struct *work)
{
	struct qcom_cpufreq_data *data;

	data = container_of(work, struct qcom_cpufreq_data, throttle_work.work);
	qcom_lmh_dcvs_notify(data);
}

static irqreturn_t qcom_lmh_dcvs_handle_irq(int irq, void *data)
{
	struct qcom_cpufreq_data *c_data = data;
	const struct qcom_cpufreq_soc_data *soc_data = c_data->soc_data;
	struct cpufreq_policy *policy = c_data->policy;
	u32 val;

	val = readl_relaxed(c_data->base + soc_data->reg_intr_status);
	if (!(val & BIT(soc_data->throttle_irq_bit)))
		return IRQ_NONE;

	/* Disable interrupt and enable polling */
	disable_irq_nosync(c_data->throttle_irq);
	trace_dcvsh_throttle(cpumask_first(policy->cpus), 1);
	schedule_delayed_work(&c_data->throttle_work, 0);

	return IRQ_HANDLED;
}

static const struct qcom_cpufreq_soc_data qcom_soc_data = {
	.reg_enable = 0x0,
	.reg_freq_lut = 0x110,
	.reg_volt_lut = 0x114,
	.reg_current_vote = 0x704,
	.reg_intr_clear = 0x778,
	.reg_intr_status = 0x77c,
	.reg_perf_state = 0x920,
	.reg_cycle_cntr = 0x9c0,
	.lut_row_size = 32,
	.throttle_irq_bit = 1,
	.accumulative_counter = false,
	.turbo_ind_support = true,
};

static const struct qcom_cpufreq_soc_data epss_soc_data = {
	.reg_enable = 0x0,
	.reg_domain_state = 0x20,
	.reg_freq_lut = 0x100,
	.reg_volt_lut = 0x200,
	.reg_intr_clear = 0x308,
	.reg_intr_status = 0x30c,
	.reg_perf_state = 0x320,
	.reg_cycle_cntr = 0x3c4,
	.lut_row_size = 4,
	.throttle_irq_bit = 2,
	.accumulative_counter = true,
	.turbo_ind_support = false,
};

static const struct of_device_id qcom_cpufreq_hw_match[] = {
	{ .compatible = "qcom,cpufreq-hw", .data = &qcom_soc_data },
	{ .compatible = "qcom,cpufreq-epss", .data = &epss_soc_data },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_cpufreq_hw_match);

static int qcom_cpufreq_hw_lmh_init(struct cpufreq_policy *policy, int index,
				    struct device *cpu_dev)
{
	struct qcom_cpufreq_data *data = policy->driver_data;
	struct platform_device *pdev = cpufreq_get_driver_data();
	int ret;

	if (data->is_irq_requested)
		return 0;
	/*
	 * Look for LMh interrupt. If no interrupt line is specified /
	 * if there is an error, allow cpufreq to be enabled as usual.
	 */
	data->throttle_irq = platform_get_irq(pdev, index);
	if (data->throttle_irq <= 0)
		return data->throttle_irq == -EPROBE_DEFER ? -EPROBE_DEFER : 0;

	data->cancel_throttle = false;
	data->policy = policy;

	mutex_init(&data->throttle_lock);
	INIT_DELAYED_WORK(&data->throttle_work, qcom_lmh_dcvs_poll);

	snprintf(data->irq_name, sizeof(data->irq_name), "dcvsh-irq-%u", policy->cpu);
	ret = request_threaded_irq(data->throttle_irq, NULL, qcom_lmh_dcvs_handle_irq,
				   IRQF_ONESHOT, data->irq_name, data);
	if (ret) {
		dev_err(&pdev->dev, "Error registering %s: %d\n", data->irq_name, ret);
		return 0;
	}

	data->is_irq_requested = true;

	sysfs_attr_init(&data->freq_limit_attr.attr);
	data->freq_limit_attr.attr.name = "dcvsh_freq_limit";
	data->freq_limit_attr.show = dcvsh_freq_limit_show;
	data->freq_limit_attr.attr.mode = 0444;
	data->dcvsh_freq_limit = U32_MAX;
	device_create_file(cpu_dev, &data->freq_limit_attr);

	return 0;
}

static void qcom_cpufreq_hw_lmh_exit(struct qcom_cpufreq_data *data)
{
	struct cpufreq_policy *policy = data->policy;
	struct device *cpu_dev;

	if (data->throttle_irq <= 0)
		return;

	if (!policy)
		return;

	mutex_lock(&data->throttle_lock);
	data->cancel_throttle = true;
	mutex_unlock(&data->throttle_lock);

	free_irq(data->throttle_irq, data);
	data->is_irq_requested = false;
	cancel_delayed_work_sync(&data->throttle_work);

	arch_update_thermal_pressure(policy->related_cpus, policy->cpuinfo.max_freq);
	cpu_dev = get_cpu_device(cpumask_first(policy->related_cpus));
	device_remove_file(cpu_dev, &data->freq_limit_attr);
	trace_dcvsh_throttle(cpumask_first(policy->related_cpus), 0);
}

static int qcom_cpufreq_hw_cpu_init(struct cpufreq_policy *policy)
{
	struct platform_device *pdev = cpufreq_get_driver_data();
	struct device *dev = &pdev->dev;
	void __iomem *base;
	unsigned int max_freq;
	int i;
	struct qcom_cpufreq_data *data;
	int ret, index;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	offsets = of_device_get_match_data(&pdev->dev);
	if (!offsets)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, index);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	if (!of_property_read_bool(dev->of_node, "qcom,skip-enable-check")) {
		/* HW should be in enabled state to proceed */
		if (!(readl_relaxed(base + offsets[REG_ENABLE]) & 0x1)) {
			dev_err(dev, "Domain-%d cpufreq hardware not enabled\n",
				 index);
			return -ENODEV;
		}
	}

	accumulative_counter = !of_property_read_bool(dev->of_node,
						"qcom,no-accumulative-counter");
	c->base = base;

	qcom_get_related_cpus(index, &c->related_cpus);
	if (!cpumask_weight(&c->related_cpus)) {
		dev_err(dev, "Domain-%d failed to get related CPUs\n", index);
		return -ENONET;
	}

	ret = qcom_cpufreq_hw_read_lut(pdev, c, max_cores);
	if (ret) {
		dev_err(dev, "Domain-%d failed to read LUT\n", index);
		return ret;
	}

	/*
	 * Register this frequency domain with FIE now that the freq table is
	 * populated. Scan the table for the max frequency since cpuinfo.max_freq
	 * isn't set until after cpu_init returns.
	 */
	max_freq = 0;
	for (i = 0; policy->freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (policy->freq_table[i].frequency != CPUFREQ_ENTRY_INVALID &&
		    policy->freq_table[i].frequency > max_freq)
			max_freq = policy->freq_table[i].frequency;
	}
	fie_init_cpu_domain(policy->cpus, max_freq);


	ret = dev_pm_opp_get_opp_count(cpu_dev);
	if (ret <= 0) {
		dev_err(cpu_dev, "Failed to add OPPs\n");
		ret = -ENODEV;
		goto error;
	}

	if (policy_has_boost_freq(policy)) {
		ret = cpufreq_enable_boost_support();
		if (ret)
			dev_warn(cpu_dev, "failed to enable boost: %d\n", ret);
	}

	ret = qcom_cpufreq_hw_lmh_init(policy, index, cpu_dev);
	if (ret)
		goto error;

	return 0;
error:
	policy->driver_data = NULL;
	return ret;
}

static int qcom_cpufreq_hw_cpu_exit(struct cpufreq_policy *policy)
{
	struct device *cpu_dev = get_cpu_device(policy->cpu);

	qcom_cpufreq_hw_lmh_exit(policy->driver_data);
	dev_pm_opp_remove_all_dynamic(cpu_dev);
	dev_pm_opp_of_cpumask_remove_table(policy->related_cpus);
	kfree(policy->freq_table);

	return 0;
}

static struct freq_attr *qcom_cpufreq_hw_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	&cpufreq_freq_attr_scaling_boost_freqs,
	NULL
};

static struct cpufreq_driver cpufreq_qcom_hw_driver = {
	.flags		= CPUFREQ_NEED_INITIAL_FREQ_CHECK |
			  CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
			  CPUFREQ_IS_COOLING_DEV,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= qcom_cpufreq_hw_target_index,
	.get		= qcom_cpufreq_hw_get,
	.init		= qcom_cpufreq_hw_cpu_init,
	.exit		= qcom_cpufreq_hw_cpu_exit,
	.register_em	= cpufreq_register_em_with_opp,
	.fast_switch    = qcom_cpufreq_hw_fast_switch,
	.name		= "qcom-cpufreq-hw",
	.attr		= qcom_cpufreq_hw_attr,
	.boost_enabled	= true,
};

static int cpuhp_qcom_online(unsigned int cpu)
{
	struct qcom_cpufreq_boost *b = &per_cpu(cpufreq_boost_pcpu, cpu);
	struct qcom_cpufreq_data *data = b->data;

	if (!data)
		return 0;

	writel_relaxed(b->max_index, data->base + data->soc_data->reg_perf_state);
	return 0;
}

static int qcom_cpufreq_hw_driver_probe(struct platform_device *pdev)
{
	struct device *cpu_dev;
	struct clk *clk;
	unsigned int cpu;
	int ret;

	clk = clk_get(&pdev->dev, "xo");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	xo_rate = clk_get_rate(clk);
	clk_put(clk);

	clk = clk_get(&pdev->dev, "alternate");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	cpu_hw_rate = clk_get_rate(clk) / CLK_HW_DIV;
	clk_put(clk);

	of_property_read_u32(pdev->dev.of_node, "qcom,lut-row-size",
			      &lut_row_size);

	of_property_read_u32(pdev->dev.of_node, "qcom,lut-max-entries",
			      &lut_max_entries);

	for_each_possible_cpu(cpu) {
		cpu_np = of_cpu_device_node_get(cpu);
		if (!cpu_np) {
			dev_dbg(&pdev->dev, "Failed to get cpu %d device\n",
				cpu);
			continue;
		}

		ret = of_parse_phandle_with_args(cpu_np, "qcom,freq-domain",
						  "#freq-domain-cells", 0,
						   &args);
		of_node_put(cpu_np);
		if (ret)
			return ret;

		if (qcom_freq_domain_map[cpu])
			continue;

		ret = qcom_cpu_resources_init(pdev, cpu, args.args[0],
					      args.args[1]);
		if (ret)
			return ret;
	}

	return 0;
}

static int qcom_cpufreq_hw_driver_probe(struct platform_device *pdev)
{
	int rc, cpu;

	/* Get the bases of cpufreq for domains */
	rc = qcom_resources_init(pdev);
	if (rc) {
		dev_err(&pdev->dev, "CPUFreq resource init failed\n");
		return rc;
	}

	for_each_possible_cpu(cpu)
		spin_lock_init(&qcom_cpufreq_counter[cpu].lock);

	rc = cpufreq_register_driver(&cpufreq_qcom_hw_driver);
	if (rc) {
		dev_err(&pdev->dev, "CPUFreq HW driver failed to register\n");
		return rc;
	}

	of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	dev_dbg(&pdev->dev, "QCOM CPUFreq HW driver initialized\n");

	return 0;
}

static int qcom_cpufreq_hw_driver_remove(struct platform_device *pdev)
{
	struct cpufreq_qcom *c;
	struct device *cpu_dev;
	int cpu;

	for_each_possible_cpu(cpu) {
		cpu_dev = get_cpu_device(cpu);
		if (!cpu_dev)
			continue;

		dev_pm_opp_remove_all_dynamic(cpu_dev);

		c = qcom_freq_domain_map[cpu];
		if (c->dcvsh_irq > 0 && c->is_irq_requested) {
			devm_free_irq(cpu_dev, c->dcvsh_irq, c);
			device_remove_file(cpu_dev, &c->freq_limit_attr);
			c->is_irq_requested = false;
		}
	}

	return cpufreq_unregister_driver(&cpufreq_qcom_hw_driver);
}

static const struct of_device_id qcom_cpufreq_hw_match[] = {
	{ .compatible = "qcom,cpufreq-hw", .data = &cpufreq_qcom_std_offsets },
	{ .compatible = "qcom,cpufreq-hw-epss",
				   .data = &cpufreq_qcom_epss_std_offsets },
	{}
};

static struct platform_driver qcom_cpufreq_hw_driver = {
	.probe = qcom_cpufreq_hw_driver_probe,
	.remove = qcom_cpufreq_hw_driver_remove,
	.driver = {
		.name = "qcom-cpufreq-hw",
		.of_match_table = qcom_cpufreq_hw_match,
	},
};

static int __init qcom_cpufreq_hw_init(void)
{
	return platform_driver_register(&qcom_cpufreq_hw_driver);
}
subsys_initcall(qcom_cpufreq_hw_init);

static void __exit qcom_cpufreq_hw_exit(void)
{
	platform_driver_unregister(&qcom_cpufreq_hw_driver);
}
module_exit(qcom_cpufreq_hw_exit);

MODULE_DESCRIPTION("QCOM CPUFREQ HW Driver");
MODULE_LICENSE("GPL v2");
