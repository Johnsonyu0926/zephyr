/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nordic_nrf_hsfll

#include "clock_control_nrf2_common.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(clock_control_nrf2, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#include <hal/nrf_hsfll.h>

/* TODO: add support for other HSFLLs */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "multiple instances not supported");

#include <hal/nrf_hsfll.h>

#ifdef CONFIG_NRFS_HAS_DVFS_SERVICE
#include <ld_dvfs_handler.h>

#define FLAG_FREQ_CHANGE_CB_EXPECTED BIT(FLAGS_COMMON_BITS)

#define NRFS_DVFS_TIMEOUT K_MSEC(CONFIG_CLOCK_CONTROL_NRF2_NRFS_DVFS_TIMEOUT_MS)

static const struct clock_options {
	uint32_t frequency;
	enum dvfs_frequency_setting setting;
} clock_options[] = {
	/* TODO: move to devicetree? */
	{
		.frequency = MHZ(64),
		.setting = DVFS_FREQ_LOW,
	},
	{
		.frequency = MHZ(128),
		.setting = DVFS_FREQ_MEDLOW,
	},
	{
		.frequency = MHZ(320),
		.setting = DVFS_FREQ_HIGH,
	},
};
static const uint32_t max_frequency = MHZ(320);

struct hsfll_dev_data {
	NRF2_STRUCT_CLOCK_CONFIG(hsfll, ARRAY_SIZE(clock_options)) clk_cfg;
	struct k_timer timer;
};
#endif /* CONFIG_NRFS_HAS_DVFS_SERVICE */

struct hsfll_dev_config {
	NRF_HSFLL_Type *hsfll;
	uint32_t trim_vsup;
	uint32_t trim_coarse;
	uint32_t trim_fine;
	uint8_t multiplier;
};

#ifdef CONFIG_NRFS_HAS_DVFS_SERVICE
static void freq_setting_applied_cb(enum dvfs_frequency_setting new_setting)
{
	ARG_UNUSED(new_setting);

	const struct device *dev = DEVICE_DT_INST_GET(0);
	struct hsfll_dev_data *dev_data = dev->data;
	atomic_val_t prev_flags;

	/* Process only expected notifications (after sent requests). */
	prev_flags = atomic_and(&dev_data->clk_cfg.flags,
				~FLAG_FREQ_CHANGE_CB_EXPECTED);
	if (prev_flags & FLAG_FREQ_CHANGE_CB_EXPECTED) {
		k_timer_stop(&dev_data->timer);

		nrf2_clock_config_update_end(&dev_data->clk_cfg, 0);
	}
}

static void hsfll_update_timeout_handler(struct k_timer *timer)
{
	struct hsfll_dev_data *dev_data =
		CONTAINER_OF(timer, struct hsfll_dev_data, timer);

	nrf2_clock_config_update_end(&dev_data->clk_cfg, -ETIMEDOUT);
}

static void hsfll_work_handler(struct k_work *work)
{
	struct hsfll_dev_data *dev_data =
		CONTAINER_OF(work, struct hsfll_dev_data, clk_cfg.work);
	enum dvfs_frequency_setting required_setting;
	uint8_t to_activate_idx;
	int rc;

	to_activate_idx = nrf2_clock_config_update_begin(work);
	required_setting = clock_options[to_activate_idx].setting;

	/* Notify the DVFS service about the required setting. */
	rc = dvfs_service_handler_change_freq_setting(required_setting);
	if (rc < 0) {
		nrf2_clock_config_update_end(&dev_data->clk_cfg, rc);
		return;
	}

	/* And expect a confirmation that the setting is ready to be used. */
	(void)atomic_or(&dev_data->clk_cfg.flags, FLAG_FREQ_CHANGE_CB_EXPECTED);
	k_timer_start(&dev_data->timer, NRFS_DVFS_TIMEOUT, K_NO_WAIT);
}

static struct onoff_manager *hsfll_find_mgr(const struct device *dev,
					    const struct nrf_clock_spec *spec)
{
	struct hsfll_dev_data *dev_data = dev->data;
	uint32_t frequency;

	if (!spec) {
		return &dev_data->clk_cfg.onoff[0].mgr;
	}

	if (spec->accuracy || spec->precision) {
		LOG_ERR("invalid specification of accuracy or precision");
		return NULL;
	}

	frequency = spec->frequency == NRF_CLOCK_CONTROL_FREQUENCY_MAX
		  ? max_frequency
		  : spec->frequency;

	for (int i = 0; i < ARRAY_SIZE(clock_options); ++i) {
		if (frequency > clock_options[i].frequency) {
			continue;
		}

		return &dev_data->clk_cfg.onoff[i].mgr;
	}

	LOG_ERR("invalid frequency");
	return NULL;
}
#endif /* CONFIG_NRFS_HAS_DVFS_SERVICE */

static int api_request_hsfll(const struct device *dev,
			     const struct nrf_clock_spec *spec,
			     struct onoff_client *cli)
{
#ifdef CONFIG_NRFS_HAS_DVFS_SERVICE
	struct onoff_manager *mgr = hsfll_find_mgr(dev, spec);

	if (mgr) {
		return onoff_request(mgr, cli);
	}

	return -EINVAL;
#else
	return -ENOTSUP;
#endif
}

static int api_release_hsfll(const struct device *dev,
			     const struct nrf_clock_spec *spec)
{
#ifdef CONFIG_NRFS_HAS_DVFS_SERVICE
	struct onoff_manager *mgr = hsfll_find_mgr(dev, spec);

	if (mgr) {
		return onoff_release(mgr);
	}

	return -EINVAL;
#else
	return -ENOTSUP;
#endif
}

static int api_cancel_or_release_hsfll(const struct device *dev,
				       const struct nrf_clock_spec *spec,
				       struct onoff_client *cli)
{
#ifdef CONFIG_NRFS_HAS_DVFS_SERVICE
	struct onoff_manager *mgr = hsfll_find_mgr(dev, spec);

	if (mgr) {
		return onoff_cancel_or_release(mgr, cli);
	}

	return -EINVAL;
#else
	return -ENOTSUP;
#endif
}

static int api_get_rate_hsfll(const struct device *dev,
			      clock_control_subsys_t sys,
			      uint32_t *rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	*rate = nrf_hsfll_clkctrl_mult_get(NRF_HSFLL) * MHZ(16);

	return 0;
}

static int hsfll_init(const struct device *dev)
{
	const struct hsfll_dev_config *dev_config = dev->config;
	nrf_hsfll_trim_t trim = {
		.vsup   = sys_read32(dev_config->trim_vsup),
		.coarse = sys_read32(dev_config->trim_coarse),
		.fine   = sys_read32(dev_config->trim_fine),
	};

	LOG_DBG("Trim: HSFLL VSUP: 0x%.8x",   trim.vsup);
	LOG_DBG("Trim: HSFLL COARSE: 0x%.8x", trim.coarse);
	LOG_DBG("Trim: HSFLL FINE: 0x%.8x",   trim.fine);

	nrf_hsfll_clkctrl_mult_set(dev_config->hsfll, dev_config->multiplier);
	nrf_hsfll_trim_set(dev_config->hsfll, &trim);

	nrf_hsfll_task_trigger(dev_config->hsfll, NRF_HSFLL_TASK_FREQ_CHANGE);
	/* HSFLL task frequency change needs to be triggered twice to take effect.*/
	nrf_hsfll_task_trigger(dev_config->hsfll, NRF_HSFLL_TASK_FREQ_CHANGE);

	LOG_DBG("NRF_HSFLL->TRIM.VSUP = %d",   dev_config->hsfll->TRIM.VSUP);
	LOG_DBG("NRF_HSFLL->TRIM.COARSE = %d", dev_config->hsfll->TRIM.COARSE);
	LOG_DBG("NRF_HSFLL->TRIM.FINE = %d",   dev_config->hsfll->TRIM.FINE);

#ifdef CONFIG_NRFS_HAS_DVFS_SERVICE
	struct hsfll_dev_data *dev_data = dev->data;
	int rc;

	rc = nrf2_clock_config_init(&dev_data->clk_cfg,
				    ARRAY_SIZE(dev_data->clk_cfg.onoff),
				    hsfll_work_handler);
	if (rc < 0) {
		return rc;
	}

	k_timer_init(&dev_data->timer, hsfll_update_timeout_handler, NULL);

	dvfs_service_handler_register_freq_setting_applied_callback(
		freq_setting_applied_cb);
#endif

	return 0;
}

static struct nrf_clock_control_driver_api hsfll_drv_api = {
	.std_api = {
		.on = api_nosys_on_off,
		.off = api_nosys_on_off,
		.get_rate = api_get_rate_hsfll,
	},
	.request = api_request_hsfll,
	.release = api_release_hsfll,
	.cancel_or_release = api_cancel_or_release_hsfll,
};

#define HSFLL_MUTLIPLIER(n) \
	DT_PROP(DT_DRV_INST(n), clock_frequency) / \
		DT_PROP(DT_CLOCKS_CTLR(DT_DRV_INST(n)), clock_frequency)

#define FICR_ADDR_GET(n, name) \
	DT_REG_ADDR(DT_PHANDLE_BY_NAME(DT_DRV_INST(n), nordic_ficrs, name)) + \
		DT_PHA_BY_NAME(DT_DRV_INST(n), nordic_ficrs, name, offset)

#ifdef CONFIG_NRFS_HAS_DVFS_SERVICE
static struct hsfll_dev_data hsfll_data;
#endif

static const struct hsfll_dev_config hsfll_config = {
	.hsfll = (NRF_HSFLL_Type *)DT_INST_REG_ADDR(0),
	.multiplier  = HSFLL_MUTLIPLIER(0),
	.trim_vsup   = FICR_ADDR_GET(0, vsup),
	.trim_coarse = FICR_ADDR_GET(0, coarse),
	.trim_fine   = FICR_ADDR_GET(0, fine),
};

DEVICE_DT_INST_DEFINE(0, hsfll_init, NULL,
		      COND_CODE_1(CONFIG_NRFS_HAS_DVFS_SERVICE,
				  (&hsfll_data),
				  (NULL)),
		      &hsfll_config,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &hsfll_drv_api);
