/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SoC initialization for Silicon Labs Series 2 products
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <em_chip.h>

#include <sl_device_init_dcdc.h>
#include <sl_clock_manager_init.h>

#ifdef CONFIG_PM
#include <sl_hfxo_manager.h>
#include <sl_power_manager.h>
#endif

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

void soc_early_init_hook(void)
{
	/* Handle chip errata */
	CHIP_Init();

#if CONFIG_HW_HAS_SILABS_DCDC
	sl_device_init_dcdc();
#endif
	sl_clock_manager_init();

#ifdef CONFIG_PM
	sl_power_manager_init();
	sl_hfxo_manager_init();
#endif
}
