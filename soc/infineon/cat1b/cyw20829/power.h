/*
 * Copyright (c) 2021 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFX_POWER_H
#define __IFX_POWER_H

void get_sleep_info(uint32_t *sleepattempts, uint32_t *sleepcounts, uint32_t *deepsleepattempts,
		    uint32_t *deepsleepcounts, int64_t *sleeptime);

#endif
