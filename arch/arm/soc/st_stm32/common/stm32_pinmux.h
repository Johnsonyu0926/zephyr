/*
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32_PINMUX_H_
#define _STM32_PINMUX_H_

/* Alternate functions */
enum stm32_pin_alt_func {
	STM32_PINMUX_FUNC_ALT_0 = 0, /* GPIO */
	STM32_PINMUX_FUNC_ALT_1,
	STM32_PINMUX_FUNC_ALT_2,
	STM32_PINMUX_FUNC_ALT_3,
	STM32_PINMUX_FUNC_ALT_4,
	STM32_PINMUX_FUNC_ALT_5,
	STM32_PINMUX_FUNC_ALT_6,
	STM32_PINMUX_FUNC_ALT_7,
	STM32_PINMUX_FUNC_ALT_8,
	STM32_PINMUX_FUNC_ALT_9,
	STM32_PINMUX_FUNC_ALT_10,
	STM32_PINMUX_FUNC_ALT_11,
	STM32_PINMUX_FUNC_ALT_12,
	STM32_PINMUX_FUNC_ALT_13,
	STM32_PINMUX_FUNC_ALT_14,
	STM32_PINMUX_FUNC_ALT_15,
	STM32_PINMUX_FUNC_ALT_16,
	STM32_PINMUX_FUNC_ALT_MAX
};

#endif /* _STM32_PINMUX_H_ */
