/*
 * Copyright (c) 2024 TOKITA Hiroshi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/ztest.h>

ZTEST(test_kconfig_functions, test_arithmetic)
{
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_ADD_10, 10);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_ADD_10_3, 10 + 3);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_ADD_10_3_2, 10 + 3 + 2);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_SUB_10, 10);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_SUB_10_3, 10 - 3);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_SUB_10_3_2, 10 - 3 - 2);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_MUL_10, 10);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_MUL_10_3, 10 * 3);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_MUL_10_3_2, 10 * 3 * 2);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_DIV_10, 10);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_DIV_10_3, 10 / 3);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_DIV_10_3_2, 10 / 3 / 2);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_MOD_10, 10);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_MOD_10_3, 10 % 3);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_MOD_10_3_2, 10 % 3 % 2);
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_INC_1, 1 + 1);
	zassert_str_equal(CONFIG_KCONFIG_ARITHMETIC_INC_1_1, "2,2");
	zassert_str_equal(CONFIG_KCONFIG_ARITHMETIC_INC_INC_1_1, "3,3");
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_DEC_1, 1 - 1);
	zassert_str_equal(CONFIG_KCONFIG_ARITHMETIC_DEC_1_1, "0,0");
	zassert_str_equal(CONFIG_KCONFIG_ARITHMETIC_DEC_DEC_1_1, "-1,-1");
	zassert_equal(CONFIG_KCONFIG_ARITHMETIC_ADD_INC_1_1, (1 + 1) + (1 + 1));
}

ZTEST_SUITE(test_kconfig_functions, NULL, NULL, NULL, NULL, NULL);
