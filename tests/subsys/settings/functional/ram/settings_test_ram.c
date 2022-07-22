/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2022 Nordic semiconductor ASA */

#include <zephyr/zephyr.h>
#include <ztest.h>
#include <errno.h>
#include <zephyr/settings/settings.h>

void test_setting_storage_get(void)
{
	int rc;
	void *storage;

	rc = settings_storage_get(&storage);
	zassert_equal(0, rc, "Can't fetch storage reference (err=%d)", rc);

	zassert_not_null(storage, "Null reference.");
}
