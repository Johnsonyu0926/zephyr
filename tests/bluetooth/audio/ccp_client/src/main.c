/* main.c - Application main entry point */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/audio/ccp.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/fff.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/ztest_test.h>
#include <zephyr/ztest_assert.h>

#include "conn.h"
#include "ccp_client.h"
#include "expects_util.h"
#include "test_common.h"

DEFINE_FFF_GLOBALS;

struct ccp_client_test_suite_fixture {
	/** Need 1 additional bearer than the max to trigger some corner cases */
	struct bt_ccp_client_bearer *bearers[CONFIG_BT_CCP_CLIENT_BEARER_COUNT + 1];
	struct bt_ccp_client *client_inst;
	struct bt_conn conn;
};

static void mock_init_rule_before(const struct ztest_unit_test *test, void *fixture)
{
	test_mocks_init();
}

static void mock_destroy_rule_after(const struct ztest_unit_test *test, void *fixture)
{
	test_mocks_cleanup();
}

ZTEST_RULE(mock_rule, mock_init_rule_before, mock_destroy_rule_after);

static void *ccp_client_test_suite_setup(void)
{
	struct ccp_client_test_suite_fixture *fixture;

	fixture = malloc(sizeof(*fixture));
	zassert_not_null(fixture);

	return fixture;
}

static void ccp_client_test_suite_before(void *f)
{
	struct ccp_client_test_suite_fixture *fixture = f;

	memset(fixture, 0, sizeof(*fixture));
	test_conn_init(&fixture->conn);
}

static void ccp_client_test_suite_after(void *f)
{
	struct ccp_client_test_suite_fixture *fixture = f;

	(void)bt_ccp_client_unregister_cb(&mock_ccp_client_cb);
	mock_bt_conn_disconnected(&fixture->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void ccp_client_test_suite_teardown(void *f)
{
	free(f);
}

ZTEST_SUITE(ccp_client_test_suite, NULL, ccp_client_test_suite_setup, ccp_client_test_suite_before,
	    ccp_client_test_suite_after, ccp_client_test_suite_teardown);

static ZTEST_F(ccp_client_test_suite, test_ccp_client_register_cb)
{
	int err;

	err = bt_ccp_client_register_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);
}

static ZTEST_F(ccp_client_test_suite, test_ccp_client_register_cb_inval_param_null)
{
	int err;

	err = bt_ccp_client_register_cb(NULL);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);
}

static ZTEST_F(ccp_client_test_suite, test_ccp_client_register_cb_inval_double_register)
{
	int err;

	err = bt_ccp_client_register_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	err = bt_ccp_client_register_cb(&mock_ccp_client_cb);
	zassert_equal(-EEXIST, err, "Unexpected return value %d", err);
}

static ZTEST_F(ccp_client_test_suite, test_ccp_client_unregister_cb)
{
	int err;

	err = bt_ccp_client_register_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	err = bt_ccp_client_unregister_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);
}

static ZTEST_F(ccp_client_test_suite, test_ccp_client_unregister_cb_inval_param_null)
{
	int err;

	err = bt_ccp_client_unregister_cb(NULL);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);
}

static ZTEST_F(ccp_client_test_suite, test_ccp_client_unregister_cb_inval_double_unregister)
{
	int err;

	err = bt_ccp_client_register_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	err = bt_ccp_client_unregister_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	err = bt_ccp_client_unregister_cb(&mock_ccp_client_cb);
	zassert_equal(-EALREADY, err, "Unexpected return value %d", err);
}

static ZTEST_F(ccp_client_test_suite, test_ccp_client_discover)
{
	int err;

	err = bt_ccp_client_register_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	err = bt_ccp_client_discover(&fixture->conn, &fixture->client_inst);
	zassert_equal(0, err, "Unexpected return value %d", err);

	/* Validate that we got the callback with valid values */
	zexpect_call_count("bt_ccp_client_cb.discover", 1,
			   mock_ccp_client_discover_cb_fake.call_count);
	zassert_not_null(mock_ccp_client_discover_cb_fake.arg0_history[0]);
	zassert_equal(0, mock_ccp_client_discover_cb_fake.arg1_history[0]);
	zassert_not_null(mock_ccp_client_discover_cb_fake.arg2_history[0]);

#if defined(CONFIG_BT_TBS_CLIENT_GTBS)
	zassert_not_null(mock_ccp_client_discover_cb_fake.arg2_history[0]->gtbs_bearer);
#endif /* CONFIG_BT_TBS_CLIENT_GTBS */

#if defined(CONFIG_BT_TBS_CLIENT_TBS)
	zassert_equal(CONFIG_BT_TBS_CLIENT_MAX_TBS_INSTANCES,
		      mock_ccp_client_discover_cb_fake.arg2_history[0]->tbs_count);
	zassert_not_null(mock_ccp_client_discover_cb_fake.arg2_history[0]->tbs_bearers);
#endif /* CONFIG_BT_TBS_CLIENT_TBS */
}

static ZTEST_F(ccp_client_test_suite, test_ccp_client_discover_inval_param_null_conn)
{
	int err;

	err = bt_ccp_client_register_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	err = bt_ccp_client_discover(NULL, &fixture->client_inst);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);
}

static ZTEST_F(ccp_client_test_suite, test_ccp_client_discover_inval_param_null_client)
{
	int err;

	err = bt_ccp_client_register_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	err = bt_ccp_client_discover(&fixture->conn, NULL);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);
}

static ZTEST_F(ccp_client_test_suite, test_ccp_client_get_bearers)
{
	struct bt_ccp_client_bearers bearers;
	int err;

	err = bt_ccp_client_register_cb(&mock_ccp_client_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	err = bt_ccp_client_discover(&fixture->conn, &fixture->client_inst);
	zassert_equal(0, err, "Unexpected return value %d", err);

	err = bt_ccp_client_get_bearers(fixture->client_inst, &bearers);
	zassert_equal(0, err, "Unexpected return value %d", err);

#if defined(CONFIG_BT_TBS_CLIENT_GTBS)
	zassert_not_null(bearers.gtbs_bearer);
#endif /* CONFIG_BT_TBS_CLIENT_GTBS */

#if defined(CONFIG_BT_TBS_CLIENT_TBS)
	zassert_equal(CONFIG_BT_TBS_CLIENT_MAX_TBS_INSTANCES, bearers.tbs_count);
	zassert_not_null(bearers.tbs_bearers);
#endif /* CONFIG_BT_TBS_CLIENT_TBS */
}
