/*
 * Copyright (c) 2024 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <libmctp.h>
#include <zephyr/mctp/mctp_uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mctp_host);

#define LOCAL_HELLO_EID 20

#define REMOTE_HELLO_EID 10

static void rx_message(uint8_t eid, bool tag_owner,
		       uint8_t msg_tag, void *data, void *msg,
		       size_t len)
{
	LOG_INF("received message %s for endpoint %d, msg_tag %d, len %zu", (char *)msg, eid,
		msg_tag, len);
}

MCTP_UART_DT_DEFINE(mctp_host, DEVICE_DT_GET(DT_NODELABEL(arduino_serial)));

int main(void)
{
	printf("Hello MCTP! %s\n", CONFIG_BOARD_TARGET);

	int rc;
	struct mctp *mctp_ctx;

	mctp_ctx = mctp_init();
	assert(mctp_ctx != NULL);

	mctp_register_bus(mctp_ctx, &mctp_host.binding, LOCAL_HELLO_EID);
	mctp_set_rx_all(mctp_ctx, rx_message, NULL);

	/* MCTP poll loop, send "hello" and get "world" back */
	while (true) {
		mctp_message_tx(mctp_ctx, REMOTE_HELLO_EID, false, 0, "hello", sizeof("hello"));

		k_msleep(1);
		for (int i = 0; i < 10000; i++) {
			rc = mctp_uart_poll(mctp_host);
		}
		k_msleep(1000);
	}

	return 0;
}
