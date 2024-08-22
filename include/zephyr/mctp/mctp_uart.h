/*
 * Copyright (c) 2024 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_MCTP_UART_H_
#define ZEPHYR_MCTP_UART_H_

#include <libmctp.h>

/**
 * Provides a MCTP bus binding for Zephyr's UART interface
 */
struct mctp_uart {
	struct mctp_bus_binding binding;
	const struct device *uart;
};

int mctp_uart_start(struct mctp_binding *binding);
int mctp_uart_tx(struct mctp_binding *binding, struct mctp_pktbuf *pkt);

/**
 * Statically define a MCTP bus binding for a UART
 *
 * @param name Symbolic name of the bus binding variable
 * @param dt_node Devicetree node
 */
#define MCTP_UART_DT_DEFINE(name, dt_node)			\
	struct mctp_uart name = {				\
		.binding = {					\
			.name = STRINGIFY(name),		\
			.version = 1,				\
			.pkt_size = MCTP_PACKET_SIZE(MCTP_BTU), \
			.pkt_header = 0,			\
			.pkt_trailer = 0,			\
			.start = mctp_uart_start,		\
			.tx = mctp_uart_tx,			\
		},						\
		.uart = DEVICE_DT_GET(dt_node),		\
	};

#endif /* ZEPHYR_MCTP_UART_H_ */
