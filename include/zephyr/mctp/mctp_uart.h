/*
 * Copyright (c) 2024 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_MCTP_UART_H_
#define ZEPHYR_MCTP_UART_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <libmctp.h>

/**
 * @brief An MCTP binding for Zephyr's polling UART interface
 */
struct mctp_binding_uart {
	/** @cond INTERNAL_HIDDEN */
	struct mctp_binding binding;
	const struct device *dev;

	/* receive buffer and state */
	uint8_t rx_buf[1024];
	struct mctp_pktbuf *rx_pkt;
	uint8_t rx_exp_len;
	uint16_t rx_fcs;
	uint16_t rx_fcs_calc;
	enum {
		STATE_WAIT_SYNC_START,
		STATE_WAIT_REVISION,
		STATE_WAIT_LEN,
		STATE_DATA,
		STATE_DATA_ESCAPED,
		STATE_WAIT_FCS1,
		STATE_WAIT_FCS2,
		STATE_WAIT_SYNC_END,
	} rx_state;
	int rx_res;
	/* Given to the receiving thread once a mctp
	 * message is available
	 */
	struct k_sem *rx_sem;

	/* staging buffer for tx */
	uint8_t tx_buf[256];
	int tx_res;
	/* Given to the transmitting thread once
	 * a mctp message is finished sending
	 */
	struct k_sem *tx_sem;
	/** @endcond INTERNAL_HIDDEN */
};

/**
 * @brief Poll the UART for a single character
 *
 * Polls the underlying UART peripheral for a single byte of information. If a byte
 * is available it is fed into an internal state machine that decodes an MCTP packet.
 *
 * When a full packet becomes available, mctp_bus_rx is called internally eventually leading
 * back to the assigned binding's rx_control function pointer callback.
 *
 * @param uart MCTP UART binding
 * @param pkt MCTP packet to transmit
 *
 * @retval 0 success
 * @retval -errno Error
 */
int mctp_uart_poll(struct mctp_binding_uart *uart);

/** @cond INTERNAL_HIDDEN */
int mctp_uart_start(struct mctp_binding *binding);
int mctp_uart_tx(struct mctp_binding *binding, struct mctp_pktbuf *pkt);
/** @endcond INTERNAL_HIDDEN */

/**
 * @brief Statically define a MCTP bus binding for a UART
 *
 * @param name Symbolic name of the bus binding variable
 * @param dt_node Devicetree node
 */
#define MCTP_UART_DT_DEFINE(_name, dt_node)                                                        \
	K_SEM_DEFINE(_name##_rx_sem, 0, 1); \
	K_SEM_DEFINE(_name##_tx_sem, 0, 1); \
	struct mctp_binding_uart _name = {                                                         \
		.binding =                                                                         \
			{                                                                          \
				.name = STRINGIFY(_name), .version = 1,                            \
						  .pkt_size = MCTP_PACKET_SIZE(MCTP_BTU),          \
						  .pkt_header = 0, .pkt_trailer = 0,               \
						  .start = mctp_uart_start, .tx = mctp_uart_tx,    \
				},                                                                 \
				.dev = dt_node,                                                    \
				.rx_state = STATE_WAIT_SYNC_START,                                 \
				.rx_pkt = NULL,                                                    \
				.rx_res = 0, \
				.rx_sem = &_name##_rx_sem, \
				.tx_res = 0, \
				.tx_sem = &_name##_tx_sem, \
	};

#endif /* ZEPHYR_MCTP_UART_H_ */
