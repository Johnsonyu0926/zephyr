/*
 * Copyright (c) 2024 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/mctp/mctp_uart.h>
#include <crc-16-ccitt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mctp_uart, CONFIG_MCTP_LOG_LEVEL);

#define MCTP_UART_REVISION	 0x01
#define MCTP_UART_FRAMING_FLAG 0x7e
#define MCTP_UART_ESCAPE	 0x7d

struct mctp_serial_header {
	uint8_t flag;
	uint8_t revision;
	uint8_t len;
};

struct mctp_serial_trailer {
	uint8_t fcs_msb;
	uint8_t fcs_lsb;
	uint8_t flag;
};

static inline struct mctp_binding_uart *binding_to_uart(struct mctp_binding *b)
{
	return (struct mctp_binding_uart *)b;
}

static void mctp_uart_callback(const struct device *dev, struct uart_event *evt,
                               void *userdata)
{
	struct mctp_binding_uart *binding = userdata;

	switch (evt->type) {
	case UART_TX_DONE:
		binding->tx_res = 0;
		k_sem_give(binding->tx_sem);
		break;
	case UART_TX_ABORTED:
		binding->tx_res = -EABORT;
		k_sem_give(binding->tx_sem);
		break;
	case UART_RX_RDY:
		/* buffer being read into is ready */
		binding->rx_res = evt->rx.len;
		k_sem_give(binding->rx_sem);
		break;
	case UART_RX_BUF_REQUEST:
		/* Ignored */
		break;
	case UART_RX_BUF_RELEASED:
		/* Ignored */
		break;
	case UART_RX_STOPPED:
		/* Ignored */
		break;
	}	
}

static void mctp_uart_finish_pkt(struct mctp_binding_uart *uart,
				 bool valid)
{
	struct mctp_pktbuf *pkt = uart->rx_pkt;

	__ASSERT_NO_MSG(pkt);

	if (valid) {
		mctp_bus_rx(&uart->binding, pkt);
	}

	uart->rx_pkt = NULL;
}

static void mctp_uart_start_pkt(struct mctp_binding_uart *uart,
				uint8_t len)
{
	uart->rx_pkt = mctp_pktbuf_alloc(&uart->binding, len);
	__ASSERT_NO_MSG(uart->rx_pkt);
}

static size_t mctp_uart_pkt_escape(struct mctp_pktbuf *pkt, uint8_t *buf)
{
	uint8_t total_len;
	uint8_t *p;
	int i, j;

	total_len = pkt->end - pkt->mctp_hdr_off;

	p = (void *)mctp_pktbuf_hdr(pkt);

	for (i = 0, j = 0; i < total_len; i++, j++) {
		uint8_t c = p[i];

		if (c == 0x7e || c == 0x7d) {
			if (buf)
				buf[j] = 0x7d;
			j++;
			c ^= 0x20;
		}
		if (buf) {
			buf[j] = c;
		}
	}

	return j;
}

/*
 * Each byte coming from the uart is run through this state machine which
 * does the MCTP packet decoding.
 *
 * The actual packet and buffer being read into is owned by the binding!
 */
static void mctp_uart_consume(struct mctp_binding_uart *uart, uint8_t c)
{
	struct mctp_pktbuf *pkt = uart->rx_pkt;
	bool valid = false;

	LOG_DBG("state: %d, char 0x%02x", uart->rx_state, c);

	__ASSERT_NO_MSG(!pkt == (uart->rx_state == STATE_WAIT_SYNC_START ||
			uart->rx_state == STATE_WAIT_REVISION ||
			uart->rx_state == STATE_WAIT_LEN));

	switch (uart->rx_state) {
	case STATE_WAIT_SYNC_START:
		if (c != MCTP_UART_FRAMING_FLAG) {
			LOG_DBG("lost sync, dropping packet");
			if (pkt)
				mctp_uart_finish_pkt(uart, false);
		} else {
			uart->rx_state = STATE_WAIT_REVISION;
		}
		break;

	case STATE_WAIT_REVISION:
		if (c == MCTP_UART_REVISION) {
			uart->rx_state = STATE_WAIT_LEN;
			uart->rx_fcs_calc = crc_16_ccitt_byte(FCS_INIT_16, c);
		} else if (c == MCTP_UART_FRAMING_FLAG) {
			/* Handle the case where there are bytes dropped in request,
			 * and the state machine is out of sync. The failed request's
			 * trailing footer i.e. 0x7e would be interpreted as next
			 * request's framing footer. So if we are in STATE_WAIT_REVISION
			 * and receive 0x7e byte, then contine to stay in
			 * STATE_WAIT_REVISION
			 */
			LOG_DBG(
				"Received serial framing flag 0x%02x while waiting"
				" for serial revision 0x%02x.",
				c, MCTP_UART_REVISION);
		} else {
			LOG_DBG("invalid revision 0x%02x", c);
			uart->rx_state = STATE_WAIT_SYNC_START;
		}
		break;
	case STATE_WAIT_LEN:
		if (c > uart->binding.pkt_size ||
		    c < sizeof(struct mctp_hdr)) {
			LOG_DBG("invalid size %d", c);
			uart->rx_state = STATE_WAIT_SYNC_START;
		} else {
			mctp_uart_start_pkt(uart, 0);
			pkt = uart->rx_pkt;
			uart->rx_exp_len = c;
			uart->rx_state = STATE_DATA;
			uart->rx_fcs_calc =
				crc_16_ccitt_byte(uart->rx_fcs_calc, c);
		}
		break;

	case STATE_DATA:
		if (c == MCTP_UART_ESCAPE) {
			uart->rx_state = STATE_DATA_ESCAPED;
		} else {
			mctp_pktbuf_push(pkt, &c, 1);
			uart->rx_fcs_calc =
				crc_16_ccitt_byte(uart->rx_fcs_calc, c);
			if (pkt->end - pkt->mctp_hdr_off == uart->rx_exp_len)
				uart->rx_state = STATE_WAIT_FCS1;
		}
		break;

	case STATE_DATA_ESCAPED:
		c ^= 0x20;
		mctp_pktbuf_push(pkt, &c, 1);
		uart->rx_fcs_calc = crc_16_ccitt_byte(uart->rx_fcs_calc, c);
		if (pkt->end - pkt->mctp_hdr_off == uart->rx_exp_len)
			uart->rx_state = STATE_WAIT_FCS1;
		else
			uart->rx_state = STATE_DATA;
		break;

	case STATE_WAIT_FCS1:
		uart->rx_fcs = c << 8;
		uart->rx_state = STATE_WAIT_FCS2;
		break;
	case STATE_WAIT_FCS2:
		uart->rx_fcs |= c;
		uart->rx_state = STATE_WAIT_SYNC_END;
		break;

	case STATE_WAIT_SYNC_END:
		if (uart->rx_fcs == uart->rx_fcs_calc) {
			if (c == MCTP_UART_FRAMING_FLAG) {
				valid = true;
			} else {
				valid = false;
				LOG_DBG("missing end frame marker");
			}
		} else {
			valid = false;
			LOG_DBG("invalid fcs : 0x%04x, expect 0x%04x",
				     uart->rx_fcs, uart->rx_fcs_calc);
		}

		mctp_uart_finish_pkt(uart, valid);
		uart->rx_state = STATE_WAIT_SYNC_START;
		break;
	}

	LOG_DBG(" -> state: %d", uart->rx_state);
}

/* Should be called initially and after each mctp message is received */
void mctp_uart_rx_enable(struct mctp_binding_uart *uart)
{
	uart_rx_enable(uart->dev, &uart->rx_buf, sizeof(uart->rx_buf), K_MSEC(1));
}

int mctp_uart_tx(struct mctp_binding *b, struct mctp_pktbuf *pkt)
{
	struct mctp_binding_uart *uart = binding_to_uart(b);
	struct mctp_serial_header *hdr;
	struct mctp_serial_trailer *tlr;
	uint8_t *buf;
	size_t len;
	uint16_t fcs;

	/* the length field in the header excludes serial framing
	 * and escape sequences
	 */
	len = mctp_pktbuf_size(pkt);

	hdr = (void *)uart->tx_buf;
	hdr->flag = MCTP_UART_FRAMING_FLAG;
	hdr->revision = MCTP_UART_REVISION;
	hdr->len = len;

	/* Calculate fcs */
	fcs = crc_16_ccitt(FCS_INIT_16, (const uint8_t *)hdr + 1, 2);
	fcs = crc_16_ccitt(fcs, (const uint8_t *)mctp_pktbuf_hdr(pkt), len);

	buf = (void *)(hdr + 1);

	len = mctp_uart_pkt_escape(pkt, NULL);
	if (len + sizeof(*hdr) + sizeof(*tlr) > sizeof(uart->txbuf))
		return -EMSGSIZE;

	mctp_uart_pkt_escape(pkt, buf);

	buf += len;

	tlr = (void *)buf;
	tlr->flag = MCTP_UART_FRAMING_FLAG;
	tlr->fcs_msb = fcs >> 8;
	tlr->fcs_lsb = fcs & 0xff;

	len += sizeof(*hdr) + sizeof(*tlr);

	uart_tx(uart->dev, uart->tx_buf, len, K_FOREVER);
	k_sem_take(uart->tx_sem, K_FOREVER);
	return uart->tx_res;
}

int mctp_uart_start(struct mctp_binding *binding)
{
	mctp_binding_set_tx_enabled(binding, true);
	return 0;
}
