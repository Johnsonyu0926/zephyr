/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Periodic advertisements synchronization status. */
enum sync_status {
	SYNC_STAT_ALLOWED,
	SYNC_STAT_READY_OR_CONT_SCAN,
	SYNC_STAT_TERM
};

struct lll_sync {
	struct lll_hdr hdr;

	uint8_t access_addr[4];
	uint8_t crc_init[3];

	uint8_t phy:3;
	uint8_t is_rx_enabled:1;
	/* Bitmask providing not allowed types of CTE. */
	uint8_t cte_type:5;
	/* The member is required for filtering by CTE type. If filtering policy is disabled then
	 * synchronization is terminated for periodic advertisements with wrong CTE type.
	 */
	uint8_t filter_policy:1;
	uint8_t is_aux_sched:1;

#if defined(CONFIG_BT_CTLR_SYNC_ISO)
	uint8_t sca:3;
#endif /* CONFIG_BT_CTLR_SYNC_ISO */

	uint16_t skip_prepare;
	uint16_t skip_event;
	uint16_t event_counter;

	uint16_t data_chan_id;
	struct {
		uint8_t data_chan_map[PDU_CHANNEL_MAP_SIZE];
		uint8_t data_chan_count:6;
	} chm[DOUBLE_BUFFER_SIZE];
	uint8_t  chm_first;
	uint8_t  chm_last;
	uint16_t chm_instant;

	uint32_t window_widening_periodic_us;
	uint32_t window_widening_max_us;
	uint32_t window_widening_prepare_us;
	uint32_t window_widening_event_us;
	uint32_t window_size_event_us;

	/* used to store lll_aux when chain is being scanned */
	struct lll_scan_aux *lll_aux;

#if defined(CONFIG_BT_CTLR_DF_SCAN_CTE_RX)
	struct lll_df_sync df_cfg;
#endif /* CONFIG_BT_CTLR_DF_SCAN_CTE_RX */
};

int lll_sync_init(void);
int lll_sync_reset(void);
void lll_sync_create_prepare(void *param);
void lll_sync_prepare(void *param);
enum sync_status lll_sync_cte_is_allowed(uint8_t cte_type_mask, uint8_t filter_policy,
					 uint8_t rx_cte_time, uint8_t rx_cte_type);
extern uint16_t ull_sync_lll_handle_get(struct lll_sync *lll);
