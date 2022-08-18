/** @file
 *  @brief Bluetooth shell functions
 *
 *  This is not to be included by the application.
 */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BT_H
#define __BT_H

#include <zephyr/bluetooth/bluetooth.h>
#include <sys/types.h>

extern const struct shell *ctx_shell;
extern struct bt_conn *default_conn;
extern struct bt_csis *csis;

#if defined(CONFIG_BT_ISO)
extern struct bt_iso_chan iso_chan;
#endif /* CONFIG_BT_ISO */

#if defined(CONFIG_BT_EXT_ADV)
extern uint8_t selected_adv;
extern struct bt_le_ext_adv *adv_sets[CONFIG_BT_EXT_ADV_MAX_ADV_SET];
#if defined(CONFIG_BT_PER_ADV_SYNC)
extern struct bt_le_per_adv_sync *per_adv_syncs[CONFIG_BT_PER_ADV_SYNC_MAX];
#endif /* CONFIG_BT_PER_ADV_SYNC */
#endif /* CONFIG_BT_EXT_ADV */

#if defined(CONFIG_BT_AUDIO)
/* Must guard before including audio.h as audio.h uses Kconfigs guarded by
 * CONFIG_BT_AUDIO
 */
#include <zephyr/bluetooth/audio/audio.h>

struct named_lc3_preset {
	const char *name;
	struct bt_audio_lc3_preset preset;
};

#if defined(CONFIG_BT_AUDIO_UNICAST_CLIENT)

extern struct bt_codec *rcodecs[CONFIG_BT_MAX_CONN][2][CONFIG_BT_AUDIO_UNICAST_CLIENT_PAC_COUNT];
extern struct bt_audio_ep *snks[CONFIG_BT_MAX_CONN][CONFIG_BT_AUDIO_UNICAST_CLIENT_ASE_SNK_COUNT];
extern struct bt_audio_ep *srcs[CONFIG_BT_MAX_CONN][CONFIG_BT_AUDIO_UNICAST_CLIENT_ASE_SRC_COUNT];
extern struct named_lc3_preset *default_preset;
#endif /* CONFIG_BT_AUDIO_UNICAST_CLIENT */
#endif /* CONFIG_BT_AUDIO */

void conn_addr_str(struct bt_conn *conn, char *addr, size_t len);
ssize_t audio_ad_data_add(struct bt_data *data, const size_t data_size, const bool discoverable,
			  const bool connectable);
ssize_t csis_ad_data_add(struct bt_data *data, const size_t data_size, const bool discoverable);

#endif /* __BT_H */
