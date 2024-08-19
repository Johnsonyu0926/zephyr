/** @file
 *  @brief Internal APIs for Bluetooth HID Device handling.
 */

/*
 * Copyright 2024 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/classic/hid_device.h>

#define BT_L2CAP_PSM_HID_CTL 0x0011
#define BT_L2CAP_PSM_HID_INT 0x0013

/* Define the HID transaction types
 */
#define BT_HID_TYPE_HANDSHAKE    0x00
#define BT_HID_TYPE_CONTROL      0x01
#define BT_HID_TYPE_GET_REPORT   0x04
#define BT_HID_TYPE_SET_REPORT   0x05
#define BT_HID_TYPE_GET_PROTOCOL 0x06
#define BT_HID_TYPE_SET_PROTOCOL 0x07
#define BT_HID_TYPE_DATA         0x0a

/* Parameters for Control
 */
#define BT_HID_CONTROL_SUSPEND              0x03
#define BT_HID_CONTROL_EXIT_SUSPEND         0x04
#define BT_HID_CONTROL_VIRTUAL_CABLE_UNPLUG 0x05

/* Parameters for Protocol Type
 */
#define BT_HID_PROTOCOL_MASK      0x01
#define BT_HID_PROTOCOL_BOOT_MODE 0x00
#define BT_HID_PROTOCOL_REPORT    0x01

/* @brief HID DEV STATE */
#define BT_HID_STATE_IDLE            0x00
#define BT_HID_STATE_CTRL_CONNECTING 0x01
#define BT_HID_STATE_CTRL_CONNECTED  0x02
#define BT_HID_STATE_INTR_CONNECTING 0x03
#define BT_HID_STATE_CONNECTED       0x04
#define BT_HID_STATE_DISCONNECT      0x05

struct bt_hid_hdr {
	uint8_t param: 4;
	uint8_t type: 4;
};

#define HID_SESSION_BY_CHAN(_ch) CONTAINER_OF(_ch, struct bt_hid_session, br_chan.chan)

/* Initialize HID layer
 */
int bt_hid_dev_init(void);
