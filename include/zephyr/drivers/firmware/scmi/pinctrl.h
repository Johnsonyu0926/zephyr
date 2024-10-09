/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SCMI pinctrl protocol helpers
 */

#ifndef _INCLUDE_ZEPHYR_DRIVERS_FIRMWARE_SCMI_PINCTRL_H_
#define _INCLUDE_ZEPHYR_DRIVERS_FIRMWARE_SCMI_PINCTRL_H_

#include <zephyr/drivers/firmware/scmi/protocol.h>

#define ARM_SCMI_PINCTRL_MAX_CONFIG_SIZE (10 * 2)

#define SCMI_PINCTRL_NO_FUNCTION 0xFFFFFFFF

#define SCMI_PINCTRL_CONFIG_ATTRIBUTES(fid_valid, cfg_num, selector)                               \
	(FIELD_PREP(BIT(1), fid_valid) | FIELD_PREP(GENMASK(7, 0), cfg_num) |                      \
	 FIELD_PREP(GENMASK(1, 0), selector))

#define SCMI_PINCTRL_SELECTOR_PIN 0x0
#define SCMI_PINCTRL_SELECTOR_GROUP 0x1

#define SCMI_PINCTRL_ATTRIBUTES_CONFIG_NUM(attributes)\
	(((attributes) & GENMASK(9, 2)) >> 2)

/**
 * @brief Pinctrl protocol command message IDs
 */
enum scmi_pinctrl_message {
	SCMI_PINCTRL_MSG_PROTOCOL_VERSION = 0x0,
	SCMI_PINCTRL_MSG_PROTOCOL_ATTRIBUTES = 0x1,
	SCMI_PINCTRL_MSG_PROTOCOL_MESSAGE_ATTRIBUTES = 0x2,
	SCMI_PINCTRL_MSG_PINCTRL_ATTRIBUTES = 0x3,
	SCMI_PINCTRL_MSG_PINCTRL_LIST_ASSOCIATIONS = 0x4,
	SCMI_PINCTRL_MSG_PINCTRL_SETTINGS_GET = 0x5,
	SCMI_PINCTRL_MSG_PINCTRL_SETTINGS_CONFIGURE = 0x6,
	SCMI_PINCTRL_MSG_PINCTRL_REQUEST = 0x7,
	SCMI_PINCTRL_MSG_PINCTRL_RELEASE = 0x8,
	SCMI_PINCTRL_MSG_PINCTRL_NAME_GET = 0x9,
	SCMI_PINCTRL_MSG_PINCTRL_SET_PERMISSIONS = 0xa,
	SCMI_PINCTRL_MSG_NEGOTIATE_PROTOCOL_VERSION = 0x10,
};

/**
 * @brief Pinctrl configurations
 */
enum scmi_pinctrl_config {
	SCMI_PINCTRL_DEFAULT = 0,
	SCMI_PINCTRL_BIAS_BUS_HOLD = 1,
	SCMI_PINCTRL_BIAS_DISABLE = 2,
	SCMI_PINCTRL_BIAS_HIGH_Z = 3,
	SCMI_PINCTRL_BIAS_PULL_UP = 4,
	SCMI_PINCTRL_BIAS_PULL_DEFAULT = 5,
	SCMI_PINCTRL_BIAS_PULL_DOWN = 6,
	SCMI_PINCTRL_DRIVE_OPEN_DRAIN = 7,
	SCMI_PINCTRL_DRIVE_OPEN_SOURCE = 8,
	SCMI_PCINTRL_DRIVE_PUSH_PULL = 9,
	SCMI_PCINTRL_DRIVE_STRENGTH = 10,
	SCMI_PINCTRL_INPUT_DEBOUNCE = 11,
	SCMI_PINCTRL_INPUT_MODE = 12,
	SCMI_PINCTRL_PULL_MODE = 13,
	SCMI_PINCTRL_INPUT_VALUE = 14,
	SCMI_PINCTRL_INPUT_SCHMITT = 15,
	SCMI_PINCTRL_LP_MODE = 16,
	SCMI_PINCTRL_OUTPUT_MODE = 17,
	SCMI_PINCTRL_OUTPUT_VALUE = 18,
	SCMI_PINCTRL_POWER_SOURCE = 19,
	SCMI_PINCTRL_SLEW_RATE = 20,
	SCMI_PINCTRL_RESERVED_START = 21,
	SCMI_PINCTRL_RESERVED_END = 191,
	SCMI_PINCTRL_VENDOR_START = 192,
};

/**
 * @struct scmi_pinctrl_settings
 *
 * @brief Describes the parameters for the PINCTRL_SETTINGS_CONFIGURE
 * command
 */
struct scmi_pinctrl_settings {
	uint32_t id;
	uint32_t function;
	uint32_t attributes;
	uint32_t config[ARM_SCMI_PINCTRL_MAX_CONFIG_SIZE];
};

/**
 * @brief Send the PINCTRL_SETTINGS_CONFIGURE command and get its reply
 *
 * @param settings pointer to settings to be applied
 *
 * @retval 0 if successful
 * @retval negative errno if failure
 */
int scmi_pinctrl_settings_configure(struct scmi_pinctrl_settings *settings);

#endif /* _INCLUDE_ZEPHYR_DRIVERS_FIRMWARE_SCMI_PINCTRL_H_ */
