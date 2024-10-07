/*
 * Copyright (c) 2019 Linaro Limited.
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_VIDEO_CONTROLS_H_
#define ZEPHYR_INCLUDE_VIDEO_CONTROLS_H_

/**
 * @file
 *
 * @brief Public APIs for Video.
 */

/**
 * @brief Video controls
 * @defgroup video_controls Video Controls
 * @ingroup io_interfaces
 *
 * The Video control IDs (CIDs) are introduced with the same name as
 * Linux V4L2 subsystem and under the same class. This facilitates
 * inter-operability and debugging devices end-to-end across Linux and
 * Zephyr.
 *
 * @{
 */

#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Control classes
 *
 * List of control classes, to group related controls together in blocks.
 *
 * This list is kept identical to the Linux kernel definitions.
 *
 * @{
 */
#define VIDEO_CTRL_CLASS_BASE			0x00980900 /**< Base class controls */
#define VIDEO_CTRL_CLASS_CODEC_BASE		0x00990900 /**< Stateful codec controls */
#define VIDEO_CTRL_CLASS_CAMERA_BASE		0x009a0900 /**< Camera class controls */
#define VIDEO_CTRL_CLASS_FLASH_BASE		0x009c0900 /**< Camera flash controls */
#define VIDEO_CTRL_CLASS_JPEG_BASE		0x009d0900 /**< JPEG-compression controls */
#define VIDEO_CTRL_CLASS_IMAGE_SOURCE_BASE	0x009e0900 /**< Image source controls */
#define VIDEO_CTRL_CLASS_IMAGE_PROC_BASE	0x009f0900 /**< Image processing controls */
#define VIDEO_CTRL_CLASS_VENDOR_BASE		0x00ff0900 /**< Vendor-specific class controls */
/**
 * @}
 */

/**
 * @name Base class control IDs
 * @{
 */
#define VIDEO_CID_BRIGHTNESS			(VIDEO_CTRL_CLASS_BASE + 0)
#define VIDEO_CID_CONTRAST			(VIDEO_CTRL_CLASS_BASE + 1)
#define VIDEO_CID_SATURATION			(VIDEO_CTRL_CLASS_BASE + 2)
#define VIDEO_CID_EXPOSURE			(VIDEO_CTRL_CLASS_BASE + 17)
#define VIDEO_CID_GAIN				(VIDEO_CTRL_CLASS_BASE + 19)
#define VIDEO_CID_HFLIP				(VIDEO_CTRL_CLASS_BASE + 20)
#define VIDEO_CID_VFLIP				(VIDEO_CTRL_CLASS_BASE + 21)
#define VIDEO_CID_WHITE_BALANCE_TEMPERATURE	(VIDEO_CTRL_CLASS_BASE + 26)
/**
 * @}
 */

/**
 * @name Camera class controls IDs
 * @{
 */
#define VIDEO_CID_ZOOM_ABSOLUTE			(VIDEO_CTRL_CLASS_CAMERA_BASE + 13)
/**
 * @}
 */

/**
 * @name JPEG class control IDs
 * @{
 */
#define VIDEO_CID_JPEG_COMPRESSION_QUALITY	(VIDEO_CTRL_CLASS_JPEG_BASE + 3)
/**
 * @}
 */

/**
 * @name Image Processing class control IDs
 * @{
 */
#define VIDEO_CID_TEST_PATTERN			(VIDEO_CTRL_CLASS_IMAGE_PROC_BASE + 3)
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_VIDEO_H_ */
