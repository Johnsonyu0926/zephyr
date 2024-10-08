/*
 * Copyright (c) currentyear authorname
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT vendorname_sensorname

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(video_sensorname, CONFIG_VIDEO_LOG_LEVEL);

/* Best to use the exact same name as in the datasheets */
#define SENSORNAME_REG0 0x0000
#define SENSORNAME_REG1 0x0001
#define SENSORNAME_REG2 0x0002
#define SENSORNAME_REG3 0x0003
#define SENSORNAME_REG4 0x0004
#define SENSORNAME_REG5 0x0005
#define SENSORNAME_REG6 0x0006

#define SENSORNAME_SENSOR_ID 0x99

struct sensorname_config {
	struct i2c_dt_spec i2c;
};

struct sensorname_data {
	struct video_format fmt;
};

struct sensorname_reg {
	uint16_t addr;
	uint8_t value;
};

static struct sensorname_reg sensorname_init_regs[] = {
	{SENSORNAME_REG0, 0x00},
	{SENSORNAME_REG1, 0x01},
	/* Example comment about REG2 */
	{SENSORNAME_REG2, 0x10},
	{SENSORNAME_REG3, 0x00},
	{0},
};

/* Replace BGGR8 and RGB565 by a VIDEO_PIX_FMT name of your choice */

enum sensorname_format_id {
	FMT_BGGR8_640x480,
	FMT_RGB565_640x480,
	FMT_RGB565_1280x720,
};

static struct sensorname_reg sensorname_bggr8_640x480_regs[] = {
	{SENSORNAME_REG4, 0x64},
	{SENSORNAME_REG5, 0x48},
	{0},
};

static struct sensorname_reg sensorname_rgb565_640x480_regs[] = {
	{SENSORNAME_REG4, 0x64},
	{SENSORNAME_REG5, 0x48},
	{0},
};

static struct sensorname_reg sensorname_rgb565_1280x720_regs[] = {
	{SENSORNAME_REG4, 0x12},
	{SENSORNAME_REG5, 0x72},
	{0},
};

static struct sensorname_reg sensorname_fmt_regs {
	[FMT_RGB565_640x480] = sensorname_rgb565_640x480_regs,
	[FMT_BGGR8_640x480] = sensorname_bggr8_640x480_regs,
	[FMT_BGGR8_1280x720] = sensorname_bggr8_1280x720_regs,
};

#define SENSORNAME_FORMAT_CAP(width, height, format)                                               \
	{                                                                                          \
		.pixelformat = (format), .width_min = (width), .width_max = (width),               \
		.height_min = (height), .height_max = (height), .width_step = 0, .height_step = 0  \
	}

static const struct video_format_cap fmts_regs[] = {
	[FMT_BGGR8_640x480] = SENSORNAME_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_BGGR8),
	[FMT_RGB565_640x480] = SENSORNAME_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_RGB565),
	[FMT_RGB565_1280x720] = SENSORNAME_FORMAT_CAP(1280, 720, VIDEO_PIX_FMT_RGB565),
	{0},
};

/* Some sensors will need reg8 or reg16 variants. */
static int sensorname_read_reg(const struct device *const dev, uint8_t reg_addr, uint8_t *value)
{
	const struct sensorname_config *conf = dev->config;
	int ret;

	ret = i2c_reg_read_byte_dt(conf->i2c, reg_addr, value);
	if (ret < 0) {
		LOG_ERR("Failed to read from %s register 0x%x", dev->name, reg_addr);
		return ret;
	}
	return 0;
}

/* Some sensors will need reg8 or reg16 variants. */
static int sensorname_write_reg(const struct device *const dev, uint8_t reg_addr, uint8_t value)
{
	const struct sensorname_config *conf = dev->config;
	int ret;

	ret = i2c_reg_write_byte_dt(conf->i2c, reg_addr, value);
	if (ret < 0) {
		LOG_ERR("Failed to write 0x%x to %s register 0x%x", value, dev->name, reg_addr);
		return ret;
	}
	return 0;
}

static int sensorname_write_multi(const struct device *const dev, struct sensorname_reg *regs)
{
	int ret;

	for (size_t i = 0; regs[i].addr != 0; i++) {
		ret = sensorname_write_reg(dev, regs[i].addr, regs[i].value);
		if (ret < 0) {
			return ret;
		}
	}
	return 0;
}

static int sensorname_ctrl_get_gain(const struct device *const dev, uint32_t *value)
{
	return sensorname_read_reg(spec, SENSORNAME_REG2, *value);
}

static int sensorname_ctrl_set_gain(const struct device *const dev, uint32_t value)
{
	return sensorname_write_reg(spec, SENSORNAME_REG2, value);
}

static int sensorname_ctrl_get_exposure(const struct device *const dev, uint32_t *value)
{
	return sensorname_read_reg(spec, SENSORNAME_REG3, *value);
}

static int sensorname_ctrl_set_exposure(const struct device *const dev, uint32_t value)
{
	return sensorname_write_reg(spec, SENSORNAME_REG3, value);
}

static int sensorname_ctrl_set(const struct device *dev, unsigned int cid, void *value)
{
	switch (cid) {
	case VIDEO_CID_EXPOSURE:
		return sensorname_ctrl_set_exposure(dev, (uint32_t)value);
	case VIDEO_CID_GAIN:
		return sensorname_ctrl_set_gain(dev, (uint32_t)value);
	default:
		return -ENOTSUP;
	}
}

static int sensorname_ctrl_get(const struct device *dev, unsigned int cid, void *value)
{
	switch (cid) {
	case VIDEO_CID_EXPOSURE:
		return sensorname_ctrl_set_exposure(dev, (uint32_t)value);
	case VIDEO_CID_GAIN:
		return sensorname_ctrl_set_gain(dev, (uint32_t)value);
	default:
		return -ENOTSUP;
	}
}

static int sensorname_set_fmt(const struct device *const dev, enum video_endpoint_id ep,
			      struct video_format *fmt)
{
	struct sensorname_data *data = dev->data;
	enum sensorname_format_id id;
	int ret;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	if (memcmp(&data->fmt, fmt, sizeof(data->fmt)) == 0) {
		return 0;
	}

	ret = video_get_format_index(fmts, fmt, &id);
	if (ret < 0) {
		LOG_ERR("Format " PRIvfmt " not found for %s", PRIvfmt_arg(fmt), dev->name);
		return ret;
	}

	ret = sensorname_write_multi(dev, &sensorname_fmts_regs[id]);
	if (ret < 0) {
		LOG_ERR("Could not apply %s format " PRIvfmt, dev->name, PRIvfmt_arg(fmt));
		return ret;
	}

	data->fmt = *fmt;
	return 0;
}

static int sensorname_get_fmt(const struct device *dev, enum video_endpoint_id ep,
                              struct video_format *fmt)
{
	struct sensorname_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

        *fmt = data->fmt;
        return 0;
}

static int sensorname_get_caps(const struct device *dev, enum video_endpoint_id ep,
			       struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static int sensorname_stream_start(const struct device *dev)
{
	return sensorname_write_reg(dev, SENSORNAME_REG0, 1);
}

static int sensorname_stream_stop(const struct device *dev)
{
	return sensorname_write_reg(dev, SENSORNAME_REG0, 0);
}

static const struct video_driver_api sensorname_driver_api = {
	.set_format = sensorname_set_fmt,
	.get_format = sensorname_get_fmt,
	.get_caps = sensorname_get_caps,
	.stream_start = sensorname_stream_start,
	.stream_stop = sensorname_stream_stop,
	.set_ctrl = sensorname_set_ctrl,
};

int sensorname_init(const struct device *dev)
{
	struct sensorname_data *data = dev->data;
	uint8_t sensor_id;
	int ret;

	if (!i2c_is_ready_dt(&conf->i2c)) {
		LOG_ERR("Bus %s is not ready", conf->i2c.bus->name);
		return -ENODEV;
	}

	ret = sensorname_read_reg(dev, SENSORNAME_REG6);
	if (ret < 0 || sensor_id != SENSORNAME_SENSOR_ID) {
		LOG_ERR("Failed to read %s sensor ID", dev->name);
		return ret;
	}

	ret = sensorname_write_multi(dev, &sensorname_init_regs);
	if (ret < 0) {
		LOG_ERR("Could not initialize %s", dev->name);
		return ret;
	}

	/* Default format */
	data->fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
	data->fmt.width = 1280;
	data->fmt.height = 720;
	data->fmt.pitch = data->fmt.width * 2;

	ret = sensorname_set_fmt(dev, VIDEO_EP_OUT, &data->fmt);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

#define SENSORNAME_DEFINE(inst)                                                                    \
	static struct sensorname_data sensorname_data_##inst;                                      \
                                                                                                   \
	static const struct sensorname_config sensorname_conf_##inst = {                           \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &sensorname_init, NULL, &sensorname_data_##inst,               \
			      &sensorname_conf_##inst, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,    \
			      &sensorname_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSORNAME_DEFINE)
