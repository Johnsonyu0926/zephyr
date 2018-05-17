/*
 * STMicroelectronics LIS2DW12 driver
 *
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <misc/__assert.h>
#include <misc/byteorder.h>
#include <sensor.h>

#ifdef CONFIG_LIS2DW12_SPI
#include <spi.h>
#else /* CONFIG_LIS2DW12_SPI */
#include <i2c.h>
#endif /* CONFIG_LIS2DW12_SPI */

#include "lis2dw12.h"

struct lis2dw12_data lis2dw12_device_data;

/**
 * lis2dw12_set_range - set full scale range for acc
 * @dev: Pointer to instance of struct device (I2C or SPI)
 * @range: Full scale range (2, 4, 8 and 16 G)
 */
static int lis2dw12_set_range(struct device *dev, u16_t range)
{
	int err;
	struct lis2dw12_data *lis2dw12 = dev->driver_data;
	const struct lis2dw12_device_config *cfg = dev->config->config_info;
	u8_t shift_gain = 0;

	err = lis2dw12->hw_tf->update_reg(lis2dw12, LIS2DW12_CTRL1_ADDR,
					  LIS2DW12_FS_MASK,
					  LIS2DW12_FS_TO_REG(range));

	if (cfg->pm == LIS2DW12_LOW_POWER_M1) {
		shift_gain = LIS2DW12_SHFT_GAIN_NOLP1;
	}

	if (!err) {
		/* save internally gain for optimization */
		lis2dw12->acc_fs_sensitivity =
			LIS2DW12_FS_TO_GAIN(LIS2DW12_FS_TO_REG(range),
					    shift_gain);
	}

	return err;
}

/**
 * lis2dw12_set_odr - set new sampling frequency
 * @dev: Pointer to instance of struct device (I2C or SPI)
 * @odr: Output data rate
 */
static int lis2dw12_set_odr(struct device *dev, u16_t odr)
{
	struct lis2dw12_data *lis2dw12 = dev->driver_data;

	/* check if power off */
	if (odr == 0) {
		return lis2dw12->hw_tf->update_reg(lis2dw12,
						   LIS2DW12_CTRL1_ADDR,
						   LIS2DW12_ODR_MASK,
						   LIS2DW12_ODR_POWER_OFF_VAL);
	}

	if (odr > LIS2DW12_MAX_ODR) {
		SYS_LOG_ERR("ODR too high");
		return -ENOTSUP;
	}

	return lis2dw12->hw_tf->update_reg(lis2dw12, LIS2DW12_CTRL1_ADDR,
					   LIS2DW12_ODR_MASK,
					   LIS2DW12_ODR_TO_REG(odr));
}

static inline void lis2dw12_channel_get_acc(struct device *dev,
					     enum sensor_channel chan,
					     struct sensor_value *val)
{
	s32_t cval;
	int i;
	u8_t ofs_start, ofs_stop;
	struct lis2dw12_data *lis2dw12 = dev->driver_data;
	struct sensor_value *pval = val;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		ofs_start = ofs_stop = 0;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		ofs_start = ofs_stop = 1;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		ofs_start = ofs_stop = 2;
		break;
	default:
		ofs_start = 0; ofs_stop = 2;
		break;
	}

	for (i = ofs_start; i <= ofs_stop ; i++) {
		cval = lis2dw12->acc[i] * lis2dw12->acc_fs_sensitivity;
		pval->val1 = cval / 1000000;
		pval->val2 = cval % 1000000;
		pval++;
	}
}

static int lis2dw12_channel_get(struct device *dev,
				 enum sensor_channel chan,
				 struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		lis2dw12_channel_get_acc(dev, chan, val);
		return 0;
	default:
		SYS_LOG_DBG("Channel not supported");
		break;
	}

	return -ENOTSUP;
}

static int lis2dw12_config(struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return lis2dw12_set_range(dev, val->val1);
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lis2dw12_set_odr(dev, val->val1);
	default:
		SYS_LOG_DBG("Acc attribute not supported");
		break;
	}

	return -ENOTSUP;
}

static int lis2dw12_attr_set(struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr,
			      const struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		return lis2dw12_config(dev, chan, attr, val);
	default:
		SYS_LOG_DBG("Attr not supported on %d channel", chan);
		break;
	}

	return -ENOTSUP;
}

static int lis2dw12_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct lis2dw12_data *lis2dw12 = dev->driver_data;
	const struct lis2dw12_device_config *cfg = dev->config->config_info;
	u8_t shift;
	union {
		u8_t raw[6];
		struct {
			s16_t a_axis[3];
		};
	} buf __aligned(2);
	u8_t tmp;

	if (lis2dw12->hw_tf->read_reg(lis2dw12, LIS2DW12_STATUS_REG, &tmp)) {
		return -EIO;
	}

	if (!(tmp & LIS2DW12_STS_XLDA_UP)) {
		return -EAGAIN;
	}

	/* fetch raw data sample */
	if (lis2dw12->hw_tf->read_data(lis2dw12, LIS2DW12_OUT_X_L_ADDR,
				buf.raw, sizeof(buf)) < 0) {
		SYS_LOG_DBG("Failed to fetch raw data sample");
		return -EIO;
	}

	/* adjust to resolution */
	if (cfg->pm == LIS2DW12_LOW_POWER_M1) {
		shift = LIS2DW12_SHIFT_PM1;
	} else {
		shift = LIS2DW12_SHIFT_PMOTHER;
	}

	lis2dw12->acc[0] = sys_le16_to_cpu(buf.a_axis[0] >> shift);
	lis2dw12->acc[1] = sys_le16_to_cpu(buf.a_axis[1] >> shift);
	lis2dw12->acc[2] = sys_le16_to_cpu(buf.a_axis[2] >> shift);

	return 0;
}

static const struct sensor_driver_api lis2dw12_driver_api = {
	.attr_set = lis2dw12_attr_set,
#if CONFIG_LIS2DW12_TRIGGER
	.trigger_set = lis2dw12_trigger_set,
#endif /* CONFIG_LIS2DW12_TRIGGER */
	.sample_fetch = lis2dw12_sample_fetch,
	.channel_get = lis2dw12_channel_get,
};

static int lis2dw12_init_interface(struct device *dev)
{
	struct lis2dw12_data *lis2dw12 = dev->driver_data;
	const struct lis2dw12_device_config *cfg = dev->config->config_info;

	lis2dw12->bus = device_get_binding(cfg->bus_name);
	if (!lis2dw12->bus) {
		SYS_LOG_DBG("master bus not found: %s", cfg->bus_name);
		return -EINVAL;
	}

#ifdef CONFIG_LIS2DW12_SPI
	lis2dw12_spi_init(dev);
#else /* CONFIG_LIS2DW12_SPI */
	lis2dw12_i2c_init(dev);
#endif /* CONFIG_LIS2DW12_SPI */

	return 0;
}

static int lis2dw12_set_power_mode(struct lis2dw12_data *lis2dw12,
				    enum lis2dh_powermode pm)
{
	u8_t regval = LIS2DW12_LOW_POWER_M1 | LIS2DW12_LOW_POWER_MODE;

	switch (pm) {
	case LIS2DW12_LOW_POWER_M2:
		regval = LIS2DW12_LOW_POWER_M2 | LIS2DW12_LOW_POWER_MODE;
		break;
	case LIS2DW12_LOW_POWER_M3:
		regval = LIS2DW12_LOW_POWER_M3 | LIS2DW12_LOW_POWER_MODE;
		break;
	case LIS2DW12_LOW_POWER_M4:
		regval = LIS2DW12_LOW_POWER_M4 | LIS2DW12_LOW_POWER_MODE;
		break;
	case LIS2DW12_HIGH_PERF:
		regval = LIS2DW12_HP_MODE;
		break;
	default:
		SYS_LOG_DBG("Apply default Power Mode");
		break;
	}

	return lis2dw12->hw_tf->write_reg(lis2dw12, LIS2DW12_CTRL1_ADDR,
					  regval);
}

static int lis2dw12_init(struct device *dev)
{
	struct lis2dw12_data *lis2dw12 = dev->driver_data;
	const struct lis2dw12_device_config *cfg = dev->config->config_info;
	u8_t wai;

	if (lis2dw12_init_interface(dev)) {
		return -EINVAL;
	}

	/* check chip ID */
	if (lis2dw12->hw_tf->read_reg(lis2dw12, LIS2DW12_WHO_AM_I_REG,
				      &wai) < 0) {
		SYS_LOG_ERR("Failed to read chip ID");
		return -EIO;
	}

	if (wai != LIS2DW12_WHO_AM_I) {
		SYS_LOG_ERR("Invalid chip ID");
		return -EINVAL;
	}

	/* reset device */
	if (lis2dw12->hw_tf->write_reg(lis2dw12, LIS2DW12_CTRL2_ADDR,
				       LIS2DW12_RESET_MASK)) {
		return -EIO;
	}

	k_busy_wait(100);

	if (lis2dw12->hw_tf->update_reg(lis2dw12, LIS2DW12_CTRL2_ADDR,
					LIS2DW12_BDU_MASK, LIS2DW12_EN_BIT)) {
		return -EIO;
	}

	/* set power mode */
	if (lis2dw12_set_power_mode(lis2dw12, CONFIG_LIS2DW12_POWER_MODE)) {
		return -EIO;
	}

	/* set default odr and full scale for acc */
	if (lis2dw12->hw_tf->update_reg(lis2dw12, LIS2DW12_CTRL1_ADDR,
					LIS2DW12_ODR_MASK,
					LIS2DW12_DEFAULT_ODR)) {
		return -EIO;
	}

	if (lis2dw12->hw_tf->write_reg(lis2dw12, LIS2DW12_CTRL6_ADDR,
				       LIS2DW12_ACC_FS)) {
		return -EIO;
	}

	lis2dw12->acc_fs_sensitivity =
		LIS2DW12_FS_TO_GAIN(LIS2DW12_ACC_FS,
				    cfg->pm == LIS2DW12_LOW_POWER_M1 ?
				    LIS2DW12_SHFT_GAIN_NOLP1 : 0);

#ifdef CONFIG_LIS2DW12_TRIGGER
	if (lis2dw12_init_interrupt(dev) < 0) {
		SYS_LOG_ERR("Failed to initialize interrupts");
		return -EIO;
	}
#endif /* CONFIG_LIS2DW12_TRIGGER */

	dev->driver_api = &lis2dw12_driver_api;

	return 0;
}

const struct lis2dw12_device_config lis2dw12_cfg = {
#ifdef CONFIG_LIS2DW12_SPI
	.bus_name = CONFIG_LIS2DW12_SPI_MASTER_DEV_NAME,
#else /* CONFIG_LIS2DW12_SPI */
	.bus_name = CONFIG_LIS2DW12_I2C_MASTER_DEV_NAME,
#endif /* CONFIG_LIS2DW12_SPI */
	.pm = CONFIG_LIS2DW12_POWER_MODE,
#ifdef CONFIG_LIS2DW12_TRIGGER
	.int_gpio_port = CONFIG_LIS2DW12_GPIO_DEV_NAME,
	.int_gpio_pin = CONFIG_LIS2DW12_GPIO_PIN_NUM,
/* Select sensor interrupt */
#ifdef CONFIG_LIS2DW12_INT_PIN_1
	.int_pin = 1,
#else /* CONFIG_LIS2DW12_INT_PIN_1 */
	.int_pin = 2,
#endif /* CONFIG_LIS2DW12_INT_PIN_1 */
#endif /* CONFIG_LIS2DW12_TRIGGER */
};

DEVICE_INIT(lis2dw12, CONFIG_LIS2DW12_NAME, lis2dw12_init,
	     &lis2dw12_device_data, &lis2dw12_cfg, POST_KERNEL,
	     CONFIG_SENSOR_INIT_PRIORITY);

