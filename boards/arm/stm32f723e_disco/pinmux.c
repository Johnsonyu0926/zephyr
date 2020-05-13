/*
 * Copyright (c) 2018 Aurelien Jarno
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for STM32F723E-DISCO board */
static const struct pin_config pinconf[] = {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart2), okay) && CONFIG_SERIAL
	{STM32_PIN_PA2, STM32F7_PINMUX_FUNC_PA2_USART2_TX},
	{STM32_PIN_PA3, STM32F7_PINMUX_FUNC_PA3_USART2_RX},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart6), okay) && CONFIG_SERIAL
	{STM32_PIN_PC6, STM32F7_PINMUX_FUNC_PC6_USART6_TX},
	{STM32_PIN_PC7, STM32F7_PINMUX_FUNC_PC7_USART6_RX},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
	{STM32_PIN_PB8, STM32F7_PINMUX_FUNC_PB8_I2C1_SCL},
	{STM32_PIN_PB9, STM32F7_PINMUX_FUNC_PB9_I2C1_SDA},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay)
	{STM32_PIN_PH4, STM32F7_PINMUX_FUNC_PH4_I2C2_SCL},
	{STM32_PIN_PH5, STM32F7_PINMUX_FUNC_PH5_I2C2_SDA},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay)
	{STM32_PIN_PA8, STM32F7_PINMUX_FUNC_PA8_I2C3_SCL},
	{STM32_PIN_PH8, STM32F7_PINMUX_FUNC_PH8_I2C3_SDA},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay)
	{STM32_PIN_PA5, STM32F7_PINMUX_FUNC_PA5_SPI1_SCK},
	{STM32_PIN_PB4, STM32F7_PINMUX_FUNC_PB4_SPI1_MISO},
	{STM32_PIN_PB5, STM32F7_PINMUX_FUNC_PB5_SPI1_MOSI},
#endif
#ifdef CONFIG_USB_DC_STM32
	{STM32_PIN_PA11, STM32F7_PINMUX_FUNC_PA11_OTG_FS_DM},
	{STM32_PIN_PA12, STM32F7_PINMUX_FUNC_PA12_OTG_FS_DP},
#endif  /* CONFIG_USB_DC_STM32 */
};

static int pinmux_stm32_init(struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
		CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
