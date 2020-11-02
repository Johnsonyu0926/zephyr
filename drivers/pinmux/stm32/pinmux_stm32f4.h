/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2019 Centaur Analytics, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PINMUX_STM32_PINMUX_STM32F4_H_
#define ZEPHYR_DRIVERS_PINMUX_STM32_PINMUX_STM32F4_H_

/**
 * @file Header for STM32F4 pin multiplexing helper
 */

/*
 * Note:
 * The SPIx_SCK pin speed must be set to VERY_HIGH to avoid last data bit
 * corruption which is a known issue of STM32F4 SPI peripheral (see errata
 * sheets).
 */

/* Port A */
#define STM32F4_PINMUX_FUNC_PA0_PWM2_CH1 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA0_PWM5_CH1 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA0_USART2_CTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PA0_UART4_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA0_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PA0_ADC123_IN0 __DEPRECATED_MACRO		\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PA1_PWM5_CH2 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA1_I2S4_SD    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA1_USART2_RTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PA1_UART4_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PA1_ETH __DEPRECATED_MACRO         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PA1_ADC123_IN1 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PA2_PWM5_CH3 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA2_USART2_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA2_ETH __DEPRECATED_MACRO         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PA2_ADC123_IN2 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PA3_PWM5_CH4 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA3_USART2_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PA3_ETH __DEPRECATED_MACRO         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PA3_ADC123_IN3 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE
#define STM32F4_PINMUX_FUNC_PA3_PWM2_CH4 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PA4_SPI1_NSS __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA4_I2S1_WS    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PA4_I2S3_WS    \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PA4_ADC12_IN4 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PA5_PWM2_CH1 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA5_SPI1_SCK __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PA5_I2S1_CK  \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PA5_ADC12_IN5 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PA6_SPI1_MISO __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)
#define STM32F4_PINMUX_FUNC_PA6_ADC12_IN6 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PA7_SPI1_MOSI __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)
#define STM32F4_PINMUX_FUNC_PA7_I2S1_SD  \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PA7_ETH __DEPRECATED_MACRO         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PA7_ADC12_IN7 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PA8_MCO	    \
	(STM32_PINMUX_ALT_FUNC_0 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PA8_I2C3_SCL __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PA8_UART7_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PA9_USART1_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PA10_USART1_RX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PA11_USART1_CTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PA11_USART6_TX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA11_FDCAN1_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA11_UART4_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PA11_OTG_FS_DM __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_10 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PA12_USART1_RTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PA12_USART6_RX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PA12_FDCAN1_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PA12_UART4_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA12_OTG_FS_DP __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_10 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PA15_SPI1_NSS __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA15_I2S3_WS    \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PA15_USART1_TX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PA15_UART7_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)

/* Port B */
#define STM32F4_PINMUX_FUNC_PB0_I2S5_CK  \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PB0_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PB0_ADC12_IN8 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE
#define STM32F4_PINMUX_FUNC_PB0_SPI1_NSS __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)


#define STM32F4_PINMUX_FUNC_PB1_ADC12_IN9 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PB3_I2S3_CK   \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PB3_USART1_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB3_UART7_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB3_I2C2_SDA __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB1_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PB3_SPI1_SCK __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)


#define STM32F4_PINMUX_FUNC_PB4_PWM3_CH1 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB4_UART7_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PB4_I2C3_SDA __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB4_SPI1_MISO __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)

#define STM32F4_PINMUX_FUNC_PB5_I2S3_SD  \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB5_FDCAN2_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PB5_UART5_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB5_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PB5_SPI1_MOSI __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)

#define STM32F4_PINMUX_FUNC_PB6_PWM4_CH1 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB6_I2C1_SCL __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB6_USART1_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PB6_FDCAN2_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB6_UART5_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PB7_PWM4_CH2 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB7_I2C1_SDA __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB7_USART1_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PB8_PWM4_CH3 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB8_I2C1_SCL __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB8_I2S5_SD  \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB8_FDCAN1_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PB8_UART5_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB8_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PB9_PWM4_CH4 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB9_I2C1_SDA __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB9_I2C2_SDA __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB9_FDCAN1_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB9_UART5_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PB9_SPI2_NSS __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PB10_I2C2_SCL __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB10_USART3_TX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PB10_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PB10_SPI2_SCK __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PB11_I2C2_SDA __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB11_USART3_RX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB11_ETH __DEPRECATED_MACRO        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PB12_SPI2_NSS __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PB12_I2S2_WS   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB12_I2S4_WS   \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB12_FDCAN2_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PB12_UART5_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB12_ETH __DEPRECATED_MACRO        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PB13_SPI2_SCK __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PB13_I2S2_CK   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PB13_I2S4_CK   \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PB13_USART3_CTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB13_FDCAN2_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB13_UART5_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PB13_ETH __DEPRECATED_MACRO        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PB14_SPI2_MISO __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)
#define STM32F4_PINMUX_FUNC_PB14_USART3_RTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PB14_OTG_HS_DM __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_12 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PB15_SPI2_MOSI __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)
#define STM32F4_PINMUX_FUNC_PB15_I2S2_SD  \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PB15_OTG_HS_DP __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_12 | STM32_PUSHPULL_NOPULL)

/* Port C */
#define STM32F4_PINMUX_FUNC_PC0_ADC123_IN10 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PC1_ETH __DEPRECATED_MACRO         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PC1_I2S2_SD    \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PC1_ADC123_IN11 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PC2_ETH __DEPRECATED_MACRO         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PC2_ADC123_IN12 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE
#define STM32F4_PINMUX_FUNC_PC2_SPI2_MISO __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)

#define STM32F4_PINMUX_FUNC_PC3_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PC3_ADC123_IN13 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE
#define STM32F4_PINMUX_FUNC_PC3_SPI2_MOSI __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)

#define STM32F4_PINMUX_FUNC_PC4_ETH __DEPRECATED_MACRO         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PC4_ADC12_IN14 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PC5_USART3_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PC5_ETH __DEPRECATED_MACRO         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PC5_ADC12_IN15 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PC6_PWM3_CH1 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PC6_USART6_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PC7_PWM3_CH2 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PC7_USART6_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PC7_I2S2_CK    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PC8_PWM3_CH3 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PC8_UART5_RTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)

#define STM32F4_PINMUX_FUNC_PC9_MCO2	    \
	(STM32_PINMUX_ALT_FUNC_0 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PC9_PWM3_CH4 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PC9_I2C3_SDA __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PC9_UART5_CTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)

#define STM32F4_PINMUX_FUNC_PC10_USART3_TX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PC10_UART4_TX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PC10_SPI3_SCK __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PC11_USART3_RX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PC11_UART4_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PC11_SPI3_MISO __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PC12_UART5_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PC12_I2C2_SDA __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PC12_SPI3_MOSI __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUPDR_PULL_DOWN)

/* Port D */
#define STM32F4_PINMUX_FUNC_PD0_FDCAN1_RX __DEPRECATED_MACRO     \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PD0_UART4_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PD1_FDCAN1_TX __DEPRECATED_MACRO     \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PD2_UART5_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PD2_SPI3_NSS __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PD3_USART2_CTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PD3_SPI2_SCK __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F4_PINMUX_FUNC_PD3_SPI2_NSS __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PD4_USART2_RTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)

#define STM32F4_PINMUX_FUNC_PD5_USART2_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PD6_USART2_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PD8_USART3_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PD9_USART3_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PD10_UART4_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PD11_USART3_CTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)

#define STM32F4_PINMUX_FUNC_PD12_USART3_RTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PD12_PWM4_CH1 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PD13_PWM4_CH2 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PD14_PWM4_CH3 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PD14_UART9_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PD14_SPI2_NSS __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PD15_PWM4_CH4 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_2 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PD15_UART9_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_PULLUP)

/* Port E */
#define STM32F4_PINMUX_FUNC_PE0_UART8_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PE1_UART8_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PE2_UART10_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PE2_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PE3_UART10_TX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PE4_I2S5_WS   \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PE5_PWM9_CH1 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_3 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PE5_SPI3_NSS __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_6 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PE6_PWM9_CH2 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_3 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PE7_UART7_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PE8_UART7_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PE11_SPI4_NSS __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PE12_SPI4_SCK __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PE13_SPI4_MISO __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PE13_PWM1_CH3 __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PE14_SPI4_MOSI __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL)

/* Port F */
#define STM32F4_PINMUX_FUNC_PF0_I2C2_SDA __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)

#define STM32F4_PINMUX_FUNC_PF1_I2C2_SCL __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)

#define STM32F4_PINMUX_FUNC_PF3_ADC3_IN9 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PF4_ADC3_IN14 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PF5_ADC3_IN15 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

#define STM32F4_PINMUX_FUNC_PF6_UART7_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PF6_ADC3_IN4 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE
#define STM32F4_PINMUX_FUNC_PF6_SPI5_MASTER_NSS __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PF7_UART7_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PF7_ADC3_IN5 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE
#define STM32F4_PINMUX_FUNC_PF7_SPI5_MASTER_SCK __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PF8_UART8_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PF8_ADC3_IN6 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE
#define STM32F4_PINMUX_FUNC_PF8_SPI5_MASTER_MISO __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)

#define STM32F4_PINMUX_FUNC_PF9_UART8_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PF9_ADC3_IN7 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE
#define STM32F4_PINMUX_FUNC_PF9_SPI5_MASTER_MOSI __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_5 | STM32_PUPDR_PULL_DOWN)

#define STM32F4_PINMUX_FUNC_PF10_ADC3_IN8 __DEPRECATED_MACRO	\
	STM32_MODER_ANALOG_MODE

/* Port G */
#define STM32F4_PINMUX_FUNC_PG0_UART9_RX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PG1_UART9_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PG9_USART6_RX __DEPRECATED_MACRO   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)

#define STM32F4_PINMUX_FUNC_PG8_USART6_RTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PG8_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PG11_UART10_RX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL)
#define STM32F4_PINMUX_FUNC_PG11_ETH __DEPRECATED_MACRO        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PG12_USART6_RTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PG12_UART10_TX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PG13_USART6_CTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PG13_ETH __DEPRECATED_MACRO        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PG14_USART6_TX __DEPRECATED_MACRO  \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)
#define STM32F4_PINMUX_FUNC_PG14_ETH __DEPRECATED_MACRO        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PG15_USART6_CTS __DEPRECATED_MACRO \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_OPENDRAIN_PULLUP)
/* Port H */
#define STM32F4_PINMUX_FUNC_PH2_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PH3_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PH4_I2C2_SCL __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)

#define STM32F4_PINMUX_FUNC_PH5_I2C2_SDA __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)

#define STM32F4_PINMUX_FUNC_PH7_I2C3_SCL __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)
#define STM32F4_PINMUX_FUNC_PH7_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F4_PINMUX_FUNC_PH8_I2C3_SDA __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP)

#define STM32F4_PINMUX_FUNC_PH13_FDCAN1_TX __DEPRECATED_MACRO    \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_NOPULL)

/* Port I */
#define STM32F4_PINMUX_FUNC_PI9_FDCAN1_RX __DEPRECATED_MACRO     \
	(STM32_PINMUX_ALT_FUNC_9 | STM32_PUSHPULL_PULLUP)

#define STM32F4_PINMUX_FUNC_PI10_ETH __DEPRECATED_MACRO			    \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#endif /* ZEPHYR_DRIVERS_PINMUX_STM32_PINMUX_STM32F4_H_ */
