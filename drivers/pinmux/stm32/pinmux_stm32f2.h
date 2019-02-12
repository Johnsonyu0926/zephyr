/*
 * Copyright (c) 2018 qianfan Zhao
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PINMUX_STM32_PINMUX_STM32F2_H_
#define ZEPHYR_DRIVERS_PINMUX_STM32_PINMUX_STM32F2_H_

/**
 * @file Header for STM32F2 pin multiplexing helper
 */

/* Port A */
#define STM32F2_PINMUX_FUNC_PA0_UART4_TX    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)
#define STM32F2_PINMUX_FUNC_PA1_ETH         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F2_PINMUX_FUNC_PA0_ADC123_IN0	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PA1_UART4_RX    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)
#define STM32F2_PINMUX_FUNC_PA1_ADC123_IN1	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PA2_USART2_TX   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)
#define STM32F2_PINMUX_FUNC_PA2_ETH         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F2_PINMUX_FUNC_PA2_ADC123_IN2	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PA3_USART2_RX   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)
#define STM32F2_PINMUX_FUNC_PA3_ETH         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F2_PINMUX_FUNC_PA3_ADC123_IN3	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PA4_ADC12_IN4	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PA5_ADC12_IN5	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PA6_ADC12_IN6	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PA7_ETH         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F2_PINMUX_FUNC_PA7_ADC12_IN7	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PA9_USART1_TX   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)

#define STM32F2_PINMUX_FUNC_PA10_USART1_RX  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)

#define STM32F2_PINMUX_FUNC_PA11_OTG_FS_DM  \
	(STM32_PINMUX_ALT_FUNC_10 | STM32_PUSHPULL_NOPULL)

#define STM32F2_PINMUX_FUNC_PA12_OTG_FS_DP  \
	(STM32_PINMUX_ALT_FUNC_10 | STM32_PUSHPULL_NOPULL)

/* Port B */
#define STM32F2_PINMUX_FUNC_PB0_ADC12_IN8	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PB1_ADC12_IN9	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PB6_USART1_TX   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)

#define STM32F2_PINMUX_FUNC_PB7_USART1_RX   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)

#define STM32F2_PINMUX_FUNC_PB10_USART3_TX  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)

#define STM32F2_PINMUX_FUNC_PB11_USART3_RX  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)
#define STM32F2_PINMUX_FUNC_PB11_ETH        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F2_PINMUX_FUNC_PB12_ETH        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F2_PINMUX_FUNC_PB13_ETH        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

/* Port C */
#define STM32F2_PINMUX_FUNC_PC0_ADC123_IN10	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PC1_ETH         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F2_PINMUX_FUNC_PC1_ADC123_IN11	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PC2_ADC123_IN12	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PC3_ADC123_IN13	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PC4_ETH         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F2_PINMUX_FUNC_PC4_ADC12_IN14	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PC5_ETH         \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F2_PINMUX_FUNC_PC5_ADC12_IN15	\
	STM32_MODER_ANALOG_MODE

#define STM32F2_PINMUX_FUNC_PC6_USART6_TX   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)

#define STM32F2_PINMUX_FUNC_PC7_USART6_RX   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)

#define STM32F2_PINMUX_FUNC_PC10_USART3_TX  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)
#define STM32F2_PINMUX_FUNC_PC10_UART4_TX   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)

#define STM32F2_PINMUX_FUNC_PC11_USART3_RX  \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)
#define STM32F2_PINMUX_FUNC_PC11_UART4_RX   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)

#define STM32F2_PINMUX_FUNC_PC12_UART5_TX   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)

/* Port D */
#define STM32F2_PINMUX_FUNC_PD2_UART5_RX    \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)

#define STM32F2_PINMUX_FUNC_PD5_USART2_TX   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)

#define STM32F2_PINMUX_FUNC_PD6_USART2_RX   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)

#define STM32F2_PINMUX_FUNC_PD8_USART3_TX   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP)

#define STM32F2_PINMUX_FUNC_PD9_USART3_RX   \
	(STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_NOPULL)

/* Port E */

/* Port F */
#define STM32F2_PINMUX_FUNC_PF3_ADC3_IN9	\
	STM32_MODER_ANALOG_MODE
#define STM32F2_PINMUX_FUNC_PF4_ADC3_IN14	\
	STM32_MODER_ANALOG_MODE
#define STM32F2_PINMUX_FUNC_PF5_ADC3_IN15	\
	STM32_MODER_ANALOG_MODE
#define STM32F2_PINMUX_FUNC_PF6_ADC3_IN4	\
	STM32_MODER_ANALOG_MODE
#define STM32F2_PINMUX_FUNC_PF7_ADC3_IN5	\
	STM32_MODER_ANALOG_MODE
#define STM32F2_PINMUX_FUNC_PF8_ADC3_IN6	\
	STM32_MODER_ANALOG_MODE
#define STM32F2_PINMUX_FUNC_PF9_ADC3_IN7	\
	STM32_MODER_ANALOG_MODE
#define STM32F2_PINMUX_FUNC_PF10_ADC3_IN8	\
	STM32_MODER_ANALOG_MODE

/* Port G */
#define STM32F2_PINMUX_FUNC_PG9_USART6_RX   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_NOPULL)

#define STM32F2_PINMUX_FUNC_PG11_ETH        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F2_PINMUX_FUNC_PG13_ETH        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)

#define STM32F2_PINMUX_FUNC_PG14_ETH        \
	(STM32_PINMUX_ALT_FUNC_11 | STM32_PUSHPULL_NOPULL | \
	 STM32_OSPEEDR_VERY_HIGH_SPEED)
#define STM32F2_PINMUX_FUNC_PG14_USART6_TX   \
	(STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP)

/* Port H */

#endif /* ZEPHYR_DRIVERS_PINMUX_STM32_PINMUX_STM32F2_H_ */
