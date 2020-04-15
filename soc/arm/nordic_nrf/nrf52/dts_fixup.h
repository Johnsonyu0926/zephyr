/* SPDX-License-Identifier: Apache-2.0 */

/* SoC level DTS fixup file */

#define DT_ADC_0_NAME			DT_NORDIC_NRF_SAADC_ADC_0_LABEL

#ifdef DT_NORDIC_NRF_UART_UART_0_LABEL
#define DT_UART_0_NAME			DT_NORDIC_NRF_UART_UART_0_LABEL
#else
#define DT_UART_0_NAME			DT_NORDIC_NRF_UARTE_UART_0_LABEL
#endif

#define DT_UART_1_NAME			DT_NORDIC_NRF_UARTE_UART_1_LABEL

#define DT_FLASH_DEV_NAME		DT_LABEL(DT_INST(0, nordic_nrf52_flash_controller))

#define DT_GPIO_P0_DEV_NAME		DT_NORDIC_NRF_GPIO_GPIO_0_LABEL
#define DT_GPIO_P1_DEV_NAME		DT_NORDIC_NRF_GPIO_GPIO_1_LABEL

#if defined(DT_NORDIC_NRF_SPIM_SPI_0_LABEL)
#define DT_SPI_0_NAME			DT_NORDIC_NRF_SPIM_SPI_0_LABEL
#else
#define DT_SPI_0_NAME			DT_NORDIC_NRF_SPI_SPI_0_LABEL
#endif

#if defined(DT_NORDIC_NRF_SPIM_SPI_1_LABEL)
#define DT_SPI_1_NAME			DT_NORDIC_NRF_SPIM_SPI_1_LABEL
#else
#define DT_SPI_1_NAME			DT_NORDIC_NRF_SPI_SPI_1_LABEL
#endif

#if defined(DT_NORDIC_NRF_SPIM_SPI_2_LABEL)
#define DT_SPI_2_NAME			DT_NORDIC_NRF_SPIM_SPI_2_LABEL
#else
#define DT_SPI_2_NAME			DT_NORDIC_NRF_SPI_SPI_2_LABEL
#endif

#define DT_SPI_3_NAME			DT_NORDIC_NRF_SPIM_SPI_3_LABEL

#define DT_WDT_0_NAME			DT_NORDIC_NRF_WATCHDOG_WDT_0_LABEL

#define DT_RTC_0_NAME			DT_NORDIC_NRF_RTC_RTC_0_LABEL
#define DT_RTC_1_NAME			DT_NORDIC_NRF_RTC_RTC_1_LABEL
#define DT_RTC_2_NAME			DT_NORDIC_NRF_RTC_RTC_2_LABEL

/* End of SoC Level DTS fixup file */
