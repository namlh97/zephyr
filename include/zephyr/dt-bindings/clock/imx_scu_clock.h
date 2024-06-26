/*
 * Copyright (c) 2024.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/dt-bindings/firmware/imx/rsrc.h>

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_IMX_SCU_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_IMX_SCU_H_
/*!
 * @brief LPCG TUPLE macors to map corresponding ip clock name, SCFW API resource index and LPCG Register base address.
 * The LPCG base should be 4KB aligned, if not it will be truncated.
 */
#define IMX_LPCG_TUPLE(rsrc, base)  ((((base) >> 12U) << 10) | (rsrc))
/*! @brief Get the resource index. */
#define IMX_LPCG_TUPLE_RSRC(tuple) ((uint32_t)(tuple)&0x3FFU)
/*!
 * @brief Peripheral clock name difinition used for clock gate, clock source
 * and clock divider setting. It is defined as the corresponding register address.
 */
#define IMX_CLOCK_M4_0_IRQSTEER        IMX_LPCG_TUPLE(IMX_SC_R_IRQSTR_M4_0, 0x00)
#define IMX_CLOCK_DMA_LPSPI0           IMX_LPCG_TUPLE(IMX_SC_R_SPI_0, 0x5A400000)
#define IMX_CLOCK_DMA_LPSPI1           IMX_LPCG_TUPLE(IMX_SC_R_SPI_1, 0x5A410000)
#define IMX_CLOCK_DMA_LPSPI2           IMX_LPCG_TUPLE(IMX_SC_R_SPI_2, 0x5A420000)
#define IMX_CLOCK_DMA_LPSPI3           IMX_LPCG_TUPLE(IMX_SC_R_SPI_3, 0x5A430000)
#define IMX_CLOCK_DMA_LPUART0          IMX_LPCG_TUPLE(IMX_SC_R_UART_0, 0x5A460000)
#define IMX_CLOCK_DMA_LPUART1          IMX_LPCG_TUPLE(IMX_SC_R_UART_1, 0x5A470000)
#define IMX_CLOCK_DMA_LPUART2          IMX_LPCG_TUPLE(IMX_SC_R_UART_2, 0x5A480000)
#define IMX_CLOCK_DMA_LPUART3          IMX_LPCG_TUPLE(IMX_SC_R_UART_3, 0x5A490000)
#define IMX_CLOCK_DMA_DMA0             IMX_LPCG_TUPLE(IMX_SC_R_DMA_0_CH0, 0x00)
#define IMX_CLOCK_DMA_LPI2C0           IMX_LPCG_TUPLE(IMX_SC_R_I2C_0, 0x5AC00000)
#define IMX_CLOCK_DMA_LPI2C1           IMX_LPCG_TUPLE(IMX_SC_R_I2C_1, 0x5AC10000)
#define IMX_CLOCK_DMA_LPI2C2           IMX_LPCG_TUPLE(IMX_SC_R_I2C_2, 0x5AC20000)
#define IMX_CLOCK_DMA_LPI2C3           IMX_LPCG_TUPLE(IMX_SC_R_I2C_3, 0x5AC30000)
#define IMX_CLOCK_DMA_FTM0             IMX_LPCG_TUPLE(IMX_SC_R_FTM_0, 0x5ACA0000)
#define IMX_CLOCK_DMA_FTM1             IMX_LPCG_TUPLE(IMX_SC_R_FTM_1, 0x5ACB0000)
#define IMX_CLOCK_DMA_CAN0             IMX_LPCG_TUPLE(IMX_SC_R_CAN_0, 0x5ACD0000)
#define IMX_CLOCK_DMA_CAN1             IMX_LPCG_TUPLE(IMX_SC_R_CAN_1, 0x5ACE0000)
#define IMX_CLOCK_DMA_CAN2             IMX_LPCG_TUPLE(IMX_SC_R_CAN_2, 0x5ACF0000)
#define IMX_CLOCK_HSIO_GPIO            IMX_LPCG_TUPLE(IMX_SC_R_HSIO_GPIO, 0x5F100000)
#define IMX_CLOCK_LVDS_0_LPI2C0        IMX_LPCG_TUPLE(IMX_SC_R_LVDS_0_I2C_0, 0x00)
#define IMX_CLOCK_LVDS_0_LPI2C1        IMX_LPCG_TUPLE(IMX_SC_R_LVDS_0_I2C_1, 0x00)
#define IMX_CLOCK_LVDS_1_LPI2C0        IMX_LPCG_TUPLE(IMX_SC_R_LVDS_1_I2C_0, 0x00)
#define IMX_CLOCK_LVDS_1_LPI2C1        IMX_LPCG_TUPLE(IMX_SC_R_LVDS_1_I2C_1, 0x00)
#define IMX_CLOCK_LSIO_PWM0            IMX_LPCG_TUPLE(IMX_SC_R_PWM_0, 0x5D400000)
#define IMX_CLOCK_LSIO_PWM1            IMX_LPCG_TUPLE(IMX_SC_R_PWM_1, 0x5D410000)
#define IMX_CLOCK_LSIO_PWM2            IMX_LPCG_TUPLE(IMX_SC_R_PWM_2, 0x5D420000)
#define IMX_CLOCK_LSIO_PWM3            IMX_LPCG_TUPLE(IMX_SC_R_PWM_3, 0x5D430000)
#define IMX_CLOCK_LSIO_PWM4            IMX_LPCG_TUPLE(IMX_SC_R_PWM_4, 0x5D440000)
#define IMX_CLOCK_LSIO_PWM5            IMX_LPCG_TUPLE(IMX_SC_R_PWM_5, 0x5D450000)
#define IMX_CLOCK_LSIO_PWM6            IMX_LPCG_TUPLE(IMX_SC_R_PWM_6, 0x5D460000)
#define IMX_CLOCK_LSIO_PWM7            IMX_LPCG_TUPLE(IMX_SC_R_PWM_7, 0x5D470000)
#define IMX_CLOCK_LSIO_GPIO0           IMX_LPCG_TUPLE(IMX_SC_R_GPIO_0, 0x5D480000)
#define IMX_CLOCK_LSIO_GPIO1           IMX_LPCG_TUPLE(IMX_SC_R_GPIO_1, 0x5D490000)
#define IMX_CLOCK_LSIO_GPIO2           IMX_LPCG_TUPLE(IMX_SC_R_GPIO_2, 0x5D4A0000)
#define IMX_CLOCK_LSIO_GPIO3           IMX_LPCG_TUPLE(IMX_SC_R_GPIO_3, 0x5D4B0000)
#define IMX_CLOCK_LSIO_GPIO4           IMX_LPCG_TUPLE(IMX_SC_R_GPIO_4, 0x5D4C0000)
#define IMX_CLOCK_LSIO_GPIO5           IMX_LPCG_TUPLE(IMX_SC_R_GPIO_5, 0x5D4D0000)
#define IMX_CLOCK_LSIO_GPIO6           IMX_LPCG_TUPLE(IMX_SC_R_GPIO_6, 0x5D4E0000)
#define IMX_CLOCK_LSIO_GPIO7           IMX_LPCG_TUPLE(IMX_SC_R_GPIO_7, 0x5D4F0000)
#define IMX_CLOCK_AUDIO_GPT0           IMX_LPCG_TUPLE(IMX_SC_R_GPT_5, 0x594B0000)
#define IMX_CLOCK_AUDIO_GPT1           IMX_LPCG_TUPLE(IMX_SC_R_GPT_6, 0x594C0000)
#define IMX_CLOCK_AUDIO_GPT2           IMX_LPCG_TUPLE(IMX_SC_R_GPT_7, 0x594D0000)
#define IMX_CLOCK_AUDIO_GPT3           IMX_LPCG_TUPLE(IMX_SC_R_GPT_8, 0x594E0000)
#define IMX_CLOCK_AUDIO_GPT4           IMX_LPCG_TUPLE(IMX_SC_R_GPT_9, 0x594F0000)
#define IMX_CLOCK_AUDIO_GPT5           IMX_LPCG_TUPLE(IMX_SC_R_GPT_10, 0x59500000)
#define IMX_CLOCK_LSIO_GPT0            IMX_LPCG_TUPLE(IMX_SC_R_GPT_0, 0x5D540000)
#define IMX_CLOCK_LSIO_GPT1            IMX_LPCG_TUPLE(IMX_SC_R_GPT_1, 0x5D550000)
#define IMX_CLOCK_LSIO_GPT2            IMX_LPCG_TUPLE(IMX_SC_R_GPT_2, 0x5D560000)
#define IMX_CLOCK_LSIO_GPT3            IMX_LPCG_TUPLE(IMX_SC_R_GPT_3, 0x5D570000)
#define IMX_CLOCK_LSIO_GPT4            IMX_LPCG_TUPLE(IMX_SC_R_GPT_4, 0x5D580000)
#define IMX_CLOCK_LSIO_MU0A            IMX_LPCG_TUPLE(IMX_SC_R_MU_0A, 0x00)
#define IMX_CLOCK_LSIO_MU1A            IMX_LPCG_TUPLE(IMX_SC_R_MU_1A, 0x00)
#define IMX_CLOCK_LSIO_MU2A            IMX_LPCG_TUPLE(IMX_SC_R_MU_2A, 0x00)
#define IMX_CLOCK_LSIO_MU3A            IMX_LPCG_TUPLE(IMX_SC_R_MU_3A, 0x00)
#define IMX_CLOCK_LSIO_MU4A            IMX_LPCG_TUPLE(IMX_SC_R_MU_4A, 0x00)
#define IMX_CLOCK_LSIO_MU5A            IMX_LPCG_TUPLE(IMX_SC_R_MU_5A, 0x5D600000)
#define IMX_CLOCK_LSIO_MU6A            IMX_LPCG_TUPLE(IMX_SC_R_MU_6A, 0x5D610000)
#define IMX_CLOCK_LSIO_MU7A            IMX_LPCG_TUPLE(IMX_SC_R_MU_7A, 0x5D620000)
#define IMX_CLOCK_LSIO_MU8A            IMX_LPCG_TUPLE(IMX_SC_R_MU_8A, 0x5D630000)
#define IMX_CLOCK_LSIO_MU9A            IMX_LPCG_TUPLE(IMX_SC_R_MU_9A, 0x5D640000)
#define IMX_CLOCK_LSIO_MU10A           IMX_LPCG_TUPLE(IMX_SC_R_MU_10A, 0x5D650000)
#define IMX_CLOCK_LSIO_MU11A           IMX_LPCG_TUPLE(IMX_SC_R_MU_11A, 0x5D660000)
#define IMX_CLOCK_LSIO_MU12A           IMX_LPCG_TUPLE(IMX_SC_R_MU_12A, 0x5D670000)
#define IMX_CLOCK_LSIO_MU13A           IMX_LPCG_TUPLE(IMX_SC_R_MU_13A, 0x5D680000)
#define IMX_CLOCK_LSIO_MU5B            IMX_LPCG_TUPLE(IMX_SC_R_MU_5B, 0x5D690000)
#define IMX_CLOCK_LSIO_MU6B            IMX_LPCG_TUPLE(IMX_SC_R_MU_6B, 0x5D6A0000)
#define IMX_CLOCK_LSIO_MU7B            IMX_LPCG_TUPLE(IMX_SC_R_MU_7B, 0x5D6B0000)
#define IMX_CLOCK_LSIO_MU8B            IMX_LPCG_TUPLE(IMX_SC_R_MU_8B, 0x5D6C0000)
#define IMX_CLOCK_LSIO_MU9B            IMX_LPCG_TUPLE(IMX_SC_R_MU_9B, 0x5D6D0000)
#define IMX_CLOCK_LSIO_MU10B           IMX_LPCG_TUPLE(IMX_SC_R_MU_10B, 0x5D6E0000)
#define IMX_CLOCK_LSIO_MU11B           IMX_LPCG_TUPLE(IMX_SC_R_MU_11B, 0x5D6F0000)
#define IMX_CLOCK_LSIO_MU12B           IMX_LPCG_TUPLE(IMX_SC_R_MU_12B, 0x5D700000)
#define IMX_CLOCK_LSIO_MU13B           IMX_LPCG_TUPLE(IMX_SC_R_MU_13B, 0x5D710000)
#define IMX_CLOCK_SCU_MU0B             IMX_LPCG_TUPLE(IMX_SC_R_SC_MU_0B, 0x00)
#define IMX_CLOCK_SCU_MU0A0            IMX_LPCG_TUPLE(IMX_SC_R_SC_MU_0A0, 0x00)
#define IMX_CLOCK_SCU_MU0A1            IMX_LPCG_TUPLE(IMX_SC_R_SC_MU_0A1, 0x00)
#define IMX_CLOCK_SCU_MU0A2            IMX_LPCG_TUPLE(IMX_SC_R_SC_MU_0A2, 0x00)
#define IMX_CLOCK_SCU_MU0A3            IMX_LPCG_TUPLE(IMX_SC_R_SC_MU_0A3, 0x00)
#define IMX_CLOCK_SCU_MU1A             IMX_LPCG_TUPLE(IMX_SC_R_SC_MU_1A, 0x00)
#define IMX_CLOCK_LSIO_FLEXSPI0        IMX_LPCG_TUPLE(IMX_SC_R_FSPI_0, 0x5D520000)
#define IMX_CLOCK_LSIO_FLEXSPI1        IMX_LPCG_TUPLE(IMX_SC_R_FSPI_1, 0x5D530000)
#define IMX_CLOCK_M4_0_RGPIO           IMX_LPCG_TUPLE(IMX_SC_R_M4_0_RGPIO, 0x00)
#define IMX_CLOCK_M4_0_SEMA42          IMX_LPCG_TUPLE(IMX_SC_R_M4_0_SEMA42, 0x00)
#define IMX_CLOCK_M4_0_TPM             IMX_LPCG_TUPLE(IMX_SC_R_M4_0_TPM, 0x41600000)
#define IMX_CLOCK_M4_0_LPIT            IMX_LPCG_TUPLE(IMX_SC_R_M4_0_PIT, 0x41610000)
#define IMX_CLOCK_M4_0_LPUART          IMX_LPCG_TUPLE(IMX_SC_R_M4_0_UART, 0x41620000)
#define IMX_CLOCK_M4_0_LPI2C           IMX_LPCG_TUPLE(IMX_SC_R_M4_0_I2C, 0x41630000)
#define IMX_CLOCK_M4_0_INTMUX          IMX_LPCG_TUPLE(IMX_SC_R_M4_0_INTMUX, 0x00)
#define IMX_CLOCK_M4_0_MU0B            IMX_LPCG_TUPLE(IMX_SC_R_M4_0_MU_0B, 0x00)
#define IMX_CLOCK_M4_0_MU0A0           IMX_LPCG_TUPLE(IMX_SC_R_M4_0_MU_0A0, 0x00)
#define IMX_CLOCK_M4_0_MU0A1           IMX_LPCG_TUPLE(IMX_SC_R_M4_0_MU_0A1, 0x00)
#define IMX_CLOCK_M4_0_MU0A2           IMX_LPCG_TUPLE(IMX_SC_R_M4_0_MU_0A2, 0x00)
#define IMX_CLOCK_M4_0_MU0A3           IMX_LPCG_TUPLE(IMX_SC_R_M4_0_MU_0A3, 0x00)
#define IMX_CLOCK_M4_0_MU1A            IMX_LPCG_TUPLE(IMX_SC_R_M4_0_MU_1A, 0x00)
#define IMX_CLOCK_SCU_LPUART           IMX_LPCG_TUPLE(IMX_SC_R_SC_UART, 0x33620000)
#define IMX_CLOCK_ADMA_LPADC0          IMX_LPCG_TUPLE(IMX_SC_R_ADC_0, 0x00)
#define IMX_CLOCK_SCU_LPI2C            IMX_LPCG_TUPLE(IMX_SC_R_SC_I2C, 0x33630000)
#define IMX_CLOCK_SCU_SEMA42           IMX_LPCG_TUPLE(IMX_SC_R_SC_SEMA42, 0x00)
#define IMX_CLOCK_SCU_LPIT             IMX_LPCG_TUPLE(IMX_SC_R_SC_PIT, 0x33610000)
#define IMX_CLOCK_SCU_TPM              IMX_LPCG_TUPLE(IMX_SC_R_SC_TPM, 0x33600000)
#define IMX_CLOCK_SCU_INTMUX           IMX_LPCG_TUPLE(IMX_SC_R_LAST, 0x00)
#define IMX_CLOCK_AUDIO_SAI0           IMX_LPCG_TUPLE(IMX_SC_R_SAI_0, 0x59440000)
#define IMX_CLOCK_AUDIO_SAI1           IMX_LPCG_TUPLE(IMX_SC_R_SAI_1, 0x59450000)
#define IMX_CLOCK_AUDIO_SAI2           IMX_LPCG_TUPLE(IMX_SC_R_SAI_2, 0x59460000)
#define IMX_CLOCK_AUDIO_SAI3           IMX_LPCG_TUPLE(IMX_SC_R_SAI_3, 0x59470000)
#define IMX_CLOCK_AUDIO_SAI4           IMX_LPCG_TUPLE(IMX_SC_R_SAI_4, 0x59C20000)
#define IMX_CLOCK_AUDIO_SAI5           IMX_LPCG_TUPLE(IMX_SC_R_SAI_5, 0x59C30000)
#define IMX_CLOCK_AUDIO_ESAI0          IMX_LPCG_TUPLE(IMX_SC_R_ESAI_0, 0x59410000)
#define IMX_CLOCK_AUDIO_ESAI1          IMX_LPCG_TUPLE(IMX_SC_R_ESAI_0, 0x00)
#define IMX_CLOCK_IMAGING_ISI0         IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH0, 0x00)
#define IMX_CLOCK_IMAGING_ISI1         IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH1, 0x00)
#define IMX_CLOCK_IMAGING_ISI2         IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH2, 0x00)
#define IMX_CLOCK_IMAGING_ISI3         IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH3, 0x00)
#define IMX_CLOCK_IMAGING_ISI4         IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH4, 0x00)
#define IMX_CLOCK_IMAGING_ISI5         IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH5, 0x00)
#define IMX_CLOCK_MIPICSI2RX0          IMX_LPCG_TUPLE(IMX_SC_R_CSI_0, 0x00)
#define IMX_CLOCK_MIPICSI2RX1          IMX_LPCG_TUPLE(IMX_SC_R_CSI_1, 0x00)
#define IMX_CLOCK_DIMIPIDSILVDS0LPI2C0 IMX_LPCG_TUPLE(IMX_SC_R_MIPI_0_I2C_0, 0x56223000)
#define IMX_CLOCK_DIMIPIDSILVDS0LPI2C1 IMX_LPCG_TUPLE(IMX_SC_R_MIPI_0_I2C_1, 0x56223000)
#define IMX_CLOCK_DIMIPIDSILVDS1LPI2C0 IMX_LPCG_TUPLE(IMX_SC_R_MIPI_1_I2C_0, 0x56243000)
#define IMX_CLOCK_DIMIPIDSILVDS1LPI2C1 IMX_LPCG_TUPLE(IMX_SC_R_MIPI_1_I2C_1, 0x56243000)
#define IMX_CLOCK_CIPILPI2C            IMX_LPCG_TUPLE(IMX_SC_R_LAST, 0x00)
#define IMX_CLOCK_MIPICSILPI2C         IMX_LPCG_TUPLE(IMX_SC_R_CSI_0_I2C_0, 0x00)
#define IMX_CLOCK_MIPIDSIHOST0         IMX_LPCG_TUPLE(IMX_SC_R_MIPI_0, 0x00)
#define IMX_CLOCK_MIPIDSIHOST1         IMX_LPCG_TUPLE(IMX_SC_R_MIPI_1, 0x00)
#define IMX_CLOCK_DPU0                 IMX_LPCG_TUPLE(IMX_SC_R_DC_0, 0x56010000)
#define IMX_CLOCK_DPU1                 IMX_LPCG_TUPLE(IMX_SC_R_DC_1, 0x00)
#define IMX_CLOCK_HDMI_LPI2C0          IMX_LPCG_TUPLE(IMX_SC_R_HDMI_I2C_0, 0x00)
#define IMX_CLOCK_HDMI_RX_LPI2C0       IMX_LPCG_TUPLE(IMX_SC_R_HDMI_RX_I2C_0, 0x00)
#define IMX_CLOCK_LDB0                 IMX_LPCG_TUPLE(IMX_SC_R_LVDS_0, 0x00)
#define IMX_CLOCK_LDB1                 IMX_LPCG_TUPLE(IMX_SC_R_LVDS_1, 0x00)
#define IMX_CLOCK_CONNECTIVITY_ENET0   IMX_LPCG_TUPLE(IMX_SC_R_ENET_0, 0x5B230000)
#define IMX_CLOCK_CONNECTIVITY_ENET1   IMX_LPCG_TUPLE(IMX_SC_R_ENET_1, 0x5B240000)
#define IMX_CLOCK_CONNECTIVITY_USDHC0  IMX_LPCG_TUPLE(IMX_SC_R_SDHC_0, 0x5B200000)
#define IMX_CLOCK_CONNECTIVITY_USDHC1  IMX_LPCG_TUPLE(IMX_SC_R_SDHC_1, 0x5B210000)
#define IMX_CLOCK_AUDIO_PLL0           IMX_LPCG_TUPLE(IMX_SC_R_AUDIO_PLL_0, 0x00)
#define IMX_CLOCK_AUDIO_PLL1           IMX_LPCG_TUPLE(IMX_SC_R_AUDIO_PLL_1, 0x00)
#define IMX_CLOCK_CAAM_JR1             IMX_LPCG_TUPLE(IMX_SC_R_CAAM_JR1, 0x00)
#define IMX_CLOCK_CAAM_JR2             IMX_LPCG_TUPLE(IMX_SC_R_CAAM_JR2, 0x00)
#define IMX_CLOCK_CAAM_JR3             IMX_LPCG_TUPLE(IMX_SC_R_CAAM_JR3, 0x00)
#define IMX_CLOCK_CIPI0                IMX_LPCG_TUPLE(IMX_SC_R_PI_0, 0x00)
#define IMX_CLOCK_ISI0                 IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH0, 0x00)
#define IMX_CLOCK_ISI1                 IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH1, 0x00)
#define IMX_CLOCK_ISI2                 IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH2, 0x00)
#define IMX_CLOCK_ISI3                 IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH3, 0x00)
#define IMX_CLOCK_ISI4                 IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH4, 0x00)
#define IMX_CLOCK_ISI5                 IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH5, 0x00)
#define IMX_CLOCK_ISI6                 IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH6, 0x00)
#define IMX_CLOCK_ISI7                 IMX_LPCG_TUPLE(IMX_SC_R_ISI_CH7, 0x00)
#define IMX_CLOCK_IPI0x00ALID          IMX_LPCG_TUPLE(IMX_SC_R_LAST, 0x00) /* The selected IP does not support clock control. */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_IMX_SCU_H_ */
