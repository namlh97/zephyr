/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IMX8QXP_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IMX8QXP_PINCTRL_H_

#define SC_P_PCIE_CTRL0_PERST_B                  0    /*!< HSIO.PCIE0.PERST_B, LSIO.GPIO4.IO00 */
#define SC_P_PCIE_CTRL0_CLKREQ_B                 1    /*!< HSIO.PCIE0.CLKREQ_B, LSIO.GPIO4.IO01 */
#define SC_P_PCIE_CTRL0_WAKE_B                   2    /*!< HSIO.PCIE0.WAKE_B, LSIO.GPIO4.IO02 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_PCIESEP       3    /*!<  */
#define SC_P_USB_SS3_TC0                         4    /*!< ADMA.I2C1.SCL, CONN.USB_OTG1.PWR, CONN.USB_OTG2.PWR, LSIO.GPIO4.IO03 */
#define SC_P_USB_SS3_TC1                         5    /*!< ADMA.I2C1.SCL, CONN.USB_OTG2.PWR, LSIO.GPIO4.IO04 */
#define SC_P_USB_SS3_TC2                         6    /*!< ADMA.I2C1.SDA, CONN.USB_OTG1.OC, CONN.USB_OTG2.OC, LSIO.GPIO4.IO05 */
#define SC_P_USB_SS3_TC3                         7    /*!< ADMA.I2C1.SDA, CONN.USB_OTG2.OC, LSIO.GPIO4.IO06 */
#define SC_P_COMP_CTL_GPIO_3V3_USB3IO            8    /*!<  */
#define SC_P_EMMC0_CLK                           9    /*!< CONN.EMMC0.CLK, CONN.NAND.READY_B, LSIO.GPIO4.IO07 */
#define SC_P_EMMC0_CMD                           10   /*!< CONN.EMMC0.CMD, CONN.NAND.DQS, LSIO.GPIO4.IO08 */
#define SC_P_EMMC0_DATA0                         11   /*!< CONN.EMMC0.DATA0, CONN.NAND.DATA00, LSIO.GPIO4.IO09 */
#define SC_P_EMMC0_DATA1                         12   /*!< CONN.EMMC0.DATA1, CONN.NAND.DATA01, LSIO.GPIO4.IO10 */
#define SC_P_EMMC0_DATA2                         13   /*!< CONN.EMMC0.DATA2, CONN.NAND.DATA02, LSIO.GPIO4.IO11 */
#define SC_P_EMMC0_DATA3                         14   /*!< CONN.EMMC0.DATA3, CONN.NAND.DATA03, LSIO.GPIO4.IO12 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_SD1FIX0       15   /*!<  */
#define SC_P_EMMC0_DATA4                         16   /*!< CONN.EMMC0.DATA4, CONN.NAND.DATA04, CONN.EMMC0.WP, LSIO.GPIO4.IO13 */
#define SC_P_EMMC0_DATA5                         17   /*!< CONN.EMMC0.DATA5, CONN.NAND.DATA05, CONN.EMMC0.VSELECT, LSIO.GPIO4.IO14 */
#define SC_P_EMMC0_DATA6                         18   /*!< CONN.EMMC0.DATA6, CONN.NAND.DATA06, CONN.MLB.CLK, LSIO.GPIO4.IO15 */
#define SC_P_EMMC0_DATA7                         19   /*!< CONN.EMMC0.DATA7, CONN.NAND.DATA07, CONN.MLB.SIG, LSIO.GPIO4.IO16 */
#define SC_P_EMMC0_STROBE                        20   /*!< CONN.EMMC0.STROBE, CONN.NAND.CLE, CONN.MLB.DATA, LSIO.GPIO4.IO17 */
#define SC_P_EMMC0_RESET_B                       21   /*!< CONN.EMMC0.RESET_B, CONN.NAND.WP_B, LSIO.GPIO4.IO18 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_SD1FIX1       22   /*!<  */
#define SC_P_USDHC1_RESET_B                      23   /*!< CONN.USDHC1.RESET_B, CONN.NAND.RE_N, ADMA.SPI2.SCK, LSIO.GPIO4.IO19 */
#define SC_P_USDHC1_VSELECT                      24   /*!< CONN.USDHC1.VSELECT, CONN.NAND.RE_P, ADMA.SPI2.SDO, CONN.NAND.RE_B, LSIO.GPIO4.IO20 */
#define SC_P_CTL_NAND_RE_P_N                     25   /*!<  */
#define SC_P_USDHC1_WP                           26   /*!< CONN.USDHC1.WP, CONN.NAND.DQS_N, ADMA.SPI2.SDI, LSIO.GPIO4.IO21 */
#define SC_P_USDHC1_CD_B                         27   /*!< CONN.USDHC1.CD_B, CONN.NAND.DQS_P, ADMA.SPI2.CS0, CONN.NAND.DQS, LSIO.GPIO4.IO22 */
#define SC_P_CTL_NAND_DQS_P_N                    28   /*!<  */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSELSEP       29   /*!<  */
#define SC_P_USDHC1_CLK                          30   /*!< CONN.USDHC1.CLK, ADMA.UART3.RX, LSIO.GPIO4.IO23 */
#define SC_P_USDHC1_CMD                          31   /*!< CONN.USDHC1.CMD, CONN.NAND.CE0_B, ADMA.MQS.R, LSIO.GPIO4.IO24 */
#define SC_P_USDHC1_DATA0                        32   /*!< CONN.USDHC1.DATA0, CONN.NAND.CE1_B, ADMA.MQS.L, LSIO.GPIO4.IO25 */
#define SC_P_USDHC1_DATA1                        33   /*!< CONN.USDHC1.DATA1, CONN.NAND.RE_B, ADMA.UART3.TX, LSIO.GPIO4.IO26 */
#define SC_P_USDHC1_DATA2                        34   /*!< CONN.USDHC1.DATA2, CONN.NAND.WE_B, ADMA.UART3.CTS_B, LSIO.GPIO4.IO27 */
#define SC_P_USDHC1_DATA3                        35   /*!< CONN.USDHC1.DATA3, CONN.NAND.ALE, ADMA.UART3.RTS_B, LSIO.GPIO4.IO28 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSEL3         36   /*!<  */
#define SC_P_ENET0_RGMII_TXC                     37   /*!< CONN.ENET0.RGMII_TXC, CONN.ENET0.RCLK50M_OUT, CONN.ENET0.RCLK50M_IN, CONN.NAND.CE1_B, LSIO.GPIO4.IO29 */
#define SC_P_ENET0_RGMII_TX_CTL                  38   /*!< CONN.ENET0.RGMII_TX_CTL, CONN.USDHC1.RESET_B, LSIO.GPIO4.IO30 */
#define SC_P_ENET0_RGMII_TXD0                    39   /*!< CONN.ENET0.RGMII_TXD0, CONN.USDHC1.VSELECT, LSIO.GPIO4.IO31 */
#define SC_P_ENET0_RGMII_TXD1                    40   /*!< CONN.ENET0.RGMII_TXD1, CONN.USDHC1.WP, LSIO.GPIO5.IO00 */
#define SC_P_ENET0_RGMII_TXD2                    41   /*!< CONN.ENET0.RGMII_TXD2, CONN.MLB.CLK, CONN.NAND.CE0_B, CONN.USDHC1.CD_B, LSIO.GPIO5.IO01 */
#define SC_P_ENET0_RGMII_TXD3                    42   /*!< CONN.ENET0.RGMII_TXD3, CONN.MLB.SIG, CONN.NAND.RE_B, LSIO.GPIO5.IO02 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_ENET_ENETB0   43   /*!<  */
#define SC_P_ENET0_RGMII_RXC                     44   /*!< CONN.ENET0.RGMII_RXC, CONN.MLB.DATA, CONN.NAND.WE_B, CONN.USDHC1.CLK, LSIO.GPIO5.IO03 */
#define SC_P_ENET0_RGMII_RX_CTL                  45   /*!< CONN.ENET0.RGMII_RX_CTL, CONN.USDHC1.CMD, LSIO.GPIO5.IO04 */
#define SC_P_ENET0_RGMII_RXD0                    46   /*!< CONN.ENET0.RGMII_RXD0, CONN.USDHC1.DATA0, LSIO.GPIO5.IO05 */
#define SC_P_ENET0_RGMII_RXD1                    47   /*!< CONN.ENET0.RGMII_RXD1, CONN.USDHC1.DATA1, LSIO.GPIO5.IO06 */
#define SC_P_ENET0_RGMII_RXD2                    48   /*!< CONN.ENET0.RGMII_RXD2, CONN.ENET0.RMII_RX_ER, CONN.USDHC1.DATA2, LSIO.GPIO5.IO07 */
#define SC_P_ENET0_RGMII_RXD3                    49   /*!< CONN.ENET0.RGMII_RXD3, CONN.NAND.ALE, CONN.USDHC1.DATA3, LSIO.GPIO5.IO08 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_ENET_ENETB1   50   /*!<  */
#define SC_P_ENET0_REFCLK_125M_25M               51   /*!< CONN.ENET0.REFCLK_125M_25M, CONN.ENET0.PPS, CONN.ENET1.PPS, LSIO.GPIO5.IO09 */
#define SC_P_ENET0_MDIO                          52   /*!< CONN.ENET0.MDIO, ADMA.I2C3.SDA, CONN.ENET1.MDIO, LSIO.GPIO5.IO10 */
#define SC_P_ENET0_MDC                           53   /*!< CONN.ENET0.MDC, ADMA.I2C3.SCL, CONN.ENET1.MDC, LSIO.GPIO5.IO11 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOCT        54   /*!<  */
#define SC_P_ESAI0_FSR                           55   /*!< ADMA.ESAI0.FSR, CONN.ENET1.RCLK50M_OUT, ADMA.LCDIF.D00, CONN.ENET1.RGMII_TXC, CONN.ENET1.RCLK50M_IN */
#define SC_P_ESAI0_FST                           56   /*!< ADMA.ESAI0.FST, CONN.MLB.CLK, ADMA.LCDIF.D01, CONN.ENET1.RGMII_TXD2, LSIO.GPIO0.IO01 */
#define SC_P_ESAI0_SCKR                          57   /*!< ADMA.ESAI0.SCKR, ADMA.LCDIF.D02, CONN.ENET1.RGMII_TX_CTL, LSIO.GPIO0.IO02 */
#define SC_P_ESAI0_SCKT                          58   /*!< ADMA.ESAI0.SCKT, CONN.MLB.SIG, ADMA.LCDIF.D03, CONN.ENET1.RGMII_TXD3, LSIO.GPIO0.IO03 */
#define SC_P_ESAI0_TX0                           59   /*!< ADMA.ESAI0.TX0, CONN.MLB.DATA, ADMA.LCDIF.D04, CONN.ENET1.RGMII_RXC, LSIO.GPIO0.IO04 */
#define SC_P_ESAI0_TX1                           60   /*!< ADMA.ESAI0.TX1, ADMA.LCDIF.D05, CONN.ENET1.RGMII_RXD3, LSIO.GPIO0.IO05 */
#define SC_P_ESAI0_TX2_RX3                       61   /*!< ADMA.ESAI0.TX2_RX3, CONN.ENET1.RMII_RX_ER, ADMA.LCDIF.D06, CONN.ENET1.RGMII_RXD2, LSIO.GPIO0.IO06 */
#define SC_P_ESAI0_TX3_RX2                       62   /*!< ADMA.ESAI0.TX3_RX2, ADMA.LCDIF.D07, CONN.ENET1.RGMII_RXD1, LSIO.GPIO0.IO07 */
#define SC_P_ESAI0_TX4_RX1                       63   /*!< ADMA.ESAI0.TX4_RX1, ADMA.LCDIF.D08, CONN.ENET1.RGMII_TXD0, LSIO.GPIO0.IO08 */
#define SC_P_ESAI0_TX5_RX0                       64   /*!< ADMA.ESAI0.TX5_RX0, ADMA.LCDIF.D09, CONN.ENET1.RGMII_TXD1, LSIO.GPIO0.IO09 */
#define SC_P_SPDIF0_RX                           65   /*!< ADMA.SPDIF0.RX, ADMA.MQS.R, ADMA.LCDIF.D10, CONN.ENET1.RGMII_RXD0, LSIO.GPIO0.IO10 */
#define SC_P_SPDIF0_TX                           66   /*!< ADMA.SPDIF0.TX, ADMA.MQS.L, ADMA.LCDIF.D11, CONN.ENET1.RGMII_RX_CTL, LSIO.GPIO0.IO11 */
#define SC_P_SPDIF0_EXT_CLK                      67   /*!< ADMA.SPDIF0.EXT_CLK, ADMA.LCDIF.D12, CONN.ENET1.REFCLK_125M_25M, LSIO.GPIO0.IO12 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHB       68   /*!<  */
#define SC_P_SPI3_SCK                            69   /*!< ADMA.SPI3.SCK, ADMA.LCDIF.D13, LSIO.GPIO0.IO13 */
#define SC_P_SPI3_SDO                            70   /*!< ADMA.SPI3.SDO, ADMA.LCDIF.D14, LSIO.GPIO0.IO14 */
#define SC_P_SPI3_SDI                            71   /*!< ADMA.SPI3.SDI, ADMA.LCDIF.D15, LSIO.GPIO0.IO15 */
#define SC_P_SPI3_CS0                            72   /*!< ADMA.SPI3.CS0, ADMA.ACM.MCLK_OUT1, ADMA.LCDIF.HSYNC, LSIO.GPIO0.IO16 */
#define SC_P_SPI3_CS1                            73   /*!< ADMA.SPI3.CS1, ADMA.I2C3.SCL, ADMA.LCDIF.RESET, ADMA.SPI2.CS0, ADMA.LCDIF.D16 */
#define SC_P_MCLK_IN1                            74   /*!< ADMA.ACM.MCLK_IN1, ADMA.I2C3.SDA, ADMA.LCDIF.EN, ADMA.SPI2.SCK, ADMA.LCDIF.D17 */
#define SC_P_MCLK_IN0                            75   /*!< ADMA.ACM.MCLK_IN0, ADMA.ESAI0.RX_HF_CLK, ADMA.LCDIF.VSYNC, ADMA.SPI2.SDI, LSIO.GPIO0.IO19 */
#define SC_P_MCLK_OUT0                           76   /*!< ADMA.ACM.MCLK_OUT0, ADMA.ESAI0.TX_HF_CLK, ADMA.LCDIF.CLK, ADMA.SPI2.SDO, LSIO.GPIO0.IO20 */
#define SC_P_UART1_TX                            77   /*!< ADMA.UART1.TX, LSIO.PWM0.OUT, LSIO.GPT0.CAPTURE, LSIO.GPIO0.IO21 */
#define SC_P_UART1_RX                            78   /*!< ADMA.UART1.RX, LSIO.PWM1.OUT, LSIO.GPT0.COMPARE, LSIO.GPT1.CLK, LSIO.GPIO0.IO22 */
#define SC_P_UART1_RTS_B                         79   /*!< ADMA.UART1.RTS_B, LSIO.PWM2.OUT, ADMA.LCDIF.D16, LSIO.GPT1.CAPTURE, LSIO.GPT0.CLK */
#define SC_P_UART1_CTS_B                         80   /*!< ADMA.UART1.CTS_B, LSIO.PWM3.OUT, ADMA.LCDIF.D17, LSIO.GPT1.COMPARE, LSIO.GPIO0.IO24 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHK       81   /*!<  */
#define SC_P_SAI0_TXD                            82   /*!< ADMA.SAI0.TXD, ADMA.SAI1.RXC, ADMA.SPI1.SDO, ADMA.LCDIF.D18, LSIO.GPIO0.IO25 */
#define SC_P_SAI0_TXC                            83   /*!< ADMA.SAI0.TXC, ADMA.SAI1.TXD, ADMA.SPI1.SDI, ADMA.LCDIF.D19, LSIO.GPIO0.IO26 */
#define SC_P_SAI0_RXD                            84   /*!< ADMA.SAI0.RXD, ADMA.SAI1.RXFS, ADMA.SPI1.CS0, ADMA.LCDIF.D20, LSIO.GPIO0.IO27 */
#define SC_P_SAI0_TXFS                           85   /*!< ADMA.SAI0.TXFS, ADMA.SPI2.CS1, ADMA.SPI1.SCK, LSIO.GPIO0.IO28 */
#define SC_P_SAI1_RXD                            86   /*!< ADMA.SAI1.RXD, ADMA.SAI0.RXFS, ADMA.SPI1.CS1, ADMA.LCDIF.D21, LSIO.GPIO0.IO29 */
#define SC_P_SAI1_RXC                            87   /*!< ADMA.SAI1.RXC, ADMA.SAI1.TXC, ADMA.LCDIF.D22, LSIO.GPIO0.IO30 */
#define SC_P_SAI1_RXFS                           88   /*!< ADMA.SAI1.RXFS, ADMA.SAI1.TXFS, ADMA.LCDIF.D23, LSIO.GPIO0.IO31 */
#define SC_P_SPI2_CS0                            89   /*!< ADMA.SPI2.CS0, LSIO.GPIO1.IO00 */
#define SC_P_SPI2_SDO                            90   /*!< ADMA.SPI2.SDO, LSIO.GPIO1.IO01 */
#define SC_P_SPI2_SDI                            91   /*!< ADMA.SPI2.SDI, LSIO.GPIO1.IO02 */
#define SC_P_SPI2_SCK                            92   /*!< ADMA.SPI2.SCK, LSIO.GPIO1.IO03 */
#define SC_P_SPI0_SCK                            93   /*!< ADMA.SPI0.SCK, ADMA.SAI0.TXC, M40.I2C0.SCL, M40.GPIO0.IO00, LSIO.GPIO1.IO04 */
#define SC_P_SPI0_SDI                            94   /*!< ADMA.SPI0.SDI, ADMA.SAI0.TXD, M40.TPM0.CH0, M40.GPIO0.IO02, LSIO.GPIO1.IO05 */
#define SC_P_SPI0_SDO                            95   /*!< ADMA.SPI0.SDO, ADMA.SAI0.TXFS, M40.I2C0.SDA, M40.GPIO0.IO01, LSIO.GPIO1.IO06 */
#define SC_P_SPI0_CS1                            96   /*!< ADMA.SPI0.CS1, ADMA.SAI0.RXC, ADMA.SAI1.TXD, ADMA.LCD_PWM0.OUT, LSIO.GPIO1.IO07 */
#define SC_P_SPI0_CS0                            97   /*!< ADMA.SPI0.CS0, ADMA.SAI0.RXD, M40.TPM0.CH1, M40.GPIO0.IO03, LSIO.GPIO1.IO08 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHT       98   /*!<  */
#define SC_P_ADC_IN1                             99   /*!< ADMA.ADC.IN1, M40.I2C0.SDA, M40.GPIO0.IO01, LSIO.GPIO1.IO09 */
#define SC_P_ADC_IN0                             100  /*!< ADMA.ADC.IN0, M40.I2C0.SCL, M40.GPIO0.IO00, LSIO.GPIO1.IO10 */
#define SC_P_ADC_IN3                             101  /*!< ADMA.ADC.IN3, M40.UART0.TX, M40.GPIO0.IO03, ADMA.ACM.MCLK_OUT0, LSIO.GPIO1.IO11 */
#define SC_P_ADC_IN2                             102  /*!< ADMA.ADC.IN2, M40.UART0.RX, M40.GPIO0.IO02, ADMA.ACM.MCLK_IN0, LSIO.GPIO1.IO12 */
#define SC_P_ADC_IN5                             103  /*!< ADMA.ADC.IN5, M40.TPM0.CH1, M40.GPIO0.IO05, LSIO.GPIO1.IO13 */
#define SC_P_ADC_IN4                             104  /*!< ADMA.ADC.IN4, M40.TPM0.CH0, M40.GPIO0.IO04, LSIO.GPIO1.IO14 */
#define SC_P_FLEXCAN0_RX                         105  /*!< ADMA.FLEXCAN0.RX, ADMA.SAI2.RXC, ADMA.UART0.RTS_B, ADMA.SAI1.TXC, LSIO.GPIO1.IO15 */
#define SC_P_FLEXCAN0_TX                         106  /*!< ADMA.FLEXCAN0.TX, ADMA.SAI2.RXD, ADMA.UART0.CTS_B, ADMA.SAI1.TXFS, LSIO.GPIO1.IO16 */
#define SC_P_FLEXCAN1_RX                         107  /*!< ADMA.FLEXCAN1.RX, ADMA.SAI2.RXFS, ADMA.FTM.CH2, ADMA.SAI1.TXD, LSIO.GPIO1.IO17 */
#define SC_P_FLEXCAN1_TX                         108  /*!< ADMA.FLEXCAN1.TX, ADMA.SAI3.RXC, ADMA.DMA0.REQ_IN0, ADMA.SAI1.RXD, LSIO.GPIO1.IO18 */
#define SC_P_FLEXCAN2_RX                         109  /*!< ADMA.FLEXCAN2.RX, ADMA.SAI3.RXD, ADMA.UART3.RX, ADMA.SAI1.RXFS, LSIO.GPIO1.IO19 */
#define SC_P_FLEXCAN2_TX                         110  /*!< ADMA.FLEXCAN2.TX, ADMA.SAI3.RXFS, ADMA.UART3.TX, ADMA.SAI1.RXC, LSIO.GPIO1.IO20 */
#define SC_P_UART0_RX                            111  /*!< ADMA.UART0.RX, ADMA.MQS.R, ADMA.FLEXCAN0.RX, SCU.UART0.RX, LSIO.GPIO1.IO21 */
#define SC_P_UART0_TX                            112  /*!< ADMA.UART0.TX, ADMA.MQS.L, ADMA.FLEXCAN0.TX, SCU.UART0.TX, LSIO.GPIO1.IO22 */
#define SC_P_UART2_TX                            113  /*!< ADMA.UART2.TX, ADMA.FTM.CH1, ADMA.FLEXCAN1.TX, LSIO.GPIO1.IO23 */
#define SC_P_UART2_RX                            114  /*!< ADMA.UART2.RX, ADMA.FTM.CH0, ADMA.FLEXCAN1.RX, LSIO.GPIO1.IO24 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOLH        115  /*!<  */
#define SC_P_MIPI_DSI0_I2C0_SCL                  116  /*!< MIPI_DSI0.I2C0.SCL, MIPI_DSI1.GPIO0.IO02, LSIO.GPIO1.IO25 */
#define SC_P_MIPI_DSI0_I2C0_SDA                  117  /*!< MIPI_DSI0.I2C0.SDA, MIPI_DSI1.GPIO0.IO03, LSIO.GPIO1.IO26 */
#define SC_P_MIPI_DSI0_GPIO0_00                  118  /*!< MIPI_DSI0.GPIO0.IO00, ADMA.I2C1.SCL, MIPI_DSI0.PWM0.OUT, LSIO.GPIO1.IO27 */
#define SC_P_MIPI_DSI0_GPIO0_01                  119  /*!< MIPI_DSI0.GPIO0.IO01, ADMA.I2C1.SDA, LSIO.GPIO1.IO28 */
#define SC_P_MIPI_DSI1_I2C0_SCL                  120  /*!< MIPI_DSI1.I2C0.SCL, MIPI_DSI0.GPIO0.IO02, LSIO.GPIO1.IO29 */
#define SC_P_MIPI_DSI1_I2C0_SDA                  121  /*!< MIPI_DSI1.I2C0.SDA, MIPI_DSI0.GPIO0.IO03, LSIO.GPIO1.IO30 */
#define SC_P_MIPI_DSI1_GPIO0_00                  122  /*!< MIPI_DSI1.GPIO0.IO00, ADMA.I2C2.SCL, MIPI_DSI1.PWM0.OUT, LSIO.GPIO1.IO31 */
#define SC_P_MIPI_DSI1_GPIO0_01                  123  /*!< MIPI_DSI1.GPIO0.IO01, ADMA.I2C2.SDA, LSIO.GPIO2.IO00 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_MIPIDSIGPIO   124  /*!<  */
#define SC_P_JTAG_TRST_B                         125  /*!< SCU.JTAG.TRST_B, SCU.WDOG0.WDOG_OUT */
#define SC_P_PMIC_I2C_SCL                        126  /*!< SCU.PMIC_I2C.SCL, SCU.GPIO0.IOXX_PMIC_A35_ON, LSIO.GPIO2.IO01 */
#define SC_P_PMIC_I2C_SDA                        127  /*!< SCU.PMIC_I2C.SDA, SCU.GPIO0.IOXX_PMIC_GPU_ON, LSIO.GPIO2.IO02 */
#define SC_P_PMIC_INT_B                          128  /*!< SCU.DSC.PMIC_INT_B */
#define SC_P_SCU_GPIO0_00                        129  /*!< SCU.GPIO0.IO00, SCU.UART0.RX, M40.UART0.RX, ADMA.UART3.RX, LSIO.GPIO2.IO03 */
#define SC_P_SCU_GPIO0_01                        130  /*!< SCU.GPIO0.IO01, SCU.UART0.TX, M40.UART0.TX, ADMA.UART3.TX, SCU.WDOG0.WDOG_OUT */
#define SC_P_SCU_PMIC_STANDBY                    131  /*!< SCU.DSC.PMIC_STANDBY */
#define SC_P_SCU_BOOT_MODE0                      132  /*!< SCU.DSC.BOOT_MODE0 */
#define SC_P_SCU_BOOT_MODE1                      133  /*!< SCU.DSC.BOOT_MODE1 */
#define SC_P_SCU_BOOT_MODE2                      134  /*!< SCU.DSC.BOOT_MODE2, SCU.PMIC_I2C.SDA */
#define SC_P_SCU_BOOT_MODE3                      135  /*!< SCU.DSC.BOOT_MODE3, SCU.PMIC_I2C.SCL, SCU.DSC.RTC_CLOCK_OUTPUT_32K */
#define SC_P_CSI_D00                             136  /*!< CI_PI.D02, ADMA.SAI0.RXC */
#define SC_P_CSI_D01                             137  /*!< CI_PI.D03, ADMA.SAI0.RXD */
#define SC_P_CSI_D02                             138  /*!< CI_PI.D04, ADMA.SAI0.RXFS */
#define SC_P_CSI_D03                             139  /*!< CI_PI.D05, ADMA.SAI2.RXC */
#define SC_P_CSI_D04                             140  /*!< CI_PI.D06, ADMA.SAI2.RXD */
#define SC_P_CSI_D05                             141  /*!< CI_PI.D07, ADMA.SAI2.RXFS */
#define SC_P_CSI_D06                             142  /*!< CI_PI.D08, ADMA.SAI3.RXC */
#define SC_P_CSI_D07                             143  /*!< CI_PI.D09, ADMA.SAI3.RXD */
#define SC_P_CSI_HSYNC                           144  /*!< CI_PI.HSYNC, CI_PI.D00, ADMA.SAI3.RXFS */
#define SC_P_CSI_VSYNC                           145  /*!< CI_PI.VSYNC, CI_PI.D01 */
#define SC_P_CSI_PCLK                            146  /*!< CI_PI.PCLK, MIPI_CSI0.I2C0.SCL, ADMA.SPI1.SCK, LSIO.GPIO3.IO00 */
#define SC_P_CSI_MCLK                            147  /*!< CI_PI.MCLK, MIPI_CSI0.I2C0.SDA, ADMA.SPI1.SDO, LSIO.GPIO3.IO01 */
#define SC_P_CSI_EN                              148  /*!< CI_PI.EN, CI_PI.I2C.SCL, ADMA.I2C3.SCL, ADMA.SPI1.SDI, LSIO.GPIO3.IO02 */
#define SC_P_CSI_RESET                           149  /*!< CI_PI.RESET, CI_PI.I2C.SDA, ADMA.I2C3.SDA, ADMA.SPI1.CS0, LSIO.GPIO3.IO03 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHD       150  /*!<  */
#define SC_P_MIPI_CSI0_MCLK_OUT                  151  /*!< MIPI_CSI0.ACM.MCLK_OUT, LSIO.GPIO3.IO04 */
#define SC_P_MIPI_CSI0_I2C0_SCL                  152  /*!< MIPI_CSI0.I2C0.SCL, MIPI_CSI0.GPIO0.IO02, LSIO.GPIO3.IO05 */
#define SC_P_MIPI_CSI0_I2C0_SDA                  153  /*!< MIPI_CSI0.I2C0.SDA, MIPI_CSI0.GPIO0.IO03, LSIO.GPIO3.IO06 */
#define SC_P_MIPI_CSI0_GPIO0_01                  154  /*!< MIPI_CSI0.GPIO0.IO01, ADMA.I2C0.SDA, LSIO.GPIO3.IO07 */
#define SC_P_MIPI_CSI0_GPIO0_00                  155  /*!< MIPI_CSI0.GPIO0.IO00, ADMA.I2C0.SCL, LSIO.GPIO3.IO08 */
#define SC_P_QSPI0A_DATA0                        156  /*!< LSIO.QSPI0A.DATA0, LSIO.GPIO3.IO09 */
#define SC_P_QSPI0A_DATA1                        157  /*!< LSIO.QSPI0A.DATA1, LSIO.GPIO3.IO10 */
#define SC_P_QSPI0A_DATA2                        158  /*!< LSIO.QSPI0A.DATA2, LSIO.GPIO3.IO11 */
#define SC_P_QSPI0A_DATA3                        159  /*!< LSIO.QSPI0A.DATA3, LSIO.GPIO3.IO12 */
#define SC_P_QSPI0A_DQS                          160  /*!< LSIO.QSPI0A.DQS, LSIO.GPIO3.IO13 */
#define SC_P_QSPI0A_SS0_B                        161  /*!< LSIO.QSPI0A.SS0_B, LSIO.GPIO3.IO14 */
#define SC_P_QSPI0A_SS1_B                        162  /*!< LSIO.QSPI0A.SS1_B, LSIO.GPIO3.IO15 */
#define SC_P_QSPI0A_SCLK                         163  /*!< LSIO.QSPI0A.SCLK, LSIO.GPIO3.IO16 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_QSPI0A        164  /*!<  */
#define SC_P_QSPI0B_SCLK                         165  /*!< LSIO.QSPI0B.SCLK, LSIO.QSPI1A.SCLK, LSIO.KPP0.COL0, LSIO.GPIO3.IO17 */
#define SC_P_QSPI0B_DATA0                        166  /*!< LSIO.QSPI0B.DATA0, LSIO.QSPI1A.DATA0, LSIO.KPP0.COL1, LSIO.GPIO3.IO18 */
#define SC_P_QSPI0B_DATA1                        167  /*!< LSIO.QSPI0B.DATA1, LSIO.QSPI1A.DATA1, LSIO.KPP0.COL2, LSIO.GPIO3.IO19 */
#define SC_P_QSPI0B_DATA2                        168  /*!< LSIO.QSPI0B.DATA2, LSIO.QSPI1A.DATA2, LSIO.KPP0.COL3, LSIO.GPIO3.IO20 */
#define SC_P_QSPI0B_DATA3                        169  /*!< LSIO.QSPI0B.DATA3, LSIO.QSPI1A.DATA3, LSIO.KPP0.ROW0, LSIO.GPIO3.IO21 */
#define SC_P_QSPI0B_DQS                          170  /*!< LSIO.QSPI0B.DQS, LSIO.QSPI1A.DQS, LSIO.KPP0.ROW1, LSIO.GPIO3.IO22 */
#define SC_P_QSPI0B_SS0_B                        171  /*!< LSIO.QSPI0B.SS0_B, LSIO.QSPI1A.SS0_B, LSIO.KPP0.ROW2, LSIO.GPIO3.IO23 */
#define SC_P_QSPI0B_SS1_B                        172  /*!< LSIO.QSPI0B.SS1_B, LSIO.QSPI1A.SS1_B, LSIO.KPP0.ROW3, LSIO.GPIO3.IO24 */
#define SC_P_COMP_CTL_GPIO_1V8_3V3_QSPI0B        173  /*!<  */

/* mux values */
#define IMX8QXP_CM4_UART_RX_CM4_UART_RX 1 
#define IMX8QXP_CM4_UART_TX_CM4_UART_TX 1

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IMX8QXP_PINCTRL_H_ */
