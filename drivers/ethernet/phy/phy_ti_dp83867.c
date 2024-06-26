/*
 * Copyright 2023 NXP
 *
 * Inspiration from phy_mii.c, which is:
 * Copyright (c) 2021 IP-Logix Inc.
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_dp83867

#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <string.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/ethernet/phy/ti-dp83867.h>

#define LOG_MODULE_NAME phy_ti_dp83867
#define LOG_LEVEL CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* TI DP83867 */
#define DP83867_DEVADDR		0x1f

#define MII_DP83867_PHYCTRL	0x10
#define MII_DP83867_MICR	0x12
#define MII_DP83867_CFG2	0x14
#define MII_DP83867_BISCR	0x16
#define DP83867_LEDCR2		0x19
#define DP83867_CTRL		0x1f

/* Extended Registers */
#define DP83867_CFG4		0x0031
#define DP83867_RGMIICTL	0x0032
#define DP83867_STRAP_STS1	0x006E
#define DP83867_STRAP_STS2	0x006f
#define DP83867_RGMIIDCTL	0x0086
#define DP83867_IO_MUX_CFG	0x0170
#define DP83867_SGMIICTL	0x00D3

#define DP83867_SW_RESET	BIT(15)
#define DP83867_SW_RESTART	BIT(14)

/* Offset to align capabilities bits of 1000BASE-T Control and Status regs */
#define MII_1KSTSR_OFFSET 2

/* MMD_CTRL bits */
#define MII_DP83867_MMD_CTRL_NOINCR		BIT(14)

/* MICR Interrupt bits */
#define MII_DP83867_MICR_AN_ERR_INT_EN		BIT(15)
#define MII_DP83867_MICR_SPEED_CHNG_INT_EN	BIT(14)
#define MII_DP83867_MICR_DUP_MODE_CHNG_INT_EN	BIT(13)
#define MII_DP83867_MICR_PAGE_RXD_INT_EN	BIT(12)
#define MII_DP83867_MICR_AUTONEG_COMP_INT_EN	BIT(11)
#define MII_DP83867_MICR_LINK_STS_CHNG_INT_EN	BIT(10)
#define MII_DP83867_MICR_FALSE_CARRIER_INT_EN	BIT(8)
#define MII_DP83867_MICR_SLEEP_MODE_CHNG_INT_EN	BIT(4)
#define MII_DP83867_MICR_WOL_INT_EN		BIT(3)
#define MII_DP83867_MICR_XGMII_ERR_INT_EN	BIT(2)
#define MII_DP83867_MICR_POL_CHNG_INT_EN	BIT(1)
#define MII_DP83867_MICR_JABBER_INT_EN		BIT(0)

/* RGMIICTL bits */
#define DP83867_RGMII_TX_CLK_DELAY_EN		BIT(1)
#define DP83867_RGMII_RX_CLK_DELAY_EN		BIT(0)

/* STRAP_STS1 bits */
#define DP83867_STRAP_STS1_RESERVED		BIT(11)

/* STRAP_STS2 bits */
#define DP83867_STRAP_STS2_CLK_SKEW_TX_MASK	GENMASK(6, 4)
#define DP83867_STRAP_STS2_CLK_SKEW_TX_SHIFT	4
#define DP83867_STRAP_STS2_CLK_SKEW_RX_MASK	GENMASK(2, 0)
#define DP83867_STRAP_STS2_CLK_SKEW_RX_SHIFT	0
#define DP83867_STRAP_STS2_CLK_SKEW_NONE	BIT(2)

/* PHY CTRL bits */
#define DP83867_PHYCR_FIFO_DEPTH_SHIFT		14
#define DP83867_PHYCR_FIFO_DEPTH_MASK		GENMASK(15, 14)
#define DP83867_PHYCR_RESERVED_MASK	BIT(11)
#define DP83867_PHYCR_FORCE_LINK_GOOD	BIT(10)
#define DP83867_MDI_CROSSOVER		5
#define DP83867_MDI_CROSSOVER_MDIX	2
#define DP83867_PHYCTRL_SGMIIEN			0x0800
#define DP83867_PHYCTRL_RXFIFO_SHIFT	12
#define DP83867_PHYCTRL_TXFIFO_SHIFT	14

/* RGMIIDCTL bits */
#define DP83867_RGMII_TX_CLK_DELAY_MAX		0xf
#define DP83867_RGMII_TX_CLK_DELAY_SHIFT	4
#define DP83867_RGMII_RX_CLK_DELAY_MAX		0xf

/* CFG2 bits */
#define MII_DP83867_CFG2_SPEEDOPT_10EN		0x0040
#define MII_DP83867_CFG2_SGMII_AUTONEGEN	0x0080
#define MII_DP83867_CFG2_SPEEDOPT_ENH		0x0100
#define MII_DP83867_CFG2_SPEEDOPT_CNT		0x0800
#define MII_DP83867_CFG2_SPEEDOPT_INTLOW	0x2000
#define MII_DP83867_CFG2_MASK			0x003F

/* User setting - can be taken from DTS */
#define DEFAULT_FIFO_DEPTH	DP83867_PHYCR_FIFO_DEPTH_4_B_NIB

/* IO_MUX_CFG bits */
#define DP83867_IO_MUX_CFG_IO_IMPEDANCE_CTRL	0x1f

#define DP83867_IO_MUX_CFG_IO_IMPEDANCE_MAX	0x0
#define DP83867_IO_MUX_CFG_IO_IMPEDANCE_MIN	0x1f
#define DP83867_IO_MUX_CFG_CLK_O_DISABLE	BIT(6)
#define DP83867_IO_MUX_CFG_CLK_O_SEL_SHIFT	8
#define DP83867_IO_MUX_CFG_CLK_O_SEL_MASK	\
		GENMASK(0x1f, DP83867_IO_MUX_CFG_CLK_O_SEL_SHIFT)

/* LEDCR2 bits */
#define DP83867_LEDCR2_LED_0_POLARITY		BIT(2)
#define DP83867_LEDCR2_LED_1_POLARITY		BIT(6)
#define DP83867_LEDCR2_LED_2_POLARITY		BIT(10)

/* CFG3 bits */
#define DP83867_CFG3_ROBUST_AUTO_MDIX   BIT(9)

/* CFG4 bits */
#define DP83867_CFG4_PORT_MIRROR_EN		BIT(0)

/* SGMIICTL bits */
#define DP83867_SGMII_TYPE			BIT(14)

enum dp83867_port_mirroring {
	DP83867_PORT_MIRRORING_KEEP,
	DP83867_PORT_MIRRORING_EN,
	DP83867_PORT_MIRRORING_DIS,
};

enum dp83867_interface {
	DP83867_MII,
	DP83867_RMII,
	DP83867_RGMII,
	DP83867_RGMII_ID,
};

struct dp83867_config {
	uint8_t addr;
	const struct device *mdio_dev;
	enum dp83867_interface phy_iface;
	bool no_reset;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(ti_reset_gpio)
	const struct gpio_dt_spec reset_gpio;
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(ti_interrupt_gpio)
	const struct gpio_dt_spec interrupt_gpio;
#endif
	uint32_t fifo_depth;
	uint32_t rx_id_delay;
	uint32_t tx_id_delay;
	int io_impedance;
	int port_mirroring;
	bool set_clk_output;
	uint32_t clk_output_sel;
	bool rxctrl_strap_quirk;
	bool led_0_active_low;
	bool led_1_active_low;
	bool led_2_active_low;
};

struct dp83867_data {
	const struct device *dev;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	struct k_mutex mutex;
	struct k_work_delayable phy_monitor_work;
	bool gigabit_supported;
};

static int phy_dp83867_read(const struct device *dev,
				uint16_t reg_addr, uint32_t *data)
{
	const struct dp83867_config *config = dev->config;
	int ret;

	ret = mdio_read(config->mdio_dev, config->addr, reg_addr, (uint16_t *)data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_dp83867_write(const struct device *dev,
				uint16_t reg_addr, uint32_t data)
{
	const struct dp83867_config *config = dev->config;
	int ret;

	ret = mdio_write(config->mdio_dev, config->addr, reg_addr, (uint16_t)data);
	if (ret) {
		return ret;
	}

	return 0;
}
static inline int phy_dp83867_mmd_start_indirect(const struct device *dev, 
											uint8_t devad, uint16_t reg_addr)
{
	int ret;

	/* Write the desired MMD Devad */
	ret = phy_dp83867_write(dev, MII_MMD_ACR, devad);
	if (ret) {
		return ret;
	}

	/* Write the desired MMD register address */
	ret = phy_dp83867_write(dev, MII_MMD_AADR, reg_addr);
	if (ret) {
		return ret;
	}

	/* Select the Function : DATA with no post increment */
	ret = phy_dp83867_write(dev, MII_MMD_ACR, devad | MII_DP83867_MMD_CTRL_NOINCR);
	if (ret) {
		return ret;
	}

	return 0;
}
static int phy_dp83867_read_mmd(const struct device *dev, uint8_t devad,
				uint16_t reg_addr, uint32_t *data)
{
	int ret;

	ret = phy_dp83867_mmd_start_indirect(dev, devad, reg_addr);
	if (ret) {
		return ret;
	}

	ret = phy_dp83867_read(dev, MII_MMD_AADR, data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_dp83867_write_mmd(const struct device *dev, uint8_t devad,
				uint16_t reg_addr, uint32_t data)
{
	int ret;

	ret = phy_dp83867_mmd_start_indirect(dev, devad, reg_addr);
	if (ret) {
		return ret;
	}

	ret = phy_dp83867_write(dev, MII_MMD_AADR, data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_dp83867_reset(const struct device *dev)
{
	uint32_t timeout = 12U;
	uint32_t value;
	int ret;

	/* Issue a soft reset */
	ret = phy_dp83867_write(dev, MII_BMCR, MII_BMCR_RESET);
	if (ret) {
		return ret;
	}

	/* Wait up to 0.6s for the reset sequence to finish. According to
	 * IEEE 802.3, Section 2, Subsection 22.2.4.1.1 a PHY reset may take
	 * up to 0.5 s.
	 */
	do {
		if (timeout-- == 0U) {
			return -ETIMEDOUT;
		}

		k_sleep(K_MSEC(50));

		ret = phy_dp83867_read(dev, MII_BMCR, &value);
		if (ret) {
			return ret;
		}
	} while (value & MII_BMCR_RESET);

	return 0;
}

static int phy_dp83867_is_gigabit_supported(const struct device *dev, 
											  bool *is_supported)
{
	uint32_t bmsr;
	uint32_t estat;
	int ret;

	*is_supported = false;

	ret = phy_dp83867_read(dev, MII_BMSR, &bmsr);
	if (ret) {
		return ret;
	}

	if (bmsr & MII_BMSR_EXTEND_STATUS) {
		ret = phy_dp83867_read(dev, MII_ESTAT, &estat);
		if (ret) {
			return ret;
		}

		if (estat & (MII_ESTAT_1000BASE_T_HALF | MII_ESTAT_1000BASE_T_FULL)) {
			*is_supported = true;
		}
	}

	return 0;
}

static int phy_dp83867_autonegotiate(const struct device *dev)
{
	const struct dp83867_config *config = dev->config;
	int ret;
	uint32_t bmcr = 0, bmsr = 0;
	uint16_t timeout = CONFIG_PHY_AUTONEG_TIMEOUT_MS / 100;

	/* Read control register to write back with autonegotiation bit */
	ret = phy_dp83867_read(dev, MII_BMCR, &bmcr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) basic control register", config->addr);
		return ret;
	}

	/* (re)start autonegotiation */
	LOG_DBG("PHY (%d) is entering autonegotiation sequence", config->addr);
	bmcr |= MII_BMCR_AUTONEG_ENABLE | MII_BMCR_AUTONEG_RESTART;
	bmcr &= ~MII_BMCR_ISOLATE;

	ret = phy_dp83867_write(dev, MII_BMCR, bmcr);
	if (ret) {
		LOG_ERR("Error writing phy (%d) basic control register", config->addr);
		return ret;
	}

	do {
		if (timeout-- == 0) {
			LOG_DBG("PHY (%d) autonegotiation timed out", config->addr);
			return -ETIMEDOUT;
		}
		k_msleep(100);

		ret = phy_dp83867_read(dev, MII_BMSR, &bmsr);
		if (ret) {
			LOG_ERR("Error reading phy (%d) basic status register", config->addr);
			return ret;
		}
	} while (!(bmsr & MII_BMSR_AUTONEG_COMPLETE));

	LOG_DBG("PHY (%d) autonegotiation completed", config->addr);

	return 0;
}

static int phy_dp83867_get_link(const struct device *dev,
					struct phy_link_state *state)
{
	const struct dp83867_config *config = dev->config;
	struct dp83867_data *data = dev->data;
	int ret;
	uint32_t bmsr = 0;
	uint32_t anar = 0;
	uint32_t anlpar = 0;
	uint32_t c1kt = 0, s1kt = 0;
	struct phy_link_state old_state = data->state;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* Read link state */
	ret = phy_dp83867_read(dev, MII_BMSR, &bmsr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) basic status register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}
	
	state->is_up = bmsr & MII_BMSR_LINK_STATUS;

	if (!state->is_up) {
		k_mutex_unlock(&data->mutex);
		goto result;
	}

	/* Read currently configured advertising options */
	ret = phy_dp83867_read(dev, MII_ANAR, &anar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) advertising register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* Read link partner device capability */
	ret = phy_dp83867_read(dev, MII_ANLPAR, &anlpar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) link partner register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	if (data->gigabit_supported) {
		ret = phy_dp83867_read(dev, MII_1KTCR, &c1kt);
		if (ret) {
			LOG_ERR("Error reading phy (%d) 1000BASE-T Configuration Register", config->addr);
			k_mutex_unlock(&data->mutex);
			return ret;
		}

		ret = phy_dp83867_read(dev, MII_1KSTSR, &s1kt);
		if (ret) {
			LOG_ERR("Error reading phy (%d) 1000BASE-T Status Register", config->addr);
			k_mutex_unlock(&data->mutex);
			return ret;
		}
		s1kt = (uint16_t)(s1kt >> MII_1KSTSR_OFFSET);
	}

	/* Unlock mutex */
	k_mutex_unlock(&data->mutex);

	if (data->gigabit_supported &&
			((c1kt & s1kt) & MII_ADVERTISE_1000_FULL)) {
		state->speed = LINK_FULL_1000BASE_T;
	} else if (data->gigabit_supported &&
			((c1kt & s1kt) & MII_ADVERTISE_1000_HALF)) {
		state->speed = LINK_HALF_1000BASE_T;
	} else if ((anar & anlpar) & MII_ADVERTISE_100_FULL) {
		state->speed = LINK_FULL_100BASE_T;
	} else if ((anar & anlpar) & MII_ADVERTISE_100_HALF) {
		state->speed = LINK_HALF_100BASE_T;
	} else if ((anar & anlpar) & MII_ADVERTISE_10_FULL) {
		state->speed = LINK_FULL_10BASE_T;
	} else {
		ret = -EIO;
	}

result:
	if (memcmp(&old_state, state, sizeof(struct phy_link_state)) != 0) {
		LOG_DBG("PHY %d is %s", config->addr, state->is_up ? "up" : "down");
		if (state->is_up) {
			LOG_DBG("PHY (%d) Link speed %s Mb, %s duplex\n",
				config->addr,
				PHY_LINK_IS_SPEED_1000M(state->speed) ? "1000" :
				(PHY_LINK_IS_SPEED_100M(state->speed) ? "100" : "10"),
				PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");
		}
	}

	return 0;
}

/*
 * Configuration set statically (DT) that should never change
 * This function is needed in case the PHY is reset then the next call
 * to configure the phy will ensure this configuration will be redone
 */
int phy_dp83867_static_cfg(const struct device *dev)
{
	const struct dp83867_config *config = dev->config;
	uint32_t val = 0, delay = 0;
	int ret = 0, bs = 0;

	/* Restart the PHY.  */
	ret = phy_dp83867_read(dev, DP83867_CTRL, &val);
	if (ret) {
		return ret;
	}

	ret = phy_dp83867_write(dev, DP83867_CTRL, val | DP83867_SW_RESTART);
	if (ret) {
		return ret;
	}

	/* Mode 1 or 2 workaround */
	if (config->rxctrl_strap_quirk) {
		ret = phy_dp83867_read_mmd(dev, DP83867_DEVADDR, DP83867_CFG4, &val);
		if (ret) {
			return ret;
		}

		val &= BIT(7);
		ret = phy_dp83867_write_mmd(dev, DP83867_DEVADDR, DP83867_CFG4, val);
		if (ret) {
			return ret;
		}
	}

	if (config->phy_iface == DP83867_RGMII || config->phy_iface == DP83867_RGMII_ID) {

		ret = phy_dp83867_read(dev, MII_DP83867_PHYCTRL, &val);
		if (ret) {
			return ret;
		}

		val &= ~DP83867_PHYCR_FIFO_DEPTH_MASK;
		val |= (config->fifo_depth << DP83867_PHYCR_FIFO_DEPTH_SHIFT);

		/* Do not force link good */
		val &= ~DP83867_PHYCR_FORCE_LINK_GOOD;

		/* The code below checks if "port mirroring" N/A MODE4 has been
		 * enabled during power on bootstrap.
		 *
		 * Such N/A mode enabled by mistake can put PHY IC in some
		 * internal testing mode and disable RGMII transmission.
		 *
		 * In this particular case one needs to check STRAP_STS1
		 * register's bit 11 (marked as RESERVED).
		 */
		ret = phy_dp83867_read_mmd(dev, DP83867_DEVADDR, DP83867_STRAP_STS1, &bs);
		if (ret) {
			return ret;
		}
		if (bs & DP83867_STRAP_STS1_RESERVED) {
			val &= ~DP83867_PHYCR_RESERVED_MASK;
		}

		ret = phy_dp83867_write(dev, MII_DP83867_PHYCTRL, val);
		if (ret) {
			return ret;
		}

		if (config->phy_iface == DP83867_RGMII_ID) {

			ret = phy_dp83867_read_mmd(dev, DP83867_DEVADDR, DP83867_RGMIICTL, &val);
			if (ret) {
				return ret;
			}

			val |= (DP83867_RGMII_TX_CLK_DELAY_EN |
				DP83867_RGMII_RX_CLK_DELAY_EN);
			ret = phy_dp83867_write_mmd(dev, DP83867_DEVADDR, DP83867_RGMIICTL, val);
			if (ret) {
				return ret;
			}

			delay = (config->rx_id_delay | \
					(config->tx_id_delay << DP83867_RGMII_TX_CLK_DELAY_SHIFT));
			ret = phy_dp83867_write_mmd(dev, DP83867_DEVADDR, DP83867_RGMIIDCTL, delay);
			if (ret) {
				return ret;
			}
		}
	}

	if (config->io_impedance >= 0 ) {
		ret = phy_dp83867_read_mmd(dev, DP83867_DEVADDR, DP83867_IO_MUX_CFG, &val);
		if (ret) {
			return ret;
		}

		val &= ~DP83867_IO_MUX_CFG_IO_IMPEDANCE_CTRL;
		val |= config->io_impedance &
				DP83867_IO_MUX_CFG_IO_IMPEDANCE_CTRL;

		ret = phy_dp83867_write_mmd(dev, DP83867_DEVADDR, DP83867_IO_MUX_CFG, val);
		if (ret) {
			return ret;
		}
	}

	if (config->port_mirroring != DP83867_PORT_MIRRORING_KEEP)	{
		ret = phy_dp83867_read_mmd(dev, DP83867_DEVADDR, DP83867_CFG4, &val);
		if (ret) {
			return ret;
		}

		if (config->port_mirroring == DP83867_PORT_MIRRORING_EN) {
			val |= DP83867_CFG4_PORT_MIRROR_EN;
		}
		else {
			val &= ~DP83867_CFG4_PORT_MIRROR_EN;
		}

		ret = phy_dp83867_write_mmd(dev, DP83867_DEVADDR, DP83867_CFG4, val);
		if (ret) {
			return ret;
		}
	}

	/* Clock output selection if muxing property is set */
	if (config->set_clk_output) {
		ret = phy_dp83867_read_mmd(dev, DP83867_DEVADDR, DP83867_IO_MUX_CFG, &val);
		if (ret) {
			return ret;
		}

		if (config->clk_output_sel == DP83867_CLK_O_SEL_OFF) {
			val |= DP83867_IO_MUX_CFG_CLK_O_DISABLE;
		} else {
			val &= ~(DP83867_IO_MUX_CFG_CLK_O_SEL_MASK | \
				 DP83867_IO_MUX_CFG_CLK_O_DISABLE);
			val |= config->clk_output_sel << \
			       DP83867_IO_MUX_CFG_CLK_O_SEL_SHIFT;
		}

		ret = phy_dp83867_write_mmd(dev, DP83867_DEVADDR, DP83867_IO_MUX_CFG, val);
		if (ret) {
			return ret;
		}
	}

	/* LED configuration */
	if (config->led_0_active_low) {
		ret = phy_dp83867_read(dev, DP83867_LEDCR2, &val);
		if (ret) {
			return ret;
		}

		val &= ~DP83867_LEDCR2_LED_0_POLARITY;

		ret = phy_dp83867_write(dev, DP83867_LEDCR2, val);
		if (ret) {
			return ret;
		}
	}

	if (config->led_1_active_low) {
		ret = phy_dp83867_read(dev, DP83867_LEDCR2, &val);
		if (ret) {
			return ret;
		}

		val &= ~DP83867_LEDCR2_LED_1_POLARITY;

		ret = phy_dp83867_write(dev, DP83867_LEDCR2, val);
		if (ret) {
			return ret;
		}
	}

	if (config->led_2_active_low) {
		ret = phy_dp83867_read(dev, DP83867_LEDCR2, &val);
		if (ret) {
			return ret;
		}

		val &= ~DP83867_LEDCR2_LED_2_POLARITY;

		ret = phy_dp83867_write(dev, DP83867_LEDCR2, val);
		if (ret) {
			return ret;
		}
	}
	return 0;
}

static int phy_dp83867_cfg_link(const struct device *dev,
					enum phy_link_speed adv_speeds)
{
	const struct dp83867_config *config = dev->config;
	struct dp83867_data *data = dev->data;
	struct phy_link_state state = {};
	int ret;
	uint32_t anar = 0, cfg1000 = 0;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		goto done;
	}

	/* We are going to reconfigure the phy, don't need to monitor until done */
	k_work_cancel_delayable(&data->phy_monitor_work);

	/* Reset PHY */
	if (config->no_reset == false) {
		ret = phy_dp83867_reset(dev);
		if (ret) {
			goto done;
		}
	}

	/* DT configurations */
	ret = phy_dp83867_static_cfg(dev);
	if (ret) {
		goto done;
	}

	/* Read ANAR register to write back */
	ret = phy_dp83867_read(dev, MII_ANAR, &anar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) advertising register", config->addr);
		goto done;
	}

	/* Setup advertising register */
	if (adv_speeds & LINK_FULL_10BASE_T) {
		anar |= MII_ADVERTISE_10_FULL;
	} else {
		anar &= ~MII_ADVERTISE_10_FULL;
	}

	if (adv_speeds & LINK_HALF_10BASE_T) {
		anar |= MII_ADVERTISE_10_HALF;
	} else {
		anar &= ~MII_ADVERTISE_10_HALF;
	}

	if (adv_speeds & LINK_FULL_100BASE_T) {
		anar |= MII_ADVERTISE_100_FULL;
	} else {
		anar &= ~MII_ADVERTISE_100_FULL;
	}

	if (adv_speeds & LINK_HALF_100BASE_T) {
		anar |= MII_ADVERTISE_100_HALF;
	} else {
		anar &= ~MII_ADVERTISE_100_HALF;
	}

	/* Write capabilities to advertising register */
	ret = phy_dp83867_write(dev, MII_ANAR, anar);
	if (ret) {
		LOG_ERR("Error writing phy (%d) advertising register", config->addr);
		goto done;
	}

	/* Configure gigabit if it's supported */
	if (data->gigabit_supported) {
		ret = phy_dp83867_read(dev, MII_1KTCR, &cfg1000);
		if (ret) {
			LOG_ERR("Error reading phy (%d) advertising register", config->addr);
			goto done;
		}
		if (adv_speeds & LINK_FULL_1000BASE_T) {
			cfg1000 |= MII_ADVERTISE_1000_FULL;
		} else {
			cfg1000 &= ~MII_ADVERTISE_1000_FULL;
		}
		if (adv_speeds & LINK_HALF_1000BASE_T) {
			cfg1000 |= MII_ADVERTISE_1000_HALF;
		} else {
			cfg1000 &= ~MII_ADVERTISE_1000_HALF;
		}

		ret = phy_dp83867_write(dev, MII_1KTCR, cfg1000);
		if (ret) {
			LOG_ERR("Error writing phy (%d) advertising register", config->addr);
			goto done;
		}
	} else {
		if (adv_speeds & LINK_FULL_1000BASE_T) {
			LOG_ERR("PHY doesn't support LINK_FULL_1000BASE_T");
		}
		if (adv_speeds & LINK_HALF_1000BASE_T) {
			LOG_ERR("PHY doesn't support LINK_HALF_1000BASE_T");
		}
	}

	/* (re)do autonegotiation */
	ret = phy_dp83867_autonegotiate(dev);
	if (ret && (ret != -ENETDOWN)) {
		LOG_ERR("Error in autonegotiation");
		goto done;
	}

	/* Get link status */
	ret = phy_dp83867_get_link(dev, &state);

	if (ret == 0 && memcmp(&state, &data->state, sizeof(struct phy_link_state)) != 0) {
		memcpy(&data->state, &state, sizeof(struct phy_link_state));
		if (data->cb) {
			data->cb(dev, &data->state, data->cb_data);
		}
	}

	LOG_DBG("PHY %d is %s", config->addr, data->state.is_up ? "up" : "down");
	if (data->state.is_up) {
		LOG_DBG("PHY (%d) Link speed %s Mb, %s duplex\n",
			config->addr,
			PHY_LINK_IS_SPEED_1000M(data->state.speed) ? "1000" :
			(PHY_LINK_IS_SPEED_100M(data->state.speed) ? "100" : "10"),
			PHY_LINK_IS_FULL_DUPLEX(data->state.speed) ? "full" : "half");
	}

done:
	/* Unlock mutex */
	k_mutex_unlock(&data->mutex);

	/* Start monitoring */
	k_work_reschedule(&data->phy_monitor_work,
				K_MSEC(CONFIG_PHY_MONITOR_PERIOD));

	return ret;
}

static int phy_dp83867_link_cb_set(const struct device *dev,
					phy_callback_t cb, void *user_data)
{
	struct dp83867_data *data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;

	phy_dp83867_get_link(dev, &data->state);

	data->cb(dev, &data->state, data->cb_data);

	return 0;
}

static void phy_dp83867_monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct dp83867_data *data =
		CONTAINER_OF(dwork, struct dp83867_data, phy_monitor_work);
	const struct device *dev = data->dev;
	struct phy_link_state state = {};
	int rc;

	rc = phy_dp83867_get_link(dev, &state);

	if (rc == 0 && memcmp(&state, &data->state, sizeof(struct phy_link_state)) != 0) {
		memcpy(&data->state, &state, sizeof(struct phy_link_state));
		if (data->cb) {
			data->cb(dev, &data->state, data->cb_data);
		}
	}

	/* TODO change this to GPIO interrupt driven */
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int phy_dp83867_init(const struct device *dev)
{
	const struct dp83867_config *config = dev->config;
	struct dp83867_data *data = dev->data;
	int ret;

	data->dev = dev;

	ret = k_mutex_init(&data->mutex);
	if (ret) {
		return ret;
	}

	mdio_bus_enable(config->mdio_dev);

	ret = phy_dp83867_is_gigabit_supported(dev, &(data->gigabit_supported));
	if (ret) {
		return ret;
	}


#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(interrupt_gpio)
	if (!config->interrupt_gpio.port) {
		goto skip_int_gpio;
	}

	/* Prevent NAND TREE mode */
	ret = gpio_pin_configure_dt(&config->interrupt_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}

skip_int_gpio:
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(interrupt_gpio) */


#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpio)
	if (!config->reset_gpio.port) {
		goto skip_reset_gpio;
	}

	/* Start reset */
	ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	/* Wait for 500 ms as specified by datasheet */
	k_busy_wait(USEC_PER_MSEC * 500);

	/* Reset over */
	ret = gpio_pin_set_dt(&config->reset_gpio, 1);
	if (ret) {
		return ret;
	}

skip_reset_gpio:
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpio) */

	/* Reset PHY */
	if (config->no_reset == false) {
		ret = phy_dp83867_reset(dev);
		if (ret) {
			return ret;
		}
	}

	k_work_init_delayable(&data->phy_monitor_work,
				phy_dp83867_monitor_work_handler);

	return 0;
}

static const struct ethphy_driver_api dp83867_phy_api = {
	.get_link = phy_dp83867_get_link,
	.cfg_link = phy_dp83867_cfg_link,
	.link_cb_set = phy_dp83867_link_cb_set,
	.read = phy_dp83867_read,
	.write = phy_dp83867_write,
};

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(ti_reset_gpio)
#define RESET_GPIO(n) \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, ti_reset_gpio, {0}),
#else
#define RESET_GPIO(n)
#endif /* reset gpio */

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(ti_interrupt_gpio)
#define INTERRUPT_GPIO(n) \
		.interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(n, ti_interrupt_gpio, {0}),
#else
#define INTERRUPT_GPIO(n)
#endif /* interrupt gpio */

#define IO_IMPENDACE(n) \
		(0 != DT_INST_PROP(n, ti_max_output_impedance)) ? DP83867_IO_MUX_CFG_IO_IMPEDANCE_MAX : \
		  ((0 != DT_INST_PROP(n, ti_min_output_impedance)) ? DP83867_IO_MUX_CFG_IO_IMPEDANCE_MIN : -EINVAL)

#define PORT_MIRRORING(n) \
		(0 != DT_INST_PROP(n, ti_enet_phy_lane_swap)) ? DP83867_PORT_MIRRORING_EN : \
		  ((0 != DT_INST_PROP(n, ti_enet_phy_lane_no_swap)) ? DP83867_PORT_MIRRORING_DIS : DP83867_PORT_MIRRORING_KEEP)

#define CLK_OUTPUT_SEL(n) \
		DT_INST_NODE_HAS_PROP(n, ti_clk_output_sel) ? true : false

#define TI_DP83867_INIT(n)						\
	static const struct dp83867_config dp83867_##n##_config = {	\
		.addr = DT_INST_REG_ADDR(n),					\
		.mdio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),			\
		.phy_iface = DT_INST_ENUM_IDX(n, ti_interface_type),		\
		.no_reset = DT_INST_PROP(n, no_reset), \
		.rx_id_delay = DT_INST_PROP_OR(n, ti_rx_internal_delay, DP83867_RGMIIDCTL_2_25_NS), \
		.tx_id_delay = DT_INST_PROP_OR(n, ti_tx_internal_delay, DP83867_RGMIIDCTL_2_75_NS), \
		RESET_GPIO(n)							\
		INTERRUPT_GPIO(n)						\
		.io_impedance = IO_IMPENDACE(n), 	    \
		.set_clk_output = CLK_OUTPUT_SEL(n), 						\
		.clk_output_sel = DT_INST_PROP_OR(n, ti_clk_output_sel, DP83867_CLK_O_SEL_OFF), \
		.fifo_depth = DT_INST_PROP_OR(n, ti_fifo_depth, DEFAULT_FIFO_DEPTH), \
		.rxctrl_strap_quirk = DT_INST_PROP(n, ti_dp83867_rxctrl_strap_quirk), \
		.port_mirroring = PORT_MIRRORING(n), \
		.led_0_active_low = DT_INST_PROP(n, ti_led0_active_low), \
		.led_1_active_low = DT_INST_PROP(n, ti_led1_active_low), \
		.led_2_active_low = DT_INST_PROP(n, ti_led2_active_low), \
	};									\
										\
	static struct dp83867_data dp83867_##n##_data;			\
										\
	DEVICE_DT_INST_DEFINE(n, &phy_dp83867_init, NULL,			\
			&dp83867_##n##_data, &dp83867_##n##_config,	\
			POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,			\
			&dp83867_phy_api);

DT_INST_FOREACH_STATUS_OKAY(TI_DP83867_INIT)
