/*
 * Copyright 2023 NXP
 *
 * Inspiration from phy_mii.c, which is:
 * Copyright (c) 2021 IP-Logix Inc.
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT qca_ar803x

#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/net/mdio.h>
#include <zephyr/drivers/mdio.h>
#include <string.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>


#define LOG_MODULE_NAME phy_qca_ar803x
#define LOG_LEVEL CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define PHY_SPECIFIC_STATUS_REG    0x11U /*!< The PHY specific status register. */
/* Defines the mask flag in specific status register. */
#define PHY_SSTATUS_LINKSTATUS_MASK 0x0400U /*!< The PHY link status mask. */
#define PHY_SSTATUS_LINKSPEED_MASK  0xC000U /*!< The PHY link speed mask. */
#define PHY_SSTATUS_LINKDUPLEX_MASK 0x2000U /*!< The PHY link duplex mask. */
#define PHY_SSTATUS_LINKSPEED_SHIFT 14U     /*!< The link speed shift */


#define AR803x_PHY_DEBUG_ADDR_REG	0x1d
#define AR803x_PHY_DEBUG_DATA_REG	0x1e

/* Debug registers */
#define AR803x_DEBUG_REG_0		    0x0
#define AR803x_RGMII_RX_CLK_DLY		BIT(15)

#define AR803x_DEBUG_REG_5		    0x5
#define AR803x_RGMII_TX_CLK_DLY		BIT(8)

#define AR803x_DEBUG_REG_1F		    0x1f
#define AR803x_PLL_ON			    BIT(2)
#define AR803x_RGMII_1V8		    BIT(3)

/* CLK_25M register is at MMD 7, address 0x8016 */
#define AR803x_CLK_25M_SEL_REG		0x8016

#define AR803x_CLK_25M_25MHZ_XTAL	(0UL << (2))
#define AR803x_CLK_25M_25MHZ_DSP	(1UL << (2))
#define AR803x_CLK_25M_50MHZ_PLL	(2UL << (2))
#define AR803x_CLK_25M_50MHZ_DSP	(3UL << (2))
#define AR803x_CLK_25M_62_5MHZ_PLL	(4UL << (2))
#define AR803x_CLK_25M_62_5MHZ_DSP	(5UL << (2))
#define AR803x_CLK_25M_125MHZ_PLL	(6UL << (2))
#define AR803x_CLK_25M_125MHZ_DSP	(7UL << (2))
#define AR803x_CLK_25M_INVALID	    (0xFFFF)


#define AR803x_CLK_25M_DR_FULL		(0UL << (7))
#define AR803x_CLK_25M_DR_HALF		(1UL << (7))
#define AR803x_CLK_25M_DR_QUARTER	(2UL << (7))
#define AR803x_CLK_25M_DR_INVALID	(0xFFFF)

#define ATH8031_PHY_ID				0x004dd074

/* MMD_CTRL bits */
#define MII_AR803x_MMD_CTRL_NOINCR		    BIT(14)

#define AT803X_MMD3_SMARTEEE_CTL1		    0x805b
#define AT803X_MMD3_SMARTEEE_CTL2		    0x805c
#define AT803X_MMD3_SMARTEEE_CTL3		    0x805d
#define AT803X_MMD3_SMARTEEE_CTL3_LPI_EN	BIT(8)

enum ar803x_interface {
	AR803X_MII,
	AR803X_RMII,
	AR803X_RGMII,
	AR803X_RGMII_ID,
	AR803X_RGMII_RXID,
	AR803X_RGMII_TXID,
};

struct ar803x_config {
	uint8_t addr;
	const struct device *mdio_dev;
	enum ar803x_interface phy_iface;
	bool no_reset;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	const struct gpio_dt_spec reset_gpio;
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	const struct gpio_dt_spec interrupt_gpio;
#endif
	bool enableEEE;
	bool keepPllEnabled;
	uint32_t min_uV;
	uint32_t max_uV;
	uint32_t clock_out_frequency;
	uint32_t clock_out_strength;
};

struct ar803x_data {
	const struct device *dev;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	struct k_mutex mutex;
	struct k_work_delayable phy_monitor_work;
	uint32_t chip_id;
};

static int phy_ar803x_read(const struct device *dev,
				uint16_t reg_addr, uint32_t *data)
{
	const struct ar803x_config *config = dev->config;
	int ret;

	ret = mdio_read(config->mdio_dev, config->addr, reg_addr, (uint16_t *)data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_ar803x_write(const struct device *dev,
				uint16_t reg_addr, uint32_t data)
{
	const struct ar803x_config *config = dev->config;
	int ret;

	ret = mdio_write(config->mdio_dev, config->addr, reg_addr, (uint16_t)data);
	if (ret) {
		return ret;
	}

	return 0;
}
static inline int phy_ar803x_mmd_start_indirect(const struct device *dev, 
											uint8_t devad, uint16_t reg_addr)
{
	int ret;

	/* Write the desired MMD Devad */
	ret = phy_ar803x_write(dev, MII_MMD_ACR, devad);
	if (ret) {
		return ret;
	}

	/* Write the desired MMD register address */
	ret = phy_ar803x_write(dev, MII_MMD_AADR, reg_addr);
	if (ret) {
		return ret;
	}

	/* Select the Function : DATA with no post increment */
	ret = phy_ar803x_write(dev, MII_MMD_ACR, devad | MII_AR803x_MMD_CTRL_NOINCR);
	if (ret) {
		return ret;
	}

	return 0;
}
static int phy_ar803x_read_mmd(const struct device *dev, uint8_t devad,
				uint16_t reg_addr, uint32_t *data)
{
	int ret;

	ret = phy_ar803x_mmd_start_indirect(dev, devad, reg_addr);
	if (ret) {
		return ret;
	}

	ret = phy_ar803x_read(dev, MII_MMD_AADR, data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_ar803x_write_mmd(const struct device *dev, uint8_t devad,
				uint16_t reg_addr, uint32_t data)
{
	int ret;

	ret = phy_ar803x_mmd_start_indirect(dev, devad, reg_addr);
	if (ret) {
		return ret;
	}

	ret = phy_ar803x_write(dev, MII_MMD_AADR, data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_ar803x_reset(const struct device *dev)
{
	uint32_t timeout = 12U;
	uint32_t value;
	int ret;

	/* Issue a soft reset */
	ret = phy_ar803x_write(dev, MII_BMCR, MII_BMCR_RESET);
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

		ret = phy_ar803x_read(dev, MII_BMCR, &value);
		if (ret) {
			return ret;
		}
	} while (value & MII_BMCR_RESET);

	return 0;
}

static int phy_ar803x_autonegotiate(const struct device *dev)
{
	const struct ar803x_config *config = dev->config;
	int ret;
	uint32_t bmcr = 0, bmsr = 0;
	uint16_t timeout = CONFIG_PHY_AUTONEG_TIMEOUT_MS / 100;

	/* Read control register to write back with autonegotiation bit */
	ret = phy_ar803x_read(dev, MII_BMCR, &bmcr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) basic control register", config->addr);
		return ret;
	}

	/* (re)start autonegotiation */
	LOG_DBG("PHY (%d) is entering autonegotiation sequence", config->addr);
	bmcr |= MII_BMCR_AUTONEG_ENABLE | MII_BMCR_AUTONEG_RESTART;
	bmcr &= ~MII_BMCR_ISOLATE;

	ret = phy_ar803x_write(dev, MII_BMCR, bmcr);
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

		ret = phy_ar803x_read(dev, MII_BMSR, &bmsr);
		if (ret) {
			LOG_ERR("Error reading phy (%d) basic status register", config->addr);
			return ret;
		}
	} while (!(bmsr & MII_BMSR_AUTONEG_COMPLETE));

	LOG_DBG("PHY (%d) autonegotiation completed", config->addr);

	return 0;
}

static int phy_ar803x_get_link(const struct device *dev,
					struct phy_link_state *state)
{
	const struct ar803x_config *config = dev->config;
	struct ar803x_data *data = dev->data;
	int ret;
	uint32_t bmsr = 0;
	uint32_t val = 0;
	uint32_t speed = 0, duplex = 0;
	struct phy_link_state old_state = data->state;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* Read link state */
	ret = phy_ar803x_read(dev, MII_BMSR, &bmsr);
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
	ret = phy_ar803x_read(dev, PHY_SPECIFIC_STATUS_REG, &val);
	if (ret) {
		LOG_ERR("Error reading phy (%d) specific status register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* Unlock mutex */
	k_mutex_unlock(&data->mutex);

	speed = (val & PHY_SSTATUS_LINKSPEED_MASK) >> PHY_SSTATUS_LINKSPEED_SHIFT;
	duplex = val & PHY_SSTATUS_LINKDUPLEX_MASK;

	switch (speed) {
		case 0:
		if (duplex) {
			state->speed = LINK_FULL_10BASE_T;
		} else {
			state->speed = LINK_HALF_10BASE_T;
		}
		break;
		case 1:
		if (duplex) {
			state->speed = LINK_FULL_100BASE_T;
		} else {
			state->speed = LINK_HALF_100BASE_T;
		}
		break;
		case 2:
		if (duplex) {
			state->speed = LINK_FULL_1000BASE_T;
		} else {
			state->speed = LINK_HALF_1000BASE_T;
		}
		break;
		default:
		ret = -EIO;
		break;
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

	return ret;
}

/*
 * Configuration set statically (DT) that should never change
 * This function is needed in case the PHY is reset then the next call
 * to configure the phy will ensure this configuration will be redone
 */
int phy_ar803x_static_cfg(const struct device *dev)
{
	const struct ar803x_config *config = dev->config;
	struct ar803x_data *data = dev->data;
	uint32_t val = 0;
	int ret = 0;

	/* Enable TX delay */
	ret = phy_ar803x_write(dev, AR803x_PHY_DEBUG_ADDR_REG, AR803x_DEBUG_REG_5);
	if (ret) {
		return ret;
	}

	ret = phy_ar803x_read(dev, AR803x_PHY_DEBUG_DATA_REG, &val);
	if (ret) {
		return ret;
	}
	if ((config->phy_iface == AR803X_RGMII_TXID) || \
		(config->phy_iface == AR803X_RGMII_ID)) {
		val |= AR803x_RGMII_TX_CLK_DLY;
	} else {
		val &= ~AR803x_RGMII_TX_CLK_DLY;
	}

	ret = phy_ar803x_write(dev, AR803x_PHY_DEBUG_DATA_REG, val);
	if (ret) {
		return ret;
	}

	/* Enable RX delay */
	ret = phy_ar803x_write(dev, AR803x_PHY_DEBUG_ADDR_REG, AR803x_DEBUG_REG_0);
	if (ret) {
		return ret;
	}

	ret = phy_ar803x_read(dev, AR803x_PHY_DEBUG_DATA_REG, &val);
	if (ret) {
		return ret;
	}
	if ((config->phy_iface == AR803X_RGMII_RXID) || \
		(config->phy_iface == AR803X_RGMII_ID)) {
		val |= AR803x_RGMII_RX_CLK_DLY;
	} else {
		val &= ~AR803x_RGMII_RX_CLK_DLY;
	}

	ret = phy_ar803x_write(dev, AR803x_PHY_DEBUG_DATA_REG, val);
	if (ret) {
		return ret;
	}

	/*
	 * Only supported on the AR8031, AR8035 has strappings for the PLL mode
	 * as well as the RGMII voltage.
	 */
	if (data->chip_id == ATH8031_PHY_ID) {
		ret = phy_ar803x_write(dev, AR803x_PHY_DEBUG_ADDR_REG, AR803x_DEBUG_REG_1F);
		if (ret) {
			return ret;
		}

		if (config->keepPllEnabled) {
			ret = phy_ar803x_read(dev, AR803x_PHY_DEBUG_DATA_REG, &val);
			if (ret) {
				return ret;
			}

			val |= AR803x_PLL_ON;

			ret = phy_ar803x_write(dev, AR803x_PHY_DEBUG_DATA_REG, val);
			if (ret) {
				return ret;
			}
		}

		if (config->min_uV == 1800000) {
			ret = phy_ar803x_read(dev, AR803x_PHY_DEBUG_DATA_REG, &val);
			if (ret) {
				return ret;
			}

			val |= AR803x_RGMII_1V8;

			ret = phy_ar803x_write(dev, AR803x_PHY_DEBUG_DATA_REG, val);
			if (ret) {
				return ret;
		
			}
		}
	}

	if (config->clock_out_frequency != AR803x_CLK_25M_INVALID) {
		ret = phy_ar803x_read_mmd(dev, MDIO_MMD_AN, AR803x_CLK_25M_SEL_REG, &val);
		if (ret) {
			return ret;
		}

		switch (config->clock_out_frequency) {
			case 25000000:
			val |= AR803x_CLK_25M_25MHZ_XTAL;
			break;
			case 50000000:
			val |= AR803x_CLK_25M_50MHZ_PLL;
			break;
			case 62500000:
			val |= AR803x_CLK_25M_62_5MHZ_PLL;
			break;
			case 125000000:
			val |= AR803x_CLK_25M_125MHZ_PLL;
			break;
			default:
			return -EIO;
		}

		ret = phy_ar803x_write_mmd(dev, MDIO_MMD_AN, AR803x_CLK_25M_SEL_REG, val);
		if (ret) {
			return ret;
		}
	}

	if (data->chip_id == ATH8031_PHY_ID) { 
		if (config->clock_out_strength != AR803x_CLK_25M_DR_INVALID) {
			ret = phy_ar803x_read_mmd(dev, MDIO_MMD_AN, AR803x_CLK_25M_SEL_REG, &val);
			if (ret) {
				return ret;
			}

			switch (config->clock_out_frequency) {
				case 0:
				val |= AR803x_CLK_25M_DR_FULL;
				break;
				case 1:
				val |= AR803x_CLK_25M_DR_HALF;
				break;
				case 2:
				val |= AR803x_CLK_25M_DR_QUARTER;
				break;
				default:
				return -EIO;
			}

			ret = phy_ar803x_write_mmd(dev, MDIO_MMD_AN, AR803x_CLK_25M_SEL_REG, val);
			if (ret) {
				return ret;
			}
		}
	}

	/* Disable SmartEEE */
	if (!config->enableEEE) {
		/* Close smartEEE. */
		ret = phy_ar803x_mmd_start_indirect(dev, MDIO_MMD_PCS, AT803X_MMD3_SMARTEEE_CTL3);
		if (ret) {
			return ret;
		}

		ret = phy_ar803x_read(dev, MII_MMD_AADR, &val);
		if (ret) {
			return ret;
		}

		ret = phy_ar803x_write(dev, MII_MMD_AADR, val &~AT803X_MMD3_SMARTEEE_CTL3_LPI_EN);
		if (ret) {
			return ret;
		}
	}

	return ret;
}

static int phy_ar803x_cfg_link(const struct device *dev,
					enum phy_link_speed adv_speeds)
{
	const struct ar803x_config *config = dev->config;
	struct ar803x_data *data = dev->data;
	struct phy_link_state state = {};
	int ret;
	uint32_t anar = 0;
	uint32_t gbcr = 0;

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
		ret = phy_ar803x_reset(dev);
		if (ret) {
			goto done;
		}
	}

	/* DT configurations */
	ret = phy_ar803x_static_cfg(dev);
	if (ret) {
		goto done;
	}

	/* Read ANAR register to write back */
	ret = phy_ar803x_read(dev, MII_ANAR, &anar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) advertising register", config->addr);
		goto done;
	}

	/* Read ANAR register to write back */
	ret = phy_ar803x_read(dev, MII_ANAR, &anar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) advertising register", config->addr);
		goto done;
	}

	/* Read GBCR register to write back */
	ret = phy_ar803x_read(dev, MII_1KTCR, &gbcr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) 1000Base-T control register", config->addr);
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
		anar |= (1 << 8);
	} else {
		anar &= ~MII_ADVERTISE_100_HALF;
	}

	/* Setup 1000Base-T control register */
	if (adv_speeds & LINK_FULL_1000BASE_T) {
		gbcr |= MII_ADVERTISE_1000_FULL;
	} else {
		gbcr &= ~MII_ADVERTISE_1000_FULL;
	}

	if (adv_speeds & LINK_HALF_1000BASE_T) {
		gbcr |= MII_ADVERTISE_1000_HALF;
	} else {
		gbcr &= ~MII_ADVERTISE_1000_HALF;
	}

	/* Write capabilities to advertising register */
	ret = phy_ar803x_write(dev, MII_ANAR, anar);
	if (ret) {
		LOG_ERR("Error writing phy (%d) advertising register", config->addr);
		goto done;
	}

	/* Write capabilities to advertising register */
	ret = phy_ar803x_write(dev, MII_1KTCR, gbcr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) 1000Base-T control register", config->addr);
		goto done;
	}


	/* (re)do autonegotiation */
	ret = phy_ar803x_autonegotiate(dev);
	if (ret && (ret != -ENETDOWN)) {
		LOG_ERR("Error in autonegotiation");
		goto done;
	}

	/* Get link status */
	ret = phy_ar803x_get_link(dev, &state);

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

static int phy_ar803x_link_cb_set(const struct device *dev,
					phy_callback_t cb, void *user_data)
{
	struct ar803x_data *data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;

	phy_ar803x_get_link(dev, &data->state);

	data->cb(dev, &data->state, data->cb_data);

	return 0;
}

static void phy_ar803x_monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct ar803x_data *data =
		CONTAINER_OF(dwork, struct ar803x_data, phy_monitor_work);
	const struct device *dev = data->dev;
	struct phy_link_state state = {};
	int rc;

	rc = phy_ar803x_get_link(dev, &state);

	if (rc == 0 && memcmp(&state, &data->state, sizeof(struct phy_link_state)) != 0) {
		memcpy(&data->state, &state, sizeof(struct phy_link_state));
		if (data->cb) {
			data->cb(dev, &data->state, data->cb_data);
		}
	}

	/* TODO change this to GPIO interrupt driven */
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int phy_ar803x_init(const struct device *dev)
{
	const struct ar803x_config *config = dev->config;
	struct ar803x_data *data = dev->data;
	int ret;

	data->dev = dev;

	ret = k_mutex_init(&data->mutex);
	if (ret) {
		return ret;
	}

	mdio_bus_enable(config->mdio_dev);

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
		ret = phy_ar803x_reset(dev);
		if (ret) {
			return ret;
		}
	}

	uint32_t val;

	ret = phy_ar803x_read(dev, MII_PHYID1R, &val);
	if (ret) {
		return ret;
	}
	data->chip_id = val << 16;

	ret = phy_ar803x_read(dev, MII_PHYID2R, &val);
	if (ret) {
		return ret;
	}

	data->chip_id |= val;

	k_work_init_delayable(&data->phy_monitor_work,
				phy_ar803x_monitor_work_handler);

	return 0;
}

static const struct ethphy_driver_api ar803x_phy_api = {
	.get_link = phy_ar803x_get_link,
	.cfg_link = phy_ar803x_cfg_link,
	.link_cb_set = phy_ar803x_link_cb_set,
	.read = phy_ar803x_read,
	.write = phy_ar803x_write,
};

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#define RESET_GPIO(n) \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),
#else
#define RESET_GPIO(n)
#endif /* reset gpio */

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
#define INTERRUPT_GPIO(n) \
		.interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),
#else
#define INTERRUPT_GPIO(n)
#endif /* interrupt gpio */

#define QCA_AR803X_INIT(n)						\
	static const struct ar803x_config ar803x_##n##_config = {	\
		.addr = DT_INST_REG_ADDR(n),					\
		.mdio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),			\
		.phy_iface = DT_INST_ENUM_IDX(n, qca_interface_type),		\
		.no_reset = DT_INST_PROP(n, no_reset), \
		RESET_GPIO(n)							\
		INTERRUPT_GPIO(n)						\
		.enableEEE = (DT_INST_NODE_HAS_PROP(n, qca_disable_smarteee) ? false : true), \
		.clock_out_frequency = DT_INST_PROP_OR(n, qca_clk_out_frequency, AR803x_CLK_25M_INVALID), \
		.clock_out_strength = DT_INST_PROP_OR(n, qca_clk_out_strength, AR803x_CLK_25M_DR_INVALID), \
	};									\
										\
	static struct ar803x_data ar803x_##n##_data;			\
										\
	DEVICE_DT_INST_DEFINE(n, &phy_ar803x_init, NULL,			\
			&ar803x_##n##_data, &ar803x_##n##_config,	\
			POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,			\
			&ar803x_phy_api);

DT_INST_FOREACH_STATUS_OKAY(QCA_AR803X_INIT)
