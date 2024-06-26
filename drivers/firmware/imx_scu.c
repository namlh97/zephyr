/*
 * Copyright (c) 2023,
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_scu
#include <errno.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/firmware/imx_scu.h>

#define LOG_LEVEL CONFIG_FIRMWARE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(firmware);

sc_ipc_t scu_ipc_handle = 0;

static int scu_get_ipc_handle(const struct device *dev,
				   imx_scu_ipc_handle * ipc_handle)
{
	if (0 == scu_ipc_handle) {
		return -ENODEV;
	}

	* ipc_handle = (imx_scu_ipc_handle)scu_ipc_handle;

	return 0;
}

static int scu_set_resource_power_mode(const struct device *dev, uint32_t resource_id)
{
	sc_err_t err = SC_ERR_NONE;

	if (0 == scu_ipc_handle) {
		return -ENODEV;
	}
	err = sc_pm_set_resource_power_mode(scu_ipc_handle, resource_id, SC_PM_PW_MODE_ON);

	return (err == SC_ERR_NONE ? 0 : -EIO);
}

static const struct imx_scu_driver_api imx_scu_driver_api = {
	.get_ipc_handle = 			scu_get_ipc_handle,
	.set_resource_power_mode =	scu_set_resource_power_mode,
};

static int imx_scu_init(const struct device *dev)
{
	sc_err_t err = SC_ERR_NONE;

	err = sc_ipc_open(&scu_ipc_handle, DT_REG_ADDR(DT_INST(0, nxp_imx_mu_scu)));
	if (err != SC_ERR_NONE) {
		return -ENODEV;
	}

	return 0;
}

DEVICE_DT_INST_DEFINE(0, imx_scu_init, NULL, NULL, NULL,
		      PRE_KERNEL_1, CONFIG_FIRMWARE_INIT_PRIORITY,
		      &imx_scu_driver_api);
