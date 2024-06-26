/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/firmware/imx_scu.h>
#include <zephyr/dt-bindings/clock/imx_scu_clock.h>
#include <zephyr/drivers/clock_control/clock_control_scu.h>
#include <fsl_clock.h>

#include <main/ipc.h>
#include <svc/misc/misc_api.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control);

static const struct device *scu_dev = DEVICE_DT_GET(DT_NODELABEL(scu));
imx_scu_ipc_handle clk_ipc_handle;

static int mcux_ccm_on(const struct device *dev,
			      clock_control_subsys_t sub_system)
{
	uint32_t clock_name = (uintptr_t)sub_system;

	CLOCK_EnableClock(clock_name);

	return 0;
}

static int mcux_ccm_off(const struct device *dev,
			       clock_control_subsys_t sub_system)
{
	uint32_t clock_name = (uintptr_t)sub_system;

	CLOCK_DisableClock(clock_name);

	return 0;

}

static int mcux_ccm_get_subsys_rate(const struct device *dev,
				    clock_control_subsys_t sub_system,
				    uint32_t *rate)
{
	uint32_t clock_name = (uintptr_t)sub_system;

	*rate = CLOCK_GetIpFreq(clock_name);

	return 0;
}

static int mcux_ccm_set_subsys_rate(const struct device *dev,
				    clock_control_subsys_t sub_system,
				    clock_control_subsys_rate_t rate)
{
	uint32_t clock_name = (uintptr_t)sub_system;
	uint32_t clock_rate = (uintptr_t)rate;

    /* Set clock Frequency. */
    clock_rate = CLOCK_SetIpFreq(clock_name, clock_rate);

	return 0;
}

static int mcux_ccm_set_subsys_configure(const struct device *dev,
				    clock_control_subsys_t sub_system,
				    void *data)
{
	uint32_t i;
	struct clock_scu_subsys *clk_config = (struct clock_scu_subsys *)data;

	for (i = 0; i <= clk_config->num; i++) {
		if (clk_config->clock_ops_list[i].gpr_id != IMX_SC_PM_CLK_PER) {
			sc_misc_set_control(clk_ipc_handle, \
					clk_config->clock_ops_list[i].rsrc_id, \
					clk_config->clock_ops_list[i].gpr_id, \
					clk_config->clock_ops_list[i].val);
		}
	}

	return 0;
}

static const struct clock_control_driver_api mcux_ccm_driver_api = {
	.on = mcux_ccm_on,
	.off = mcux_ccm_off,
	.get_rate = mcux_ccm_get_subsys_rate,
	.set_rate = mcux_ccm_set_subsys_rate,
	.configure = mcux_ccm_set_subsys_configure,
};

static int mcux_ccm_init(const struct device *dev)
{
	int ret;

	ret = imx_scu_get_ipc_handle(scu_dev, &clk_ipc_handle);
	if (0 != ret) {
		return ret;
	}

	CLOCK_Init(clk_ipc_handle);

	return 0;
}

#define DT_DRV_COMPAT nxp_imx_clock_scu

DEVICE_DT_INST_DEFINE(0, mcux_ccm_init, NULL, NULL, NULL,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &mcux_ccm_driver_api);
