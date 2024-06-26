/*
 * Copyright (c) 2024.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/drivers/firmware/imx_scu.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(power_domain_imx_scu, LOG_LEVEL_INF);

struct imx_scu_pd_config {
	uint32_t *pd;
	uint32_t pd_count;
};

static const struct device *scu_dev = DEVICE_DT_GET(DT_NODELABEL(scu));

#ifdef CONFIG_PM_DEVICE

static int pd_imx_scu_pm_action(const struct device *dev, 
								  enum pm_device_action action)
{
	struct imx_scu_pd_config * pd_list = \
										(struct imx_scu_pd_config *)dev->config;
	uint32_t i;
	int ret = 0;

	switch (action) {
		case PM_DEVICE_ACTION_RESUME:
		for(i = 0; i <= pd_list->pd_count; i++) {
			ret = imx_scu_set_resource_power_mode(scu_dev, pd_list->pd[i]);
		}
			break;
		case PM_DEVICE_ACTION_SUSPEND:
			break;
		case PM_DEVICE_ACTION_TURN_ON:
			break;
		case PM_DEVICE_ACTION_TURN_OFF:
			break;
		default:
			return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static int pd_imx_scu_init(const struct device *dev)
{
	return pm_device_driver_init(dev, pd_imx_scu_pm_action);
}

#define DT_DRV_COMPAT nxp_imx_scu_power_domain

#define POWER_DOMAIN_DEVICE(n)								\
	uint32_t pd_list##n[]=  DT_INST_PROP(n, domain_resource);	\
	struct imx_scu_pd_config pd_resource##n##config = {    \
		.pd = pd_list##n,			\
		.pd_count = DT_INST_PROP_LEN(n, domain_resource) 	\
	}; 														\
	PM_DEVICE_DT_INST_DEFINE(n, pd_imx_scu_pm_action);				\
	DEVICE_DT_INST_DEFINE(n, pd_imx_scu_init, PM_DEVICE_DT_INST_GET(n),	\
			      NULL, &pd_resource##n##config, PRE_KERNEL_1,			\
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(POWER_DOMAIN_DEVICE)
