/*
 * Copyright (c) 2023
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SCU functions using MUs APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_IMX_SCU_H_
#define ZEPHYR_INCLUDE_DRIVERS_IMX_SCU_H_

/**
 * @brief IMX SCU Interface
 * @defgroup imx_scu_interface IMX SCU Interface
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/slist.h>

#include <main/ipc.h>
#include <svc/pm/pm_api.h>

#ifdef __cplusplus
extern "C" {
#endif
/**
 * imx_scu_ipc_handle_t is a type to identify a IPC handle
 */
typedef uint32_t imx_scu_ipc_handle;

typedef int (*imx_scu_get_ipc_handle_fn)(const struct device *dev,
				 imx_scu_ipc_handle * ipc_handle);

typedef int (*imx_scu_set_resource_power_mode_fn)(const struct device *dev, 
				 uint32_t resource_id);

struct imx_scu_driver_api {
	imx_scu_get_ipc_handle_fn			get_ipc_handle;
	imx_scu_set_resource_power_mode_fn	set_resource_power_mode;
};

/**
 * @brief Get an IPC handle
 *
 * @param dev		 Device structure.
 * @param ipc_handle IPC handle .
 * @return 0 on success, negative errno on failure.
 */
static inline int imx_scu_get_ipc_handle(const struct device *dev,
				   imx_scu_ipc_handle * ipc_handle)
{
	const struct imx_scu_driver_api *api =
		(const struct imx_scu_driver_api *)dev->api;

	return api->get_ipc_handle(dev, ipc_handle);
}

/**
 * @brief Sets the power mode of a resource.
 *
 * @param dev			Device structure.
 * @param resource_id	ID of the resource.
 * @return 0 on success, negative errno on failure.
 */
static inline int imx_scu_set_resource_power_mode(const struct device *dev,
													uint32_t resource_id)
{
	const struct imx_scu_driver_api *api =
		(const struct imx_scu_driver_api *)dev->api;

	return api->set_resource_power_mode(dev, resource_id);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_IMX_SCU_H_ */
