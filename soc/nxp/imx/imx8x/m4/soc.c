/*
 * Copyright (c) 2021, Kwon Tae-young <tykwon@m2i.co.kr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <fsl_common.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>

static int nxp_mimx8qx6_init(void)
{
	return 0;
}

SYS_INIT(nxp_mimx8qx6_init, PRE_KERNEL_1, 0);
