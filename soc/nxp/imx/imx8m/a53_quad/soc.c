/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/rdc/imx_rdc.h>
#include <fsl_common.h>
#include <fsl_clock.h>
#include <fsl_rdc.h>

static void soc_clock_init(void)
{
	/* ARM A53 clock, SYSTEM PLL1, SYSTEM PLL2 and SYSTEM PLL3 are configured by U-boot SPL */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(enet), okay)
	CLOCK_SetRootDivider(kCLOCK_RootEnetAxi, 1U, 1U);
    CLOCK_SetRootMux(kCLOCK_RootEnetAxi, kCLOCK_EnetAxiRootmuxSysPll2Div4); /* SYSTEM PLL2 divided by 4: 250Mhz */

    CLOCK_SetRootDivider(kCLOCK_RootEnetTimer, 1U, 1U);
    CLOCK_SetRootMux(kCLOCK_RootEnetTimer, kCLOCK_EnetTimerRootmuxSysPll2Div10); /* SYSTEM PLL2 divided by 10: 100Mhz */

    CLOCK_SetRootDivider(kCLOCK_RootEnetRef, 1U, 1U);
    CLOCK_SetRootMux(kCLOCK_RootEnetRef, kCLOCK_EnetRefRootmuxSysPll2Div8); /* SYSTEM PLL2 divided by 8: 125Mhz */
#endif
	return;
}

/* set RDC permission for peripherals */
static void soc_rdc_init(void)
{
	rdc_domain_assignment_t assignment = {0};
	rdc_periph_access_config_t periphConfig;

	RDC_Init(RDC);
	assignment.domainId = A53_DOMAIN_ID;
	RDC_SetMasterDomainAssignment(RDC, kRDC_Master_A53, &assignment);

	CLOCK_ControlGate(kCLOCK_SysPll1Gate, kCLOCK_ClockNeededAll);
	CLOCK_ControlGate(kCLOCK_SysPll2Gate, kCLOCK_ClockNeededAll);
	CLOCK_ControlGate(kCLOCK_SysPll3Gate, kCLOCK_ClockNeededAll);
	CLOCK_ControlGate(kCLOCK_AudioPll1Gate, kCLOCK_ClockNeededAll);
	CLOCK_ControlGate(kCLOCK_AudioPll2Gate, kCLOCK_ClockNeededAll);
	CLOCK_ControlGate(kCLOCK_VideoPll1Gate, kCLOCK_ClockNeededAll);
	CLOCK_ControlGate(kCLOCK_VideoPll2Gate, kCLOCK_ClockNeededAll);

	RDC_GetDefaultPeriphAccessConfig(&periphConfig);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay) && DT_NODE_HAS_PROP(DT_NODELABEL(uart2), rdc)
	periphConfig.periph = kRDC_Periph_UART2;
	periphConfig.policy = RDC_DT_VAL(uart2);
	RDC_SetPeriphAccessConfig(RDC, &periphConfig);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay) && DT_NODE_HAS_PROP(DT_NODELABEL(uart1), rdc)
	periphConfig.periph = kRDC_Periph_UART1;
	periphConfig.policy = RDC_DT_VAL(uart1);
	RDC_SetPeriphAccessConfig(RDC, &periphConfig);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(enet), okay) && DT_NODE_HAS_PROP(DT_NODELABEL(enet), rdc)
	periphConfig.periph = kRDC_Periph_ENET1;
	periphConfig.policy = RDC_DT_VAL(enet);
	RDC_SetPeriphAccessConfig(RDC, &periphConfig);
#endif
}

static int soc_init(void)
{
	/* SoC specific RDC settings */
	soc_rdc_init();

	soc_clock_init();

	return 0;
}

SYS_INIT(soc_init, EARLY, 1);
