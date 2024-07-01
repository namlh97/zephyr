/*
 * Copyright 2020-2022,2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm64/arm_mmu.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#define ANA_OSC_BASE_ADDR	0x30270000
#define ANA_OSC_BASE_SIZE	0x10000

#ifdef CONFIG_ARMV8_A_NS
#define MT_CONFIG_SECURE_STATE MT_NS
#else
#define MT_CONFIG_SECURE_STATE MT_DEFAULT_SECURE_STATE
#endif

static const struct arm_mmu_region mmu_regions[] = {

	MMU_REGION_FLAT_ENTRY("GIC",
			      DT_REG_ADDR_BY_IDX(DT_NODELABEL(gic), 0),
			      DT_REG_SIZE_BY_IDX(DT_NODELABEL(gic), 0),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_CONFIG_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("GIC",
			      DT_REG_ADDR_BY_IDX(DT_NODELABEL(gic), 1),
			      DT_REG_SIZE_BY_IDX(DT_NODELABEL(gic), 1),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_CONFIG_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("CCM",
			      DT_REG_ADDR(DT_NODELABEL(ccm)),
			      DT_REG_SIZE(DT_NODELABEL(ccm)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_CONFIG_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("ANA_PLL",
			      DT_REG_ADDR(DT_NODELABEL(ana_pll)),
			      DT_REG_SIZE(DT_NODELABEL(ana_pll)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_CONFIG_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("IOMUXC",
			      DT_REG_ADDR(DT_NODELABEL(iomuxc)),
			      DT_REG_SIZE(DT_NODELABEL(iomuxc)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_CONFIG_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("RDC",
			      DT_REG_ADDR(DT_NODELABEL(rdc)),
			      DT_REG_SIZE(DT_NODELABEL(rdc)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_CONFIG_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("ANA_OSC",
			      ANA_OSC_BASE_ADDR,
			      ANA_OSC_BASE_SIZE,
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_CONFIG_SECURE_STATE),

	MMU_REGION_DT_COMPAT_FOREACH_FLAT_ENTRY(nxp_imx_iuart,
				  (MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_CONFIG_SECURE_STATE))
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
