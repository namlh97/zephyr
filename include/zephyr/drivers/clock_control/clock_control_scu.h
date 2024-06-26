/*
 * Copyright (c) 2023.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/dt-bindings/clock/imx_scu_clock.h>

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_SCU_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_SCU_H_

#define CLOCK_SCU_MAX_OPS 6

struct clock_scu_clk_id {
	uint32_t rsrc_id;
	uint32_t gpr_id;
	uint32_t val;
};

struct clock_scu_subsys {
	uint32_t num;
	struct clock_scu_clk_id clock_ops_list[CLOCK_SCU_MAX_OPS];
};

/** Device tree clocks helpers  */
#define CLOCK_SCU_GET_ID(idx, n) { 											  \
				.rsrc_id = IMX_LPCG_TUPLE_RSRC(								  \
					DT_CLOCKS_CELL_BY_IDX(n, idx, name)), 		  \
				.gpr_id = DT_CLOCKS_CELL_BY_IDX(n, idx, gpr_id), \
				.val = DT_CLOCKS_CELL_BY_IDX(n, idx, value),}

#define CLOCK_SCU_GET_CONFIG(n) 						\
	(struct clock_scu_subsys) {  													\
		.num = DT_NUM_CLOCKS(n), 			\
		.clock_ops_list = { 							\
				LISTIFY(DT_NUM_CLOCKS(n),	\
					CLOCK_SCU_GET_ID, (,), n) 	\
		}, 												\
	}

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_SCU_H_ */
