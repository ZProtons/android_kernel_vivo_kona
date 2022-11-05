/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef __DSI_IRIS_CLK_SWITCH__
#define __DSI_IRIS_CLK_SWITCH__

void iris_switch_mipi_clk_lock(void);
int32_t iris_parse_dynamic_clk_info(struct device_node *np);
void iris_proc_switch_clk(void);
void iris_deinit_clk_switch(void);

#endif //__DSI_IRIS_CLK_SWITCH__
