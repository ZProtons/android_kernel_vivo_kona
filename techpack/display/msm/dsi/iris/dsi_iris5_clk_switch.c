// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <dsi_drm.h>
#include "dsi_iris5_lightup.h"
#include "dsi_iris5_api.h"
#include "dsi_iris5_log.h"


#define CLK_4M_HZ	4000000
#define CLK_DIV_BASE	9600000

static bool panel_dynamic_clk_supported;
static bool iris_dynamic_clk_supported;
static uint32_t mipi_clk_div;
static bool mipi_pll_update;
static int32_t pll_elem_count;
static uint32_t *pll_opt_arry;

void iris_set_panel_dynamic_clk_cap(struct dsi_panel *panel)
{
	if (!iris_is_chip_supported())
		return;

	if (panel == NULL)
		return;

	if (panel->is_secondary)
		return;

	panel_dynamic_clk_supported = panel->dyn_clk_caps.dyn_clk_support;

	IRIS_LOGI("%s(), panel support dynamic clock: %s", __func__,
			panel_dynamic_clk_supported ? "true" : "false");
}

int32_t iris_parse_dynamic_clk_info(struct device_node *np)
{
	int i = 0;
	const uint8_t *key = "pxlw,dynamic-clk-pll-opt-map";

	pll_elem_count = of_property_count_u32_elems(np, key);
	if (pll_elem_count <= 0 || pll_elem_count % 2) {
		IRIS_LOGI("%s(), iris doesn't support dynamic clock switch", __func__);
		return 0;
	}

	if (pll_opt_arry)
		vfree(pll_opt_arry);

	pll_opt_arry = vzalloc(pll_elem_count * sizeof(uint32_t));
	if (pll_opt_arry == NULL) {
		IRIS_LOGE("%s(), failed to alloc buffer for dynamic clock", __func__);
		return 0;
	}

	if (of_property_read_u32_array(np, key, pll_opt_arry, pll_elem_count)) {
		IRIS_LOGW("%s(), failed to parse pll opt arry", __func__);
		return 0;
	}

	// clock div, mipi pll opt
	for (i = 0; i < pll_elem_count; i += 2)
		IRIS_LOGI("%s(), [%d 0x%02X]", __func__,
				pll_opt_arry[i], pll_opt_arry[i + 1]);

	iris_dynamic_clk_supported = true;

	IRIS_LOGI("%s(), iris support dynamic clock switch", __func__);

	return 0;
}

static bool _iris_switch_clk_supported(void)
{
	return panel_dynamic_clk_supported && iris_dynamic_clk_supported;
}

void iris_config_mipi_clk(int clk_rate)
{
	IRIS_LOGI("%s()", __func__);

	if (!_iris_switch_clk_supported())
		return;

	IRIS_LOGI("%s(), mipi clk rate %d", __func__, clk_rate);

	mipi_pll_update = true;
	if (do_div(clk_rate, CLK_DIV_BASE) >= CLK_4M_HZ) {
		mipi_clk_div = clk_rate / 2;
	} else {
		mipi_clk_div = clk_rate;
	}

	IRIS_LOGI("%s(), clk rate %d, clk div %d", __func__, clk_rate, mipi_clk_div);
}

static int32_t _iris_select_pll_opt(void)
{
	int i = 0;

	IRIS_LOGI("%s(), arry elem count: %d", __func__, pll_elem_count);

	if (pll_elem_count <= 0)
		return -EINVAL;

	if (pll_opt_arry == NULL)
		return -EINVAL;

	for (i = 0; i < pll_elem_count; i += 2) {
		if (pll_opt_arry[i] == mipi_clk_div)
			return pll_opt_arry[i + 1];
	}

	return -EINVAL;
}

void iris_proc_switch_clk(void)
{
	int32_t mipi_pll_opt = 0;

	IRIS_LOGI("%s()", __func__);

	if (!_iris_switch_clk_supported())
		return;

	if (!mipi_pll_update)
		return;

	mipi_pll_update = false;
	mipi_pll_opt = _iris_select_pll_opt();
	if (mipi_pll_opt < 0)
		return;

	iris_send_ipopt_cmds(IRIS_IP_SYS, mipi_pll_opt);

	IRIS_LOGI("%s(), set pll opt 0x%02X to Iris for div %d",
			__func__, mipi_pll_opt, mipi_clk_div);
}

void iris_switch_mipi_clk(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_display *display = pcfg->display;

	IRIS_LOGI("%s()", __func__);
	if (display && !atomic_read(&display->clkrate_change_pending) &&
			display->cached_clk_rate == pcfg->clk_rate[1]) {
		atomic_set(&pcfg->set_dsi_mode, 1);
		IRIS_LOGI("%s() display dsi clk %u", __func__, display->cached_clk_rate);
	} else if (display && !atomic_read(&display->clkrate_change_pending) &&
			display->cached_clk_rate == pcfg->clk_rate[0]) {
		atomic_set(&pcfg->set_dsi_mode, 0);
		IRIS_LOGI("%s() display dsi clk %u", __func__, display->cached_clk_rate);
	}
	
	if (pcfg->abypss_ctrl.abypass_mode != PASS_THROUGH_MODE)
		return;

	iris_proc_switch_clk();
}

void iris_switch_mipi_clk_lock(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!_iris_switch_clk_supported())
		return;
	if (!mipi_pll_update)
		return;

	IRIS_LOGI("%s()", __func__);

	mutex_lock(&pcfg->panel->panel_lock);
	iris_proc_switch_clk();
	mutex_unlock(&pcfg->panel->panel_lock);
}

void iris_deinit_clk_switch(void)
{
	IRIS_LOGI("%s()", __func__);

	if (pll_opt_arry) {
		vfree(pll_opt_arry);
		pll_opt_arry = NULL;
	}
}
