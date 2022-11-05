/*
 * Copyright (C) 2020 vivo Co., Ltd.
 * pengmingming <pengmingming@vivo.com>
 *
 * This driver is used to improve performance by requesting ddr bandwidth.
 *
 */

#define pr_fmt(fmt) "ddr_bw: %s(): " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include <linux/debugfs.h>


static uint32_t dsc;

#define DDR_BW_VECTOR_ENTRY(ab_val, ib_val)		\
	{												\
		.src = MSM_BUS_MASTER_AMPSS_M0,				\
		.dst = MSM_BUS_SLAVE_EBI_CH0,				\
		.ab = (ab_val),								\
		.ib = (ib_val),								\
	}

static struct msm_bus_vectors ddr_bw_vectors[] = {
	DDR_BW_VECTOR_ENTRY(0, 0),
	DDR_BW_VECTOR_ENTRY(0, 1804800 * 8 * 1000000UL),
};

static struct msm_bus_paths ddr_bw_usecases[] = { {
		.num_paths = 1,
		.vectors = &ddr_bw_vectors[0],
	}, {
		.num_paths = 1,
		.vectors = &ddr_bw_vectors[1],
	}
};

static struct msm_bus_scale_pdata ddr_bw_scale_table = {
	.usecase = ddr_bw_usecases,
	.num_usecases = ARRAY_SIZE(ddr_bw_usecases),
	.name = "ddr_bw_request",
};

static void ddr_bw_set(int idx)
{
	int ret = 0;

	if (dsc) {
		ret = msm_bus_scale_client_update_request(dsc, idx);
		if (ret)
			pr_err("ddr bandwidth request failed(ret:%d)\n", ret);
	} else {
		pr_err("client of requesting ddr bandwidth not available\n");
	}
}

int ddr_bw_enable(void)
{
	int ret = 0;
	if (dsc) {
		ret = msm_bus_scale_client_update_request(dsc, 1);
		if (ret) {
			pr_err("ddr bandwidth request failed(ret:%d)\n", ret);
			return ret;
		}
	} else {
		pr_err("client of requesting ddr bandwidth not available\n");
		ret = 1;
	}

	return ret;
}
EXPORT_SYMBOL(ddr_bw_enable);

void ddr_bw_disable(void)
{
	if (dsc)
		msm_bus_scale_client_update_request(dsc, 0);
}
EXPORT_SYMBOL(ddr_bw_disable);

static int ddr_bw_request_dbg_set(void *data, u64 val)
{
	int idx = 0;

	idx = (int)val;
	ddr_bw_set(idx);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ddr_bw_client_en_fops, NULL, ddr_bw_request_dbg_set, "%llu\n");

static int __init ddr_bw_request_register(void)
{
	struct dentry *ddr_bw_client = NULL;
	uint64_t val = 0;
	struct msm_bus_scale_pdata *pdata = &ddr_bw_scale_table;

	ddr_bw_client = debugfs_create_dir("ddr_bw_client", NULL);
	if ((!ddr_bw_client) || IS_ERR(ddr_bw_client)) {
		return -ENOMEM;
	}

	if (debugfs_create_file("ddr_bw_enable", 0644,
			ddr_bw_client, &val, &ddr_bw_client_en_fops) == NULL) {
		return -ENOMEM;
	}

	dsc = msm_bus_scale_register_client(pdata);
	if (!dsc) {
		pr_err("ddr_bw_request_register dsc %d \n", dsc);
	}

	return 0;
}

late_initcall(ddr_bw_request_register);
