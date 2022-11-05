// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

extern char qpnp_pon_reason_str[];
extern char qpnp_poff_reason_str[];
extern char *vivo_get_pmic_status(void);

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s androidboot.bootreason=%s/%s/%s\n",
			saved_command_line,
			qpnp_pon_reason_str,
			qpnp_poff_reason_str,
			vivo_get_pmic_status());
	return 0;
}

static int __init proc_cmdline_init(void)
{
	proc_create_single("cmdline", 0, NULL, cmdline_proc_show);
	return 0;
}
fs_initcall(proc_cmdline_init);
