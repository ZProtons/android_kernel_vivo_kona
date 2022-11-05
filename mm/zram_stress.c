/*
 * mm/mem_big_order.c
 *
 * VIVO Kernel Memory Monitor
 *
 * <rongqianfeng@vivo.com>
 *
*/
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mmzone.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/list_sort.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

static unsigned int zram_stress_enable = 1;
#define ZS_BUF_SIZE (4*PAGE_SIZE)
extern unsigned int bsp_test_mode;

struct zram_stress_data {
	ssize_t buf_size;
	char *buf;
	char *rd, *wr, *end;
};

/* rw_lock protecting the access to zs_buffer */
static DEFINE_RWLOCK(zs_lock);
static struct zram_stress_data zs_buffer;

static int zram_stress_data_init(void)
{
	int ret = 0;

	zs_buffer.buf = kzalloc(ZS_BUF_SIZE, GFP_KERNEL);

	if (!zs_buffer.buf) {
		pr_err("zram_stress: alloc full failed, may memory leak issue");
		return -ENOMEM;
	}

	zs_buffer.rd = zs_buffer.buf;
	zs_buffer.wr = zs_buffer.buf;
	zs_buffer.end = zs_buffer.buf + ZS_BUF_SIZE;
	zs_buffer.buf_size = ZS_BUF_SIZE;

	return ret;
}

static ssize_t
zram_stress_read(struct file *filp, char __user *ubuf, size_t count,
			loff_t *ppos)
{
	ssize_t len = count;
	unsigned long flags;

	if (zs_buffer.rd < zs_buffer.wr)
		len = min(len, (ssize_t)(zs_buffer.wr - zs_buffer.rd));
	else
		return 0;

	*ppos = zs_buffer.rd - zs_buffer.buf;

	if (copy_to_user(ubuf, zs_buffer.buf + (*ppos), len)) {
		pr_err("zram_stress: read error");
		return -EFAULT;
	} else {
		write_lock_irqsave(&zs_lock, flags);
		zs_buffer.rd += len;
		write_unlock_irqrestore(&zs_lock, flags);
	}

	return len;
}

static ssize_t
zram_stress_write(struct file *filp, const char __user *ubuf,
					size_t count, loff_t *ppos)
{
	size_t len = count;
	unsigned long flags;

	if (len > ZS_BUF_SIZE)
		len = ZS_BUF_SIZE;

	write_lock_irqsave(&zs_lock, flags);

	if (zs_buffer.wr < zs_buffer.end)
		len = min(len, (size_t)(zs_buffer.end - zs_buffer.wr));
	else if (zs_buffer.wr == zs_buffer.end) {
		pr_err("zram_stress: stop write, zs_buffer full");
		len = -EINVAL;
		goto out;
	} else {
		pr_err("zram_stress: memory corruption");
		BUG_ON(1);
	}

	if (zs_buffer.wr + len > zs_buffer.end) {
		pr_err("zram_stress: stop write, zs_buffer full");
		len = -EINVAL;
		goto out;
	}

	*ppos = zs_buffer.wr - zs_buffer.buf;

	if (copy_from_user(zs_buffer.buf + (*ppos), ubuf, len)) {
		pr_err("zram_stress: write error");
		len = -EFAULT;
		goto out;
	} else {
		zs_buffer.wr += len;
	}

out:
	write_unlock_irqrestore(&zs_lock, flags);
	return len;
}

static int zram_stress_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations zram_stress_fops = {
	.open		= generic_file_open,
	.read		= zram_stress_read,
	.write		= zram_stress_write,
	.llseek		= generic_file_llseek,
	.release	= zram_stress_release,
};

static int __init zram_stress_init(void)
{
	int ret = 0;
	struct dentry *dentry;

	if (!bsp_test_mode || !zram_stress_enable)
		return ret;

	ret = zram_stress_data_init();

	if (ret)
		return ret;

	dentry = debugfs_create_file("zram_stress", S_IRUGO, NULL, NULL,
				     &zram_stress_fops);
	if (!dentry)
		pr_warn("Failed to create the debugfs mem_big_order file\n");

	return ret;
}

static void __exit zram_stress_exit(void)
{
	if (bsp_test_mode && zram_stress_enable) {
		if (zs_buffer.buf)
			kfree(zs_buffer.buf);
	}
}

module_init(zram_stress_init);
module_exit(zram_stress_exit);
MODULE_LICENSE("GPL v2");