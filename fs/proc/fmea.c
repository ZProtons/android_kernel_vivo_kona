/*
 * Copyright (C) 2013 vivo Co., Ltd.
 * YangChun <yangchun@iqoo.com>
 *
 * This driver is used to collect system facilities failure event
 *
**/
#define pr_fmt(fmt)	"FMEA: %s: " fmt, __func__

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include "internal.h"

#include <asm/uaccess.h>
#include <asm/io.h>

#define NAME_LEN_MAX	32
#define DATA_LEN_MAX	128
#define FIFO_SIZE_MAX	512

#ifdef CONFIG_FUEL_SUMMARY
#define ENTRY_LEN_MAX	384
#else
#define ENTRY_LEN_MAX	160
#endif

struct fmea_entry{
	size_t	len;
	char	data[ENTRY_LEN_MAX];
};

struct fmea{
	wait_queue_head_t	fmea_waitq;
	spinlock_t			lock;
	struct dentry		*dent;
	DECLARE_KFIFO_PTR(fmea_fifo, struct fmea_entry *);
};

static struct fmea *sfmea;

#define ENTRY_FMT "%s,%s\n"

static int fmea_write_entry(struct fmea *fmea, struct fmea_entry *entry)
{
	int err = 0;
	struct fmea_entry *dr;

	if (!entry->len) {
		err = -EFAULT;
		goto out;
	}
	if (entry->data[entry->len-1] != '\n')
		entry->data[entry->len-1] = '\n';

	if (kfifo_is_full(&fmea->fmea_fifo)) {
		if (!kfifo_get(&fmea->fmea_fifo, &dr)) {
			err = -EFAULT;
			goto out;
		}
	}

	if (!kfifo_put(&fmea->fmea_fifo,
		entry)) {
		err = -EFAULT;
		goto out;
	}

out:
	return err;
}

int fmea_notify(char *name, char *data)
{
	int err = 0;
	struct fmea_entry *entry;
	if (!sfmea) {
		err = -ENOENT;
		goto out;
	}

	entry = kzalloc(sizeof(struct fmea_entry), GFP_KERNEL);
	if (!entry) {
		err = -ENOMEM;
		goto out;
	}
	entry->len = scnprintf(entry->data,
		ENTRY_LEN_MAX, ENTRY_FMT, name, data);

	spin_lock_irq(&sfmea->lock);
	err = fmea_write_entry(sfmea, entry);
	spin_unlock_irq(&sfmea->lock);
	if (err)
		kfree(entry);
	else
		wake_up_interruptible(&sfmea->fmea_waitq);
out:
	return err;
}
EXPORT_SYMBOL(fmea_notify);

int writeData(char *modelId, char *filename, char *data)
{
	int err = 0;
	struct fmea_entry *entry;
	if (!sfmea) {
		err = -ENOENT;
		goto out;
	}

	entry = kzalloc(sizeof(struct fmea_entry), GFP_KERNEL);
	if (!entry) {
		err = -ENOMEM;
		goto out;
	}
	//entry->len = scnprintf(entry->data,
	//ENTRY_LEN_MAX, ENTRY_FMT, name, data);
	entry->len = scnprintf(entry->data,
	   ENTRY_LEN_MAX, "%s^~^%s^_^%s\n", modelId, filename, data);

	spin_lock_irq(&sfmea->lock);
	err = fmea_write_entry(sfmea, entry);
	spin_unlock_irq(&sfmea->lock);
	if (err)
		kfree(entry);
	else
		wake_up_interruptible(&sfmea->fmea_waitq);
out:
	return err;
}
EXPORT_SYMBOL(writeData);

int writeDatas(char *modelId, char *filename, char *fmt, ...)
{
	va_list args;
	char data[DATA_LEN_MAX];
	memset(data, 0, sizeof(data));
	va_start(args, fmt);
	vscnprintf(data, DATA_LEN_MAX, fmt, args);
	va_end(args);
	return writeData(modelId, filename, data);
}
EXPORT_SYMBOL(writeDatas);

int fmea_notifys(char *name, char *fmt, ...)
{
	va_list args;
	char	data[DATA_LEN_MAX];
	memset(data, 0, sizeof(data));
	va_start(args, fmt);
	vscnprintf(data, DATA_LEN_MAX, fmt, args);
	va_end(args);
	return fmea_notify(name, data);
}
EXPORT_SYMBOL(fmea_notifys);

static int fmea_open(struct inode *inode, struct file *file)
{
	if (!sfmea)
		return -ENOENT;
	file->private_data = sfmea;
	return nonseekable_open(inode, file);
}

static int fmea_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t fmea_read(struct file *file, char __user *buf,
			 size_t count, loff_t *ppos)
{
	int err = 0;
	struct fmea *fmea = file->private_data;
	struct fmea_entry *entry;
	struct fmea_entry entout;

start:
	if ((file->f_flags & O_NONBLOCK)
		&& kfifo_is_empty(&fmea->fmea_fifo)) {
		err = -EAGAIN;
		goto out;
	} else {
		err = wait_event_interruptible (fmea->fmea_waitq,
			!kfifo_is_empty(&fmea->fmea_fifo));
		if (err)
			goto out;
	}

	spin_lock_irq (&fmea->lock);
	if (kfifo_is_empty(&fmea->fmea_fifo)) {
		spin_unlock_irq(&fmea->lock);
		goto start;
	}

	if (!kfifo_peek(&fmea->fmea_fifo, &entry)) {
		err = -EFAULT;
		goto unlock;
	}

	if (count >= entry->len) {
		entout.len = entry->len;
		memcpy(entout.data, entry->data, entout.len);
		if (kfifo_get(&fmea->fmea_fifo, &entry))
			kfree(entry);
	} else {
		entout.len = count;
		memcpy(entout.data, entry->data, entout.len);
		entry->len = entry->len-entout.len;
		memmove(entry->data, entry->data+entout.len, entry->len);
	}

unlock:
	spin_unlock_irq(&fmea->lock);
	if (!err && copy_to_user(buf, entout.data, entout.len))
		err = -EFAULT;
	if (!err)
		err = entout.len;
out:
	return err;
}

static ssize_t fmea_write(struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
	int err = 0;
	size_t max = ENTRY_LEN_MAX;

	struct fmea *fmea = file->private_data;
	struct fmea_entry *entry = kzalloc(sizeof(struct fmea_entry), GFP_KERNEL);

	if (!count)
		goto error;
	if (!entry) {
		err = -ENOMEM;
		goto out;
	}
	entry->len = min(count, max);
	if (copy_from_user(entry->data, buf, entry->len)) {
		err = -EFAULT;
		goto error;
	}
	spin_lock_irq(&fmea->lock);
	err = fmea_write_entry(fmea, entry);
	spin_unlock_irq(&fmea->lock);
	if (err)
		goto error;
	else
		wake_up_interruptible(&fmea->fmea_waitq);
	return count;
error:
	kfree(entry);
out:
	return err;
}

//#ifdef CONFIG_DEBUG_FS
#if 1

static int fmea_debug_read(struct seq_file *m, void *v)
{
	struct fmea *fmea = m->private;
	seq_printf(m, "fifo-size:%d, fifo-len:%d, full:%d, empty:%d\n",
		kfifo_size(&fmea->fmea_fifo),
		kfifo_len(&fmea->fmea_fifo),
		kfifo_is_full(&fmea->fmea_fifo),
		kfifo_is_empty(&fmea->fmea_fifo));

	return 0;
}

static ssize_t fmea_debug_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	int err = 0;
	size_t max = ENTRY_LEN_MAX;
	size_t sz = min(count, max);
	struct fmea_entry entry;
	memset(&entry, 0, sizeof(entry));
	if (copy_from_user(entry.data, buf, sz)) {
		err = -EFAULT;
		goto out;
	}
	err = fmea_notify("fmea-debug", entry.data);
	err |= fmea_notifys("fmea-debug", "%s:%d,%s\n", entry.data, 1, "string");
	err |= fmea_notifys("fmea-debug", "%s:%d,%s,0x%2X\n", entry.data, 2, "string2", 0xFF);
	if (!err)
		err = count;

out:
	return err;
}

static int fmea_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, fmea_debug_read, inode->i_private);
}

static const struct file_operations proc_fmea_ops = {
	.open		= fmea_open,
	.read		= fmea_read,
	.write		= fmea_write,
	.release	= fmea_release,
	.llseek		= noop_llseek,
};

static const struct file_operations debug_fmea_ops = {
	.read		= seq_read,
	.write		= fmea_debug_write,
	.open		= fmea_debug_open,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int fmea_init_debugfs(struct fmea *fmea)
{
	struct dentry *dent;
	fmea->dent = debugfs_create_dir("fmea", NULL);
	if (IS_ERR(fmea->dent)) {
		pr_err("couldn't create debugfs dir\n");
		return -EFAULT;
	}
	dent = debugfs_create_file("debug", 0644, fmea->dent, fmea, &debug_fmea_ops);
	if (IS_ERR(dent))
		return -EFAULT;
	return 0;
}
#endif

#ifndef AID_SYSTEM
	#define AID_SYSTEM 1000
#endif

static int __init proc_fmea_init(void)
{
	int err;
	struct fmea *fmea;
	struct proc_dir_entry *pdent;
	sfmea = 0;
	fmea = kzalloc(sizeof(*fmea), GFP_KERNEL);
	if (!fmea) {
		pr_err("could not allocate fmea\n");
		err = -ENOMEM;
		goto fail;
	}
	sfmea = fmea;
	err = kfifo_alloc(&fmea->fmea_fifo, FIFO_SIZE_MAX, GFP_KERNEL);
	if (err) {
		pr_err("could not allocate fmea FIFO\n");
		goto fail_fifo;
	}
	spin_lock_init(&fmea->lock);
	init_waitqueue_head(&fmea->fmea_waitq);

	pdent = proc_create("fmea", 0664, NULL, &proc_fmea_ops);

	if (!pdent) {
		err = -EFAULT;
		goto fail_fifo;
	}

	pdent->gid.val = AID_SYSTEM;
	pdent->uid.val = AID_SYSTEM;

#if 1	//CONFIG_DEBUG_FS
	if (fmea_init_debugfs(fmea)) {
		err = -EFAULT;
		if (fmea->dent)
			debugfs_remove_recursive(fmea->dent);
		goto fail_fifo;
	}
#endif

	return 0;

fail_fifo:
	kfree(fmea);
	sfmea = 0;
fail:
	return err;
}
postcore_initcall(proc_fmea_init);
