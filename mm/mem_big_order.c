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
#include "internal.h"
#include <linux/seq_file.h>
#include <linux/debugfs.h>

#define SYSTEM_BOOT_TIME 	150	/* 150s */
#define MAX_TRACE			16	/* stack trace length */
#define MAX_OBJECT			100
#define SCAN_TIMEOUT		100	/* 100s san time out */
#define CLEAR_OBJECT		"clear"
static int mem_big_order_enable = 1;
unsigned int system_boot_time = SYSTEM_BOOT_TIME;
static unsigned int scan_timeout = SCAN_TIMEOUT;
extern unsigned int bsp_test_mode;
static struct kobject *mbo_kobj;

/* the list of all allocated objects */
static LIST_HEAD(object_list);
/* rw_lock protecting the access to object_list */
static DEFINE_RWLOCK(big_order_lock);
/* allocation caches for big order monitor internal data */
static struct kmem_cache *object_cache;
/* object_count <= MAX_OBJECT */
static atomic_t object_count;

static struct task_struct *scan_thread;

/* protects the memory scanning */
static DEFINE_MUTEX(scan_mutex);

#define gfp_big_order_mask(gfp)	(((gfp) & (GFP_KERNEL | GFP_ATOMIC)) | \
				 __GFP_NORETRY | __GFP_NOMEMALLOC | \
				 __GFP_NOWARN)

extern void *kmem_cache_alloc(struct kmem_cache *s, gfp_t gfpflags);
/*
 * big_order_object add to object_list, big_order_lock protected this list
 * object data protected by spinlock，
*/
struct big_order_object {
	spinlock_t lock;
	struct list_head object_list;
	struct rcu_head rcu;		/* object_list lockless traversal */
	int order;
	gfp_t gfp_mask;
	unsigned long trace[MAX_TRACE];
	unsigned int trace_len;
	unsigned long jiffies;		/* creation timestamp */
	pid_t pid;			/* pid of the current task */
	char comm[TASK_COMM_LEN];	/* executable name */
};

static int get_stack_trace(unsigned long *trace)
{
	struct stack_trace stack_trace;

	stack_trace.max_entries = MAX_TRACE;
	stack_trace.nr_entries = 0;
	stack_trace.entries = trace;
	stack_trace.skip = 0;
	save_stack_trace(&stack_trace);

	return stack_trace.nr_entries;
}

static struct big_order_object *create_object(gfp_t gfp_mask, unsigned int order)
{
	unsigned long flags;
	struct big_order_object *object;

	object = kmem_cache_alloc(object_cache, GFP_KERNEL);
	if (!object) {
		pr_warn("Cannot allocate a big_order_object structure\n");
		return NULL;
	}

	INIT_LIST_HEAD(&object->object_list);
	spin_lock_init(&object->lock);
	object->order = order;
	object->jiffies = jiffies;
	object->gfp_mask = gfp_mask;

	/* task information */
	if (in_irq()) {
		object->pid = 0;
		strlcpy(object->comm, "hardirq", sizeof(object->comm));
	} else if (in_softirq()) {
		object->pid = 0;
		strlcpy(object->comm, "softirq", sizeof(object->comm));
	} else {
		object->pid = current->pid;
		/*
		 * not use set_task_comm()&get_task_comm here
		 */
		strlcpy(object->comm, current->comm, sizeof(object->comm));
	}

	/* kernel backtrace */
	object->trace_len = get_stack_trace(object->trace);

	write_lock_irqsave(&big_order_lock, flags);
	list_add_tail_rcu(&object->object_list, &object_list);
	atomic_inc(&object_count);
	write_unlock_irqrestore(&big_order_lock, flags);

	return object;
}

static inline void delete_object(struct big_order_object *object)
{
	unsigned long flags;

	write_lock_irqsave(&big_order_lock, flags);
	list_del_rcu(&object->object_list);
	atomic_dec(&object_count);
	write_unlock_irqrestore(&big_order_lock, flags);

	kmem_cache_free(object_cache, object);
}

static void clear_object(void)
{
	struct big_order_object *object;
	unsigned long flags;

	write_lock_irqsave(&big_order_lock, flags);
	if (atomic_read(&object_count) == 0) {
		write_unlock_irqrestore(&big_order_lock, flags);
		return;
	}
	write_unlock_irqrestore(&big_order_lock, flags);

	rcu_read_lock();
	list_for_each_entry_rcu(object, &object_list, object_list) {
		if (IS_ERR(object))
			continue;
		delete_object(object);
	}
	rcu_read_unlock();
}

static int cmp_object(void *priv, struct list_head *a, struct list_head *b)
{
	struct big_order_object *object_a, *object_b;

	if (!a || !b)
		panic("a or b is empty list");

	object_a = list_entry_rcu(a, typeof(*object_a), object_list);
	object_b = list_entry_rcu(b, typeof(*object_b), object_list);

	if (!object_a || !object_b) {
		pr_err("cmp_object error object_a %pk object_b %pk\n", object_a, object_b);
		panic("object_a or object_a is NULL point");
	}

	return memcmp(object_a->trace, object_b->trace, MAX_TRACE * sizeof(unsigned long));
}
static void big_order_scan(void)
{
	struct big_order_object *object;
	struct big_order_object *next;
	unsigned long flags;

	if (list_empty(&object_list))
		return;

	write_lock_irqsave(&big_order_lock, flags);
	list_sort(NULL, &object_list, cmp_object);
	write_unlock_irqrestore(&big_order_lock, flags);

	list_for_each_entry_rcu(object, &object_list, object_list) {
		if (!object)
			break;
		next = list_entry_rcu(object->object_list.next, typeof(*object), object_list);
		if (!next || (&next->object_list == &object_list))
			break;
		if ((object->trace_len == next->trace_len) && (object->order == next->order) &&
			!memcmp(object->trace, next->trace, MAX_TRACE * sizeof(unsigned long))) {
			delete_object(object);
		}
		object = next;
	}

	return;
}

static int big_order_scan_thread(void *arg)
{
	static int first_run = 1;

	pr_info("big order memory scanning thread started\n");
	set_user_nice(current, 10);

	/*
	 * Wait before the first scan to allow the system to fully initialize.
	 */
	if (first_run) {
		signed long timeout = msecs_to_jiffies(SYSTEM_BOOT_TIME * 1000);
		first_run = 0;
		while (timeout && !kthread_should_stop())
			timeout = schedule_timeout_interruptible(timeout);
	}

	while (!kthread_should_stop()) {
		signed long timeout = msecs_to_jiffies(scan_timeout * 1000);

		mutex_lock(&scan_mutex);
		big_order_scan();
		mutex_unlock(&scan_mutex);

		/* wait before the next scan */
		while (timeout && !kthread_should_stop())
			timeout = schedule_timeout_interruptible(timeout);
	}

	pr_info("big order memory scanning thread ended\n");

	return 0;
}

static void start_scan_thread(void)
{
	if (scan_thread)
		return;
	scan_thread = kthread_run(big_order_scan_thread, NULL, "big_order_scan");
	if (IS_ERR(scan_thread)) {
		pr_warn("Failed to create the scan thread\n");
		scan_thread = NULL;
	}
}

static void stop_scan_thread(void)
{
	if (scan_thread) {
		kthread_stop(scan_thread);
		scan_thread = NULL;
	}
}

int mem_big_order_monitor(gfp_t gfp_mask, unsigned int order)
{
	struct big_order_object *object;
	unsigned long flags;

	write_lock_irqsave(&big_order_lock, flags);
	if (atomic_read(&object_count) >= MAX_OBJECT) {
		write_unlock_irqrestore(&big_order_lock, flags);
		return -ENOMEM;
	}
	write_unlock_irqrestore(&big_order_lock, flags);

	object = create_object(gfp_mask, order);
	if (!object)
		return -ENOMEM;

	return 0;
}

static void get_migration_types(char *tmp, unsigned char type)
{
	static const char types[MIGRATE_TYPES] = {
		[MIGRATE_UNMOVABLE]	= 'U',
		[MIGRATE_MOVABLE]	= 'M',
		[MIGRATE_RECLAIMABLE]	= 'E',
		[MIGRATE_HIGHATOMIC]	= 'H',
#ifdef CONFIG_CMA
		[MIGRATE_CMA]		= 'C',
#endif

#ifdef CONFIG_RSC_MEM_DEFRAG
		[MIGRATE_UNMOVABLE_ISOLATE_SORD] = 'S',
		[MIGRATE_UNMOVABLE_ISOLATE_BORD] = 'B',
#endif
#ifdef CONFIG_HIGHATOMIC_POOL
		[MIGRATE_HIGHATOMIC_POOL] = 'A',
#endif
#ifdef CONFIG_MEMORY_ISOLATION
		[MIGRATE_ISOLATE]	= 'I',
#endif
	};
	char *p = tmp;
	int i;

	for (i = 0; i < MIGRATE_TYPES; i++) {
		if (type & (1 << i))
			*p++ = types[i];
	}

	*p = '\0';
}


static void print_object(struct seq_file *seq,
			       struct big_order_object *object)
{
	int i;
	char migrate[MIGRATE_TYPES + 1];
	unsigned int msecs_age = jiffies_to_msecs(jiffies - object->jiffies);

	get_migration_types(migrate, gfpflags_to_migratetype(object->gfp_mask));

	seq_printf(seq, "  comm \"%s\", pid %d, order %d, count %d, migrate %s "
			"mode:%#x(%pGg) jiffies %lu (age %d.%03ds)\n",
			object->comm, object->pid, object->order, object_count.counter, migrate,
			object->gfp_mask, &object->gfp_mask, object->jiffies, msecs_age / 1000, msecs_age % 1000);
	seq_printf(seq, "  backtrace:\n");

	for (i = 0; i < object->trace_len; i++) {
		void *ptr = (void *)object->trace[i];
		seq_printf(seq, "    [<%px>] %pS\n", ptr, ptr);
	}
}

static void *stack_seq_start(struct seq_file *seq, loff_t *pos)
{
	struct big_order_object *object;
	loff_t n = *pos;
	int err;

	err = mutex_lock_interruptible(&scan_mutex);
	if (err < 0)
		return ERR_PTR(err);

	rcu_read_lock();
	list_for_each_entry_rcu(object, &object_list, object_list) {
		if (n-- > 0)
			continue;
		if (object)
			goto out;
	}
	object = NULL;
out:
	return object;
}

static void *stack_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct big_order_object *prev_obj = v;
	struct big_order_object *next_obj = NULL;
	struct big_order_object *obj = prev_obj;

	++(*pos);

	list_for_each_entry_continue_rcu(obj, &object_list, object_list) {
		if (obj) {
			next_obj = obj;
			break;
		}
	}
	return next_obj;
}

/*
 * Decrement the use_count of the last object required, if any.
 */
static void stack_seq_stop(struct seq_file *seq, void *v)
{
	if (!IS_ERR(v)) {
		/*
		 * stack_seq_stop may return ERR_PTR if the scan_mutex
		 * waiting was interrupted, so only release it if !IS_ERR.
		 */
		rcu_read_unlock();
		mutex_unlock(&scan_mutex);
	}
}

/*
 * Print the information for an object to the seq file.
 */
static int stack_seq_show(struct seq_file *seq, void *v)
{
	struct big_order_object *object = v;
	unsigned long flags;

	spin_lock_irqsave(&object->lock, flags);
	print_object(seq, object);
	spin_unlock_irqrestore(&object->lock, flags);
	return 0;
}

static const struct seq_operations stack_seq_ops = {
	.start = stack_seq_start,
	.next  = stack_seq_next,
	.stop  = stack_seq_stop,
	.show  = stack_seq_show,
};

static int stack_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &stack_seq_ops);
}

static inline ssize_t stack_write(struct file *file,
					const char __user *buf,
					size_t count, loff_t *ppos)
{
	char *buf_data;
	int retval;
	char buf_temp[16];

	buf_data = kmalloc(count, GFP_KERNEL);
	if (buf_data == NULL)
		return count;

	retval = copy_from_user(buf_data, buf, count);
	if (retval < 0)
		goto out;

	retval = sscanf(buf_data, "%s", &buf_temp);
	if (retval < 0)
		goto out;

	if (!strncmp(buf_temp, CLEAR_OBJECT, sizeof(CLEAR_OBJECT))) {
		clear_object();
	}

out:
	kfree(buf_data);
	return count;

}

static const struct file_operations stack_fops = {
	.owner		= THIS_MODULE,
	.open		= stack_open,
	.read		= seq_read,
	.write		= stack_write,
};

static ssize_t sys_boot_time_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", system_boot_time);
}

static ssize_t sys_boot_time_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = kstrtouint(buf, 10, &system_boot_time);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t enable_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mem_big_order_enable);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = kstrtouint(buf, 10, &mem_big_order_enable);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t test_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
					"Please use cmd like 'echo order > test' to test this feature\n"
					"order mast below %d \n", MAX_ORDER);
}

static ssize_t test_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned int order;
	unsigned long *test = NULL;

	ret = kstrtouint(buf, 10, &order);
	if (ret < 0 || order > MAX_ORDER)
		return ret;

	test = (unsigned long *)kmalloc((1 << order)*PAGE_SIZE, GFP_KERNEL);

	if (test) {
		kfree(test);
		pr_err("big order monitor test alloc order:%d memory success\n", order);
	} else
		pr_err("big order monitor test alloc order:%d memory failed\n", order);

	return count;
}

static ssize_t scan_timeout_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", scan_timeout);
}

static ssize_t scan_timeout_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = kstrtouint(buf, 10, &scan_timeout);
	if (ret < 0)
		return ret;

	return count;
}

static struct kobj_attribute sys_boot_time_attr =
	__ATTR(sys_boot_time, 0664, sys_boot_time_show, sys_boot_time_store);
static struct kobj_attribute enable_attr =
	__ATTR(enable, 0664, enable_show, enable_store);
static struct kobj_attribute test_attr =
	__ATTR(test, 0664, test_show, test_store);
	static struct kobj_attribute scan_timeout_attr =
	__ATTR(scan_timeout, 0664, scan_timeout_show, scan_timeout_store);

static struct attribute *mem_big_order_attrs[] = {
	&sys_boot_time_attr.attr,
	&enable_attr.attr,
	&test_attr.attr,
	&scan_timeout_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name = "mbo_monitor",
	.attrs = mem_big_order_attrs,
};

static int __init mem_big_order_init(void)
{
	int ret;
	struct dentry *dentry;

	if (!bsp_test_mode)
		return 0;

	mbo_kobj = kobject_create_and_add("kmem_bsp_test", kernel_kobj);
	if (!mbo_kobj)
		return -ENOMEM;

	dentry = debugfs_create_file("mbo_monitor", S_IRUGO, NULL, NULL,
				     &stack_fops);
	if (!dentry)
		pr_warn("Failed to create the debugfs mem_big_order file\n");

	object_cache = KMEM_CACHE(big_order_object, SLAB_NOLEAKTRACE);

	ret = sysfs_create_group(mbo_kobj, &attr_group);
	if (ret)
		kobject_put(mbo_kobj);

	mutex_lock(&scan_mutex);
	start_scan_thread();
	mutex_unlock(&scan_mutex);

	return ret;
}

static void __exit mem_big_order_exit(void)
{
	if (bsp_test_mode) {
		sysfs_remove_group(mbo_kobj, &attr_group);
		kobject_put(mbo_kobj);
		stop_scan_thread();
		clear_object();
	}
}

module_init(mem_big_order_init);
module_exit(mem_big_order_exit);
MODULE_LICENSE("GPL v2");