#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/libfdt_env.h>
#include <linux/string.h>
#include <linux/types.h>

#define SEC_SERIAL_NUM_ADDR			0x00786134
#define	SEC_ENABLE_REG_ADDR			0x007805E8
#define SEC_KEY_VER_ADDR			(0x007801DC)
#define SEC_ROOT_CERT_HASH_ADDR		0x00780250
#define	SEC_RPMB_KEY_PROVISION_ADDR	0x00780238
#define	SEC_ANTI_ROLLBACK_ENABLE_ADDR	0x007801E4
void __iomem *rpmb_key_prov_ptr;

u32 serial_num, sec_enable, sec_key_ver_reg, sec_anti_rollback_enable;

char sec_key_hash[128];
//lizonglin add for anti-rollback start
typedef	struct{
	char *partion;
	unsigned long fuse_start_addr;
	int start_bit;
	int end_bit;
} anti_rollback_fuse_info;


static const anti_rollback_fuse_info anti_rollback_fuse_mapper[] = {

	{"EN",				0x7801E4,  0, 7},
	{"xbl",				0x780228,  0, 31},
	{"tz",				0x780234,  0, 16},
	{"aop",				0x780234, 17, 24},
	{"dp",				0x78023C, 12, 16},
	{"devcfg",			0x78023C, 17, 27},
	{NULL,				0,  0,  0},

};



//end
static int  get_secureboot_root_cert_hash(void __iomem *key_hash_addr, char *hash, const int size)
{
	char hex_str[17];
	void __iomem *reg_addr = key_hash_addr;
	uint64_t reg_val = 0;
	int i = 0; 
	strlcpy(sec_key_hash, "", sizeof(""));
	if (size < 64 || NULL == hash) 
		return 0;	
	//read hash data from gfprom
	for (i = 0; i < 6; i++) {
		reg_val = readq(reg_addr);
		reg_val = cpu_to_fdt64(reg_val); //converting val to big-endian
		snprintf(hex_str, sizeof(hex_str), "%016llx", reg_val);
		strlcat(hash, hex_str, size);
		reg_addr += 8;
	}
	return 1;
}

static int serial_num_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%08x\n",
		serial_num);
	return 0;
}

static int sec_enable_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%08x\n",
		sec_enable);
	return 0;
}

static int sec_enable_anti_rollback_proc_show(struct seq_file *m, void *v)
{
	int i = 0;
	int anti_rollback_bit_count = 0;
	int anti_rollback_verison = 0;
	void __iomem *anti_rollback_fuse_ptr;
	u32 anti_rollback_fuse_value;
	int verison_count = 0; 
	
	while (anti_rollback_fuse_mapper[i].partion != NULL) {
		anti_rollback_verison = 0;
		anti_rollback_bit_count = anti_rollback_fuse_mapper[i].end_bit - anti_rollback_fuse_mapper[i].start_bit + 1;

		//read the fuse value via the addr
		request_mem_region(anti_rollback_fuse_mapper[i].fuse_start_addr, 0x8, "sec_anti_rollback_enable"); 
		anti_rollback_fuse_ptr = ioremap(anti_rollback_fuse_mapper[i].fuse_start_addr, 0x8);
		anti_rollback_fuse_value = ioread32(anti_rollback_fuse_ptr);
	
		//starting count the bit in the fuse 
		anti_rollback_verison = anti_rollback_fuse_value >> anti_rollback_fuse_mapper[i].start_bit;
	
		for (verison_count = 0; anti_rollback_bit_count > 0; anti_rollback_verison >>= 1) {
			verison_count += anti_rollback_verison & 1; 
			anti_rollback_bit_count--;
		} 
	
		//printf the infor
		if (!strcmp(anti_rollback_fuse_mapper[i].partion, "xbl")) {
			seq_printf(m, "xlb, xbl_config,abl:%d\n", verison_count);
		} else {
			seq_printf(m, "%s:%d\n", anti_rollback_fuse_mapper[i].partion, verison_count);
		}
		release_mem_region(anti_rollback_fuse_mapper[i].fuse_start_addr, 0x8);  
		iounmap(anti_rollback_fuse_ptr);
		i++;
	}
	return 0;
}


static int key_ver_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sec_key_ver_reg >> 16);//key version bit 31:16
	return 0;
}

static int key_hash_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", sec_key_hash);	
	return 0;
}
static int rpmb_prov_proc_show(struct seq_file *m, void *v)
{
	//u32 rpmb;
	//rpmb = ioread32(rpmb_key_prov_ptr);
	if (rpmb_key_prov_ptr !=  NULL) {
		seq_printf(m, "0x%08x\n", ioread32(rpmb_key_prov_ptr) & (0x01 << 24)?1:0); //bit24
		//printk(KERN_ERR "hl2019 rpmb key provision : 0x%08x\n", rpmb);
	} else {
		printk(KERN_ERR "Sec:read rpmb key provision register error!\n");
		seq_puts(m, "0x0");
	}
	return 0;
}

static int sec_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sec_enable_proc_show, NULL);
}
static int key_ver_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, key_ver_proc_show, NULL);
}
static int key_hash_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, key_hash_proc_show, NULL);
}
static int serial_num_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, serial_num_proc_show, NULL);
}
static int rpmb_prov_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rpmb_prov_proc_show, NULL);
}

static int sec_enable_anti_rollback_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sec_enable_anti_rollback_proc_show, NULL);
}



static const struct file_operations serial_num_proc_fops = {
	.open		= serial_num_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations key_ver_proc_fops = {
	.open		= key_ver_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations key_hash_proc_fops = {
	.open		= key_hash_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static const struct file_operations sec_enable_proc_fops = {
	.open		= sec_enable_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations rpmb_prov_proc_fops = {
	.open		= rpmb_prov_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations sec_enable_anti_rollback_proc_fops = {
	.open		= sec_enable_anti_rollback_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int __init proc_sec_reg_init(void)
{
	void __iomem *sec_enable_ptr, *sec_serial_num_ptr, *key_ver_ptr;
	void __iomem *key_hash_addr_ptr, *sec_enable_anti_rollback_ptr;
	//create node 
	proc_create("sec_serial_num", 0444, NULL, &serial_num_proc_fops);
	proc_create("sec_enable", 0444, NULL, &sec_enable_proc_fops);
	proc_create("sec_anti_rollback_enable", 0444, NULL, &sec_enable_anti_rollback_proc_fops);
	proc_create("sec_key_ver", 0444, NULL, &key_ver_proc_fops);
	proc_create("sec_key_hash", 0444, NULL, &key_hash_proc_fops);
	proc_create("sec_rpmb_prov", 0444, NULL, &rpmb_prov_proc_fops);
	//request memery
	//request_mem_region(SEC_SERIAL_NUM_ADDR, 0x8, "serial_num");
	//request_mem_region(SEC_ENABLE_REG_ADDR, 0x8, "sec_enable");
	//request_mem_region(SEC_ANTI_ROLLBACK_ENABLE_ADDR, 0x8, "sec_anti_rollback_enable"); 
	//request_mem_region(SEC_KEY_VER_ADDR, 0x8, "sec_key_ver");
	//request_mem_region(SEC_ROOT_CERT_HASH_ADDR, 0x2F, "sec_key_hash");	
	//request_mem_region(SEC_RPMB_KEY_PROVISION_ADDR, 0x8, "sec_rpmb_prov");

	sec_serial_num_ptr = ioremap(SEC_SERIAL_NUM_ADDR, 0x8);
	key_ver_ptr = ioremap(SEC_KEY_VER_ADDR, 0x8);
	key_hash_addr_ptr = ioremap(SEC_ROOT_CERT_HASH_ADDR, 0x2F);
	sec_enable_ptr = ioremap(SEC_ENABLE_REG_ADDR, 0x8);
	sec_enable_anti_rollback_ptr = ioremap(SEC_ANTI_ROLLBACK_ENABLE_ADDR, 0x8);
	rpmb_key_prov_ptr = ioremap(SEC_RPMB_KEY_PROVISION_ADDR, 0x8);
	//get secure boot info 
	serial_num = ioread32(sec_serial_num_ptr);
	sec_enable = ioread32(sec_enable_ptr); 
	sec_anti_rollback_enable = ioread32(sec_enable_anti_rollback_ptr);
	sec_key_ver_reg = ioread32(key_ver_ptr);
	strlcpy(sec_key_hash, "", sizeof(""));
	get_secureboot_root_cert_hash(key_hash_addr_ptr, sec_key_hash, sizeof(sec_key_hash));
	//release
	//release_mem_region(SEC_SERIAL_NUM_ADDR, 0x8); 
	//release_mem_region(SEC_ENABLE_REG_ADDR, 0x8); 
	//release_mem_region(SEC_KEY_VER_ADDR, 0x8);
	//release_mem_region(SEC_ANTI_ROLLBACK_ENABLE_ADDR, 0x8);  
	//release_mem_region(SEC_ROOT_CERT_HASH_ADDR, 0x2F); 
	iounmap(sec_serial_num_ptr);
	iounmap(sec_enable_ptr);
	iounmap(sec_enable_anti_rollback_ptr);
	iounmap(key_ver_ptr);
	iounmap(key_hash_addr_ptr);
	return 0;
}
fs_initcall(proc_sec_reg_init);

