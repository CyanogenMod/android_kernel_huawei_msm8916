#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/stat.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/kprobes.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <mach/msm_iomap.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <asm/barrier.h>
#include <linux/platform_device.h>
#include <linux/of_fdt.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/huawei_apanic.h>
#include <linux/of_address.h>

#define TRUE 1
#define FALSE 0


static void *crash_magic_addr;
static void *crash_len_addr;
void *hw_reset_magic_addr;

static void *huawei_apanic_virt = NULL;
unsigned int huawei_reserve_memory_size = 0;
unsigned int huawei_reserve_memory_start = 0;
static int huawei_apanic_log_length = 0;

static unsigned long hardware_reset_magic_number = 0;
static int huawei_apanic_probe(struct platform_device *pdev);

void raw_writel(unsigned long value,void *p_addr)
{
    pr_info(HUAWEI_PANIC_TAG"raw_writel p_addr=%p value=%08lx\n",p_addr,value);
   if(p_addr == NULL)
   return;
   __raw_writel(value,p_addr);
   return;
}

unsigned long raw_readl(void *p_addr)
{
    unsigned long return_value = 0;
    pr_info(HUAWEI_PANIC_TAG"raw_readl p_addr=%p \n",p_addr);
    if(p_addr == NULL)
      return return_value;

	return_value = __raw_readl(p_addr);
    pr_info(HUAWEI_PANIC_TAG"raw_readl value=%08lx \n",return_value);
    return return_value;
}

#define HUAWEI_APANIC_COMPAT_STR "huawei,huawei-mem-hole"
void clear_hw_reset()
{
    raw_writel(0, hw_reset_magic_addr);
    mb();
}

static int huawei_apanic_handler(struct notifier_block *this,
				  unsigned long event, void *ptr)
{
	int log_length = 0;
	pr_info(HUAWEI_PANIC_TAG "huawei_apanic_handler enters \n");

	/* don't re-save panic log */
	if(CRASH_LOG_MAGIC_NUM ==  raw_readl(crash_magic_addr)) {

		/* clear this log */
		raw_writel(0, crash_magic_addr);
		raw_writel(0, crash_len_addr);
		mb();

		return NOTIFY_DONE;
	}

	if(!huawei_apanic_virt) {
		return NOTIFY_DONE;
	}

#ifdef CONFIG_PREEMPT
	/* Ensure that cond_resched() won't try to preempt anybody */
	add_preempt_count(PREEMPT_ACTIVE);
#endif

	log_length = ((huawei_get_log_buf_len()>huawei_reserve_memory_size)?huawei_reserve_memory_size:huawei_get_log_buf_len());

	/* save panic info */
	_memcpy_toio(huawei_apanic_virt, (void *)huawei_get_log_buf_addr(),log_length);

	/* mark this log */
	raw_writel(CRASH_LOG_MAGIC_NUM, crash_magic_addr);
	raw_writel(log_length, crash_len_addr);
	mb();

#ifdef CONFIG_PREEMPT
        sub_preempt_count(PREEMPT_ACTIVE);
#endif

	return NOTIFY_DONE;
}

static struct notifier_block huawei_apanic_event_nb = {
	.notifier_call  = huawei_apanic_handler,
	.priority = INT_MAX,
};



static int huawei_apanic_remove(struct platform_device *pdev)
{
	atomic_notifier_chain_unregister(&panic_notifier_list, &huawei_apanic_event_nb);

    if(!huawei_apanic_virt)
	iounmap(huawei_apanic_virt);

	pr_info(HUAWEI_PANIC_TAG "Huawei apanic module remove success.\n");

	return 0;
}

static struct of_device_id huawei_apanic_match_table[] = {
	{.compatible = HUAWEI_APANIC_COMPAT_STR},
	{},
};

static struct platform_driver huawei_apanic_driver = {
	.probe = huawei_apanic_probe,
	.remove = huawei_apanic_remove,
	.driver = {
		.name = "huawei_apanic",
		.of_match_table = huawei_apanic_match_table,
	},
};

static ssize_t huawei_apanic_read(struct file *file, char __user *buf, size_t count,
			     loff_t *pos)
{
	static int read_num = 0;
	char *temp = NULL;
	int real_count = 0;
	int ret = 0;

	pr_debug(HUAWEI_PANIC_TAG "%s request length is %08x\n", __func__, count);

	if(!(CRASH_LOG_MAGIC_NUM ==  raw_readl(crash_magic_addr))) {
		printk(HUAWEI_PANIC_TAG "%s no magic number\n", __func__);
		return 0;
	}

	if(!huawei_apanic_virt) {
		printk(HUAWEI_PANIC_TAG "%s buffer have not map?\n", __func__);
		return 0;
	}

	temp = vmalloc(count);

	if(NULL == temp) {
		printk(HUAWEI_PANIC_TAG "%s malloc %d failed\n", __func__, count);
		ret = -EFAULT;
		goto error_out;
	}

	real_count = (huawei_apanic_log_length - read_num > count?
		      count:huawei_apanic_log_length - read_num);

	pr_debug(HUAWEI_PANIC_TAG "%s real count = %d\n", __func__, real_count);

	if(real_count > 0) {
		_memcpy_fromio(temp, huawei_apanic_virt + read_num, real_count);

		if(copy_to_user(buf, temp, real_count)) {
			printk(HUAWEI_PANIC_TAG "copy log %08x error!\n", count);
			ret = -EFAULT;
		}

		pr_debug(HUAWEI_PANIC_TAG "%s write log_length is %08x\n", __func__, real_count);

		read_num = read_num + real_count;
		ret = real_count;
	} else {
		ret = 0;
	}

	vfree(temp);
	temp = NULL;

error_out:
	return ret;
}

static int huawei_apanic_open(struct inode *inode, struct file *file)
{
	pr_debug(HUAWEI_PANIC_TAG "%s enter\n", __func__);

	if(!(CRASH_LOG_MAGIC_NUM ==  raw_readl(crash_magic_addr))) {
		printk(HUAWEI_PANIC_TAG "%s no magic number\n", __func__);
		return -EFAULT;
	}

	huawei_apanic_log_length = raw_readl(crash_len_addr);

	/* avoid crashed value */
	if(huawei_apanic_log_length > MAX_LOG_BUF_LENGTH) {
		printk(HUAWEI_PANIC_TAG "%s crashed log length %08x\n", __func__, huawei_apanic_log_length);
		return -EFAULT;
	}

	pr_info(HUAWEI_PANIC_TAG "%s log_length is %08x\n", __func__, huawei_apanic_log_length);

	return 0;
}

static int huawei_apanic_release(struct inode *inode, struct file *file)
{
	pr_debug(HUAWEI_PANIC_TAG "%s enter\n", __func__);

	/* clear this log */
	raw_writel(0, crash_magic_addr);
	raw_writel(0, crash_len_addr);
	mb();

	return 0;
}

static const struct file_operations huawei_apanic_fops = {
	.owner = THIS_MODULE,
	.open = huawei_apanic_open,
	.release = huawei_apanic_release,
	.read = huawei_apanic_read,
};

static struct miscdevice huawei_apanic_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "huawei_apanic",
	.fops = &huawei_apanic_fops
};

static int huawei_hw_reset_open(struct inode *inode, struct file *file)
{
	pr_debug(HUAWEI_PANIC_TAG "%s enter\n", __func__);



	pr_debug(HUAWEI_PANIC_TAG "%s hardware_reset_magic_number is %lx\n", __func__, hardware_reset_magic_number);

	return 0;
}

static ssize_t huawei_hw_reset_read(struct file *file, char __user *buf, size_t count,
			     loff_t *pos)
{

	pr_debug(HUAWEI_PANIC_TAG "%s request length is %08x\n", __func__, count);

	if(count != HW_RESET_LOG_MAGIC_NUM_LEN ){
		printk(HUAWEI_PANIC_TAG "ERROR: %s magic number len must be 4\n", __func__);
		return -EINVAL;
	}

	hardware_reset_magic_number = raw_readl(hw_reset_magic_addr);
	pr_info(HUAWEI_PANIC_TAG "%s hardware_reset_magic_number is %08lx\n", __func__, hardware_reset_magic_number);

	count = sizeof(hardware_reset_magic_number);

	if(copy_to_user(buf, &hardware_reset_magic_number, count)) {
		printk(HUAWEI_PANIC_TAG "ERROR:%s copy log %08x error!\n", __func__, count);
		return -EFAULT;
	}

	pr_debug(HUAWEI_PANIC_TAG "%s read hw reset data len is %08x\n", __func__, count);

	return count;
}

static int huawei_hw_reset_release(struct inode *inode, struct file *file)
{
	pr_debug(HUAWEI_PANIC_TAG "%s enter\n", __func__);

	return 0;
}

static ssize_t huawei_hw_reset_write(struct file *fp, const char __user *buf,
              size_t count, loff_t *pos)
{
	int ret = -1;
	unsigned long magic_number = HW_RESET_LOG_MAGIC_NUM;

	if(count != HW_RESET_LOG_MAGIC_NUM_LEN ){
		printk(HUAWEI_PANIC_TAG "ERROR: %s magic number len must be 4\n", __func__);
		return -EINVAL;
	}

     ret = copy_from_user(&magic_number, buf, HW_RESET_LOG_MAGIC_NUM_LEN);
     if (ret != 0) {
         printk(HUAWEI_PANIC_TAG "ERROR: %s could not copy %i bytes\n",
                __func__, ret);
         return -EINVAL;
     }

	pr_debug(HUAWEI_PANIC_TAG "%s to be write magic number 0x%lx\n", __func__, magic_number);

	/*write hardware reset magic number to imem*/
	raw_writel(magic_number, hw_reset_magic_addr);
	pr_info(HUAWEI_PANIC_TAG "%s written hardware reset magic number[%lx] to imem[%lx]\n", __func__, magic_number, (unsigned long)hw_reset_magic_addr);

	return count;
}

static const struct file_operations huawei_hw_reset_fops = {
	.owner = THIS_MODULE,
	.open = huawei_hw_reset_open,
	.release = huawei_hw_reset_release,
	.read = huawei_hw_reset_read,
	.write = huawei_hw_reset_write,
};

static struct miscdevice huawei_hw_reset_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "huawei_hw_reset",
	.fops = &huawei_hw_reset_fops
};

static int huawei_apanic_probe(struct platform_device *pdev)
{
   struct device_node *np;

   pr_info(HUAWEI_PANIC_TAG " huawei_apanic_probe.\n");
	if(!huawei_reserve_memory_start || !huawei_reserve_memory_size) {
		pr_err(HUAWEI_PANIC_TAG "got reserved memory failed\n");
		goto out;
	}

	pr_info(HUAWEI_PANIC_TAG "got reserved memory start %08x, size %08x.\n",
	       huawei_reserve_memory_start,
	       huawei_reserve_memory_size);

	huawei_apanic_virt = ioremap_nocache(huawei_reserve_memory_start, huawei_reserve_memory_size);

	pr_info(HUAWEI_PANIC_TAG "phys: %08x, virt %08x \n",
	       huawei_reserve_memory_start,
	       (int)huawei_apanic_virt);


	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-crash_magic_addr");

	if (!np) {
		pr_err(HUAWEI_PANIC_TAG "unable to find DT imem crash_magic_addr node\n");
		goto crash_magic_addr_out;
	} else {
		crash_magic_addr = of_iomap(np, 0);
		if (!crash_magic_addr)
		{
			pr_err(HUAWEI_PANIC_TAG "unable to map imem crash_magic_addr offset\n");
		    goto crash_magic_addr_out;
        }
	}


	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-crash_len_addr");

	if (!np) {
		pr_err(HUAWEI_PANIC_TAG "unable to find DT imem crash_len_addr node\n");
		goto error_crash_len_addr;
	} else {
		crash_len_addr = of_iomap(np, 0);
		if (!crash_len_addr)
		{
			pr_err(HUAWEI_PANIC_TAG "unable to map imem crash_len_addr offset\n");
		    goto error_crash_len_addr;
        }
	}

	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-hw_reset_magic_addr");

	if (!np) {
		pr_err(HUAWEI_PANIC_TAG "unable to find DT imem hw_reset_magic_addr node\n");
		goto error_hw_reset_magic_addr;
	} else {
		hw_reset_magic_addr = of_iomap(np, 0);
		if (!hw_reset_magic_addr)
		{
			pr_err(HUAWEI_PANIC_TAG"unable to map imem hw_reset_magic_addr offset\n");
		    goto error_hw_reset_magic_addr;
        }
	}


	if(crash_magic_addr)
    {
	    pr_info(HUAWEI_PANIC_TAG"magic_addr=%p \n",crash_magic_addr);
    }

	if(crash_len_addr)
    {
        pr_info(HUAWEI_PANIC_TAG"crash_len_addr=%p \n",crash_len_addr);
    }

	if(hw_reset_magic_addr)
    {
        pr_info(HUAWEI_PANIC_TAG"hw_reset_magic_addr=%p \n",hw_reset_magic_addr);
    }
	/* regitster the panic & reboot notifier */
	atomic_notifier_chain_register(&panic_notifier_list, &huawei_apanic_event_nb);

	pr_info(HUAWEI_PANIC_TAG "Huawei apanic module probe before register apnic.\n");

    if(CRASH_LOG_MAGIC_NUM ==  raw_readl(crash_magic_addr)){
		/* register this misc dev only when there is valie panic log */
		misc_register(&huawei_apanic_miscdev);
	}

	pr_info(HUAWEI_PANIC_TAG "Huawei apanic module probe success.\n");
	
	goto out;

error_hw_reset_magic_addr:
pr_info(HUAWEI_PANIC_TAG "unmap crash_len_addr");
iounmap(crash_len_addr);

error_crash_len_addr:
pr_info(HUAWEI_PANIC_TAG "unmap crash_magic_addr");
iounmap(crash_magic_addr);

crash_magic_addr_out:
pr_info(HUAWEI_PANIC_TAG "unmap huawei_apanic_virt");
iounmap(huawei_apanic_virt);

out:
	return 0;
}

static int __init huawei_apanic_init(void)
{



	misc_register(&huawei_hw_reset_miscdev);
	return platform_driver_register(&huawei_apanic_driver);
}

static void __exit huawei_apanic_exit(void)
{
	platform_driver_unregister(&huawei_apanic_driver);
}

module_init(huawei_apanic_init);
module_exit(huawei_apanic_exit);
MODULE_LICENSE(GPL);
EXPORT_COMPAT("huawei,huawei-mem-hole");
