/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/qpnp/power-on.h>
#include <linux/of_address.h>

#include <asm/cacheflush.h>
#include <asm/system_misc.h>

#include <soc/qcom/scm.h>
#include <soc/qcom/restart.h>
#include <soc/qcom/watchdog.h>

#ifdef CONFIG_HUAWEI_KERNEL
#include <linux/huawei_apanic.h>
#endif

#ifdef CONFIG_HUAWEI_FEATURE_NFF
#include <linux/huawei_boot_log.h>
#endif
#define EMERGENCY_DLOAD_MAGIC1    0x322A4F99
#define EMERGENCY_DLOAD_MAGIC2    0xC67E4350
#define EMERGENCY_DLOAD_MAGIC3    0x77777777

#define SCM_IO_DISABLE_PMIC_ARBITER	1
#define SCM_WDOG_DEBUG_BOOT_PART	0x9
#define SCM_DLOAD_MODE			0X10
#define SCM_EDLOAD_MODE			0X01
#define SCM_DLOAD_CMD			0x10

#ifdef CONFIG_HUAWEI_KERNEL
#define SDUPDATE_FLAG_MAGIC_NUM  0x77665528
#define USBUPDATE_FLAG_MAGIC_NUM  0x77665523
#define SD_UPDATE_RESET_FLAG   "sdupdate"
#define USB_UPDATE_RESET_FLAG   "usbupdate"
#endif

#ifdef CONFIG_FEATURE_HUAWEI_EMERGENCY_DATA
#define MOUNTFAIL_MAGIC_NUM 0x77665527
#endif

#if defined(CONFIG_HUAWEI_DEBUG_MODE) || defined(CONFIG_HUAWEI_FACTORY_MODE_ENABLE_DUMP)
extern char *saved_command_line;
#endif

static int restart_mode;
void *restart_reason;
#ifdef CONFIG_HUAWEI_KERNEL
#define RESTART_FLAG_MAGIC_NUM    0x20890206
void *restart_flag_addr = NULL;
#endif
static bool scm_pmic_arbiter_disable_supported;
/* Download mode master kill-switch */
static void __iomem *msm_ps_hold;

#ifdef CONFIG_MSM_DLOAD_MODE
#define EDL_MODE_PROP "qcom,msm-imem-emergency_download_mode"
#define DL_MODE_PROP "qcom,msm-imem-download_mode"

static int in_panic;
static void *dload_mode_addr;
static bool dload_mode_enabled;
static void *emergency_dload_mode_addr;
static bool scm_dload_supported;

static int dload_set(const char *val, struct kernel_param *kp);
#ifndef CONFIG_HUAWEI_KERNEL
static int download_mode = 1;
#else
static int download_mode = 0;
#endif

module_param_call(download_mode, dload_set, param_get_int,
			&download_mode, 0644);
static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

static void set_dload_mode(int on)
{
	int ret;

	if (dload_mode_addr) {
		__raw_writel(on ? 0xE47B337D : 0, dload_mode_addr);
		__raw_writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		mb();
	}

	if (scm_dload_supported) {
		ret = scm_call_atomic2(SCM_SVC_BOOT,
				SCM_DLOAD_CMD, on ? SCM_DLOAD_MODE : 0, 0);
		if (ret)
			pr_err("Failed to set DLOAD mode: %d\n", ret);
	}

	dload_mode_enabled = on;
}

#ifdef CONFIG_HUAWEI_KERNEL
void clear_dload_mode(void)
{
    set_dload_mode(0);
}
#endif

#ifdef CONFIG_HUAWEI_FEATURE_NFF
extern void *boot_log_virt;
static void clear_bootup_flag(void)
{

	uint32_t *reboot_flag_addr = NULL;

	if (NULL == boot_log_virt) 
		reboot_flag_addr = (uint32_t *)ioremap_nocache(HUAWEI_BOOT_LOG_ADDR,0x100000);
	else 
		reboot_flag_addr = boot_log_virt;

	if(NULL != reboot_flag_addr)
	{
		uint32_t *magic    = (uint32_t*)(reboot_flag_addr);

		/* If by any chance (version upgreade) the addresses are changed,
		 * We might crash. Lets avoid that difficult to debug scenrio.
		 * We trading it off with the non-availability of NFF logs*/
		if (*magic != HUAWEI_BOOT_MAGIC_NUMBER) {
			pr_notice("NFF: Invalid magic number %x. Disabled NFF\n", *magic);
			return;
		}
		__raw_writel( 0x00000000, reboot_flag_addr);
		mb();
	}
	return;
}
#endif
static bool get_dload_mode(void)
{
	return dload_mode_enabled;
}

static void enable_emergency_dload_mode(void)
{
	int ret;

	if (emergency_dload_mode_addr) {
		__raw_writel(EMERGENCY_DLOAD_MAGIC1,
				emergency_dload_mode_addr);
		__raw_writel(EMERGENCY_DLOAD_MAGIC2,
				emergency_dload_mode_addr +
				sizeof(unsigned int));
		__raw_writel(EMERGENCY_DLOAD_MAGIC3,
				emergency_dload_mode_addr +
				(2 * sizeof(unsigned int)));

		/* Need disable the pmic wdt, then the emergency dload mode
		 * will not auto reset. */
		qpnp_pon_wd_config(0);
		mb();
	}

	if (scm_dload_supported) {
		ret = scm_call_atomic2(SCM_SVC_BOOT,
				SCM_DLOAD_CMD, SCM_EDLOAD_MODE, 0);
		if (ret)
			pr_err("Failed to set EDLOAD mode: %d\n", ret);
	}
}

static int dload_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = download_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not zero or one, ignore. */
	if (download_mode >> 1) {
		download_mode = old_val;
		return -EINVAL;
	}

	set_dload_mode(download_mode);

	return 0;
}
#else
#define set_dload_mode(x) do {} while (0)

static void enable_emergency_dload_mode(void)
{
	pr_err("dload mode is not enabled on target\n");
}

static bool get_dload_mode(void)
{
	return false;
}
#endif

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

/*
 * Force the SPMI PMIC arbiter to shutdown so that no more SPMI transactions
 * are sent from the MSM to the PMIC.  This is required in order to avoid an
 * SPMI lockup on certain PMIC chips if PS_HOLD is lowered in the middle of
 * an SPMI transaction.
 */
static void halt_spmi_pmic_arbiter(void)
{
	if (scm_pmic_arbiter_disable_supported) {
		pr_crit("Calling SCM to disable SPMI PMIC arbiter\n");
		scm_call_atomic1(SCM_SVC_PWR, SCM_IO_DISABLE_PMIC_ARBITER, 0);
	}
}

static void msm_restart_prepare(const char *cmd)
{
#ifdef CONFIG_MSM_DLOAD_MODE

	/* Write download mode flags if we're panic'ing
	 * Write download mode flags if restart_mode says so
	 * Kill download mode if master-kill switch is set
	 */

	set_dload_mode(download_mode &&
			(in_panic || restart_mode == RESTART_DLOAD));
#endif
#ifdef CONFIG_HUAWEI_KERNEL
	/*clear hardware reset magic number to imem*/
	if (in_panic || get_dload_mode())
	{
	}
	else
	{
		__raw_writel(0, hw_reset_magic_addr);
		pr_info("clear hardware reset magic number when reboot\n");
	}
#endif
#ifdef CONFIG_HUAWEI_KERNEL
   if(restart_flag_addr)
  {
   __raw_writel(RESTART_FLAG_MAGIC_NUM, restart_flag_addr);
  }
#endif

#ifdef CONFIG_HUAWEI_FEATURE_NFF
	clear_bootup_flag();
#endif     
	/* Hard reset the PMIC unless memory contents must be maintained. */
	if (in_panic || get_dload_mode() || (cmd != NULL && cmd[0] != '\0'))
    {
        pr_err("It is a warm reset, not clean the memory info \n ");
		qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);
    }
	else
    {
        pr_err("It is a cold reset, nclean the memory info \n ");
		qpnp_pon_system_pwr_off(PON_POWER_OFF_HARD_RESET);
    }

	if (cmd != NULL) {
		if (!strncmp(cmd, "bootloader", 10)) {
			__raw_writel(0x77665500, restart_reason);
		} else if (!strncmp(cmd, "recovery", 8)) {
			__raw_writel(0x77665502, restart_reason);
		} else if (!strcmp(cmd, "rtc")) {
			__raw_writel(0x77665503, restart_reason);
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned long code;
			int ret;
			ret = kstrtoul(cmd + 4, 16, &code);
			if (!ret)
				__raw_writel(0x6f656d00 | (code & 0xff),
					     restart_reason);
		} else if (!strncmp(cmd, "edl", 3)) {
			enable_emergency_dload_mode();
#ifdef CONFIG_HUAWEI_KERNEL
		} else if (!strncmp(cmd, "huawei_dload", 12)) {
			__raw_writel(0x77665503, restart_reason);
		//Added adb reboot sdupdate/usbupdate command support
		} else if(!strncmp(cmd, SD_UPDATE_RESET_FLAG, strlen(SD_UPDATE_RESET_FLAG))) {
			__raw_writel(SDUPDATE_FLAG_MAGIC_NUM, restart_reason);
		} else if(!strncmp(cmd, USB_UPDATE_RESET_FLAG, strlen(USB_UPDATE_RESET_FLAG))) {
			__raw_writel(USBUPDATE_FLAG_MAGIC_NUM, restart_reason);
#endif
#ifdef CONFIG_HUAWEI_KERNEL
		}  else if (!strncmp(cmd, "emergency_restart", 17)) {
            pr_info("do nothing\n");
#endif
#ifdef CONFIG_FEATURE_HUAWEI_EMERGENCY_DATA
		} else if (!strncmp(cmd, "mountfail", strlen("mountfail"))) {
		    __raw_writel(MOUNTFAIL_MAGIC_NUM, restart_reason);
#endif
#ifdef CONFIG_HUAWEI_KERNEL
		} else if (!strncmp(cmd, "huawei_rtc", 10)) {
			__raw_writel(0x77665524, restart_reason);
#endif
		} else {
			__raw_writel(0x77665501, restart_reason);
		}
	}

	flush_cache_all();

	/*outer_flush_all is not supported by 64bit kernel*/
#ifndef CONFIG_ARM64
	outer_flush_all();
#endif

}


static void do_msm_restart(enum reboot_mode reboot_mode, const char *cmd)
{
	int ret;

	pr_notice("Going down for restart now\n");


	msm_restart_prepare(cmd);

	/*
	 * Trigger a watchdog bite here and if this fails,
	 * device will take the usual restart path.
	 */

	if (WDOG_BITE_ON_PANIC && in_panic)
		msm_trigger_wdog_bite();

	/* Needed to bypass debug image on some chips */
	ret = scm_call_atomic2(SCM_SVC_BOOT,
			       SCM_WDOG_DEBUG_BOOT_PART, 1, 0);
	if (ret)
		pr_err("Failed to disable wdog debug: %d\n", ret);

	halt_spmi_pmic_arbiter();
	__raw_writel(0, msm_ps_hold);

	mdelay(10000);
}

static void do_msm_poweroff(void)
{
	int ret;

	pr_notice("Powering off the SoC\n");
#ifdef CONFIG_MSM_DLOAD_MODE
	set_dload_mode(0);
#endif

#ifdef CONFIG_HUAWEI_KERNEL
    /*clear hardware reset magic number to imem*/
    __raw_writel(0, hw_reset_magic_addr);
	pr_info("clear hardware reset magic number when power off\n");
#endif
	qpnp_pon_system_pwr_off(PON_POWER_OFF_SHUTDOWN);
	/* Needed to bypass debug image on some chips */
	ret = scm_call_atomic2(SCM_SVC_BOOT,
			       SCM_WDOG_DEBUG_BOOT_PART, 1, 0);
	if (ret)
		pr_err("Failed to disable wdog debug: %d\n", ret);

	halt_spmi_pmic_arbiter();
	/* MSM initiated power off, lower ps_hold */
	__raw_writel(0, msm_ps_hold);

	mdelay(10000);
	pr_err("Powering off has failed\n");
	return;
}

#ifdef CONFIG_HUAWEI_FACTORY_MODE_ENABLE_DUMP
static bool is_factory_mode(void)
{
    if (strstr(saved_command_line, "androidboot.huawei_swtype=factory") != NULL) {
        return true;
    }

    return false;
}
#endif

static int msm_restart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem;
	struct device_node *np;
	int ret = 0;

#ifdef CONFIG_MSM_DLOAD_MODE
	if (scm_is_call_available(SCM_SVC_BOOT, SCM_DLOAD_CMD) > 0)
		scm_dload_supported = true;

	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	np = of_find_compatible_node(NULL, NULL, DL_MODE_PROP);
	if (!np) {
		pr_err("unable to find DT imem DLOAD mode node\n");
	} else {
		dload_mode_addr = of_iomap(np, 0);
		if (!dload_mode_addr)
			pr_err("unable to map imem DLOAD offset\n");
	}

	np = of_find_compatible_node(NULL, NULL, EDL_MODE_PROP);
	if (!np) {
		pr_err("unable to find DT imem EDLOAD mode node\n");
	} else {
		emergency_dload_mode_addr = of_iomap(np, 0);
		if (!emergency_dload_mode_addr)
			pr_err("unable to map imem EDLOAD mode offset\n");
	}

#ifdef CONFIG_HUAWEI_FACTORY_MODE_ENABLE_DUMP
	if (is_factory_mode()) {
        download_mode = 1;
    }
#endif
    
#ifdef CONFIG_HUAWEI_DEBUG_MODE		
	if(strstr(saved_command_line,"huawei_debug_mode=1")!=NULL
	    || strstr(saved_command_line,"emcno=1")!=NULL)
	{
		download_mode = 1;
	}
#endif
	set_dload_mode(download_mode);
#endif
	np = of_find_compatible_node(NULL, NULL,
				"qcom,msm-imem-restart_reason");
	if (!np) {
		pr_err("unable to find DT imem restart reason node\n");
	} else {
		restart_reason = of_iomap(np, 0);
		if (!restart_reason) {
			pr_err("unable to map imem restart reason offset\n");
			ret = -ENOMEM;
			goto err_restart_reason;
		}
	}
#ifdef CONFIG_HUAWEI_KERNEL
    np = of_find_compatible_node(NULL, NULL,
				"qcom,msm-imem-restart_flag_addr");
	if (!np) {
		pr_err("unable to find DT imem restart flag addr node\n");
	} else {
		restart_flag_addr = of_iomap(np, 0);
		if (!restart_flag_addr) {
			pr_err("unable to map imem restart flag addr\n");
		}
	}
#endif
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	msm_ps_hold = devm_ioremap_resource(dev, mem);
	if (IS_ERR(msm_ps_hold))
		return PTR_ERR(msm_ps_hold);

	pm_power_off = do_msm_poweroff;
	arm_pm_restart = do_msm_restart;

	if (scm_is_call_available(SCM_SVC_PWR, SCM_IO_DISABLE_PMIC_ARBITER) > 0)
		scm_pmic_arbiter_disable_supported = true;

	return 0;

err_restart_reason:
#ifdef CONFIG_MSM_DLOAD_MODE
	iounmap(emergency_dload_mode_addr);
	iounmap(dload_mode_addr);
#endif
	return ret;
}

static const struct of_device_id of_msm_restart_match[] = {
	{ .compatible = "qcom,pshold", },
	{},
};
MODULE_DEVICE_TABLE(of, of_msm_restart_match);

static struct platform_driver msm_restart_driver = {
	.probe = msm_restart_probe,
	.driver = {
		.name = "msm-restart",
		.of_match_table = of_match_ptr(of_msm_restart_match),
	},
};

static int __init msm_restart_init(void)
{
	return platform_driver_register(&msm_restart_driver);
}
device_initcall(msm_restart_init);
