/*
 * log_module_debug_mask_example.c
 * just example code
 *
 * Copyright (C) 2012-2014  Huawei Corporation
 */

#include <linux/log_module.h>
#include <linux/module.h>
#include <linux/moduleparam.h>


/* this module don't contain native module log system, we just control the debug mask */
DEFINE_LOG_MASK(log_module_example_debug_mask);
module_param(log_module_example_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);





int log_module_example_debug_mask_config(void * value)
{
    printk("mask altered from [%d]\n", log_module_example_debug_mask);
    log_module_example_debug_mask = (int)*(int*)value;

    HW_DRV_LOG_DEBUG("[%s], [%d]\n", __func__, __LINE__);

    return log_module_example_debug_mask;
}


static struct hw_device_debug new_module_example ={
    .device_name = "debug_mask_example",
    .category = HW_DEVICE_MODULE_LCD, /* LCD is one example */
    .hw_debug_mask_config = log_module_example_debug_mask_config,
    .log_level = HW_DEVICE_DEBUG_LEVEL_NONE,
};

static int __init hw_debug_log_init_module_example(void)
{
    log_module_example_debug_mask = HW_DEVICE_DEBUG_LEVEL_NONE;
    register_log_module(&new_module_example);
    return 0;
}

static void __exit hw_debug_log_exit_module_example(void)
{
    unregister_log_module(&new_module_example);
}



module_init(hw_debug_log_init_module_example);
module_exit(hw_debug_log_exit_module_example);

MODULE_DESCRIPTION("Huawei log control module example");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("huawei driver");
MODULE_VERSION("1.0");
