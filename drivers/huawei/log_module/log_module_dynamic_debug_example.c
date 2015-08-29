/*
 * log_module_dynamic_debug_example.c
 * just example code
 *
 *  Copyright (C) 2012-2014  Huawei Corporation
 */

#include <linux/log_module.h>
#include <linux/module.h>
#include <linux/moduleparam.h>


int log_module_example_dynamic_debug_config(void * value)
{
    // TODO: find better way to orgnize the file list.
    return -ENOTSUPP;
}


static struct hw_device_debug new_module_dynamic_debug_example ={
    .device_name = "dynamic_debug_example",
    .category = HW_DEVICE_MODULE_CAMERA, /* camera is one example */
    .hw_dynamic_debug_config = log_module_example_dynamic_debug_config,
    .log_level = HW_DEVICE_DEBUG_LEVEL_NONE,
};

static int __init hw_debug_log_init_module_example(void)
{
    register_log_module(&new_module_dynamic_debug_example);
    return 0;
}

static void __exit hw_debug_log_exit_module_example(void)
{
    unregister_log_module(&new_module_dynamic_debug_example);
}



module_init(hw_debug_log_init_module_example);
module_exit(hw_debug_log_exit_module_example);

MODULE_DESCRIPTION("Huawei log control module dynamic debug example");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("huawei driver");
MODULE_VERSION("1.0");
