/*
 * log_module.h
 *
 * provide interface to other module to enable/disable log system.
 *
 *  Copyright (C) 2012-2014  Huawei Corporation
 */

#ifndef __LOG_MODULE_H
#define __LOG_MODULE_H

#include <linux/list.h>

typedef int (*debug_mask_config)(void * value);

typedef int (*dynamic_debug_config)(void * value);

 /* log level
  * 1, follow the kernel log level
  * 2, add reg and key level to do some special log
  * 3, NONE means no log output
  * 4, User need to use bit to make different log level(optimized)
  */
typedef enum _device_log_level{
    HW_DEVICE_DEBUG_LEVEL_NONE = 0,
    HW_DEVICE_DEBUG_LEVEL_EMERG = 1U << 0,
    HW_DEVICE_DEBUG_LEVEL_ALERT = 1U << 1,
    HW_DEVICE_DEBUG_LEVEL_CRIT = 1U << 2,
    HW_DEVICE_DEBUG_LEVEL_ERR = 1U << 3,
    HW_DEVICE_DEBUG_LEVEL_WARNING = 1U << 4,
    HW_DEVICE_DEBUG_LEVEL_NOTICE = 1U << 5,
    HW_DEVICE_DEBUG_LEVEL_INFO = 1U << 6,
    HW_DEVICE_DEBUG_LEVEL_DEBUG = 1U << 7,
    HW_DEVICE_DEBUG_LEVEL_REG = 1U << 8,
    HW_DEVICE_DEBUG_LEVEL_KEY = 1U << 9,  /* seldom use,depends on useage */
    HW_DEVICE_DEBUG_LEVEL_ALL = 0xFFFF,
}device_log_level;

/* module category */
typedef enum _module_category{
    HW_DEVICE_MODULE_INVALID,
    HW_DEVICE_MODULE_LCD,
    HW_DEVICE_MODULE_CAMERA,
    HW_DEVICE_MODULE_TP,
    HW_DEVICE_MODULE_SENSOR,
    HW_DEVICE_MODULE_LED,
    HW_DEVICE_MODULE_KEYPAD,
    HW_DEVICE_MODULE_BT,
    HW_DEVICE_MODULE_WIFI,
    HW_DEVICE_MODULE_NFC,
    HW_DEVICE_MODULE_ALL
} module_category;

#define MAX_MODULE_NAME_LEN 32

struct hw_device_debug{
    struct list_head list;
    device_log_level  log_level;
    char device_name[MAX_MODULE_NAME_LEN]; /* driver don't provide space to save name */
    module_category category;    /* module category */
    long id_num;    /* to diff from dynamic module which has same device_name */
    debug_mask_config hw_debug_mask_config;
    dynamic_debug_config hw_dynamic_debug_config;
};

#define LOG_MODULE_PRIVATE_BASE 0x8888
#define LOG_M_SET_DEVICE_LOG   _IOW(LOG_MODULE_PRIVATE_BASE, 1, struct hw_device_debug *)
#define LOG_M_GET_MODULE_NUM    _IOR(LOG_MODULE_PRIVATE_BASE, 2, int)
#define LOG_M_GET_MODULE    _IOWR(LOG_MODULE_PRIVATE_BASE, 3,  struct hw_device_debug *)
#define LOG_M_GET_MODULE_HEAD   _IOWR(LOG_MODULE_PRIVATE_BASE, 4,  struct hw_device_debug *)

/* unify the interface and the MACRO for each module */
#ifdef CONFIG_HUAWEI_LOG_MODULE
int register_log_module(struct hw_device_debug * new_module);
void unregister_log_module(struct hw_device_debug *new_module);

#else
static inline int register_log_module(struct hw_device_debug * new_module)
{
    return 0;
}
static inline void unregister_log_module(struct hw_device_debug *new_module)
{
}
#endif /* CONFIG_HUAWEI_LOG_MODULE */


/* user this to define the debug mask */
#define DEFINE_LOG_MASK(_mask)   \
                            static int _mask;\
                            static int *p_define_log_mask=&_mask;

#define HW_DRV_LOG_COMMON(level, mask, ...)   \
                            do{    \
                                if(level <= mask)     \
                                {    \
                                    printk(__VA_ARGS__);   \
                                }   \
                            }while(0);


#ifdef CONFIG_HUAWEI_LOG_MODULE
/* user this to print the log */
#define HW_DRV_LOG_EMERG(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_EMERG, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_ALERT(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_ALERT, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_CRIT(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_CRIT, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_ERR(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_ERR, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_WARNING(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_WARNING, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_NOTICE(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_NOTICE, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_INFO(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_INFO, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_DEBUG(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_DEBUG, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_FLOW(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_DEBUG, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_REG(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_REG, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_KEY(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_KEY, * p_define_log_mask, __VA_ARGS__)
#define HW_DRV_LOG_ALL(...)    HW_DRV_LOG_COMMON(HW_DEVICE_DEBUG_LEVEL_ALL, * p_define_log_mask, __VA_ARGS__)
#else

#define HW_DRV_LOG_EMERG(...)
#define HW_DRV_LOG_ALERT(...)
#define HW_DRV_LOG_CRIT(...)
#define HW_DRV_LOG_ERR(...)
#define HW_DRV_LOG_WARNING(...)
#define HW_DRV_LOG_NOTICE(...)
#define HW_DRV_LOG_INFO(...)
#define HW_DRV_LOG_DEBUG(...)
#define HW_DRV_LOG_FLOW(...)
#define HW_DRV_LOG_REG(...)
#define HW_DRV_LOG_KEY(...)
#define HW_DRV_LOG_ALL(...)

#endif /* CONFIG_HUAWEI_LOG_MODULE */

#endif
