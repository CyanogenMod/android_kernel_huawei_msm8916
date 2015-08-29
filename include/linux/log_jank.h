/*
 *  Copyright (C) 2014 Huawei Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_LOG_JANK_H__
#define __LINUX_LOG_JANK_H__

#include <linux/janklogconstants.h>
#include <linux/sched.h>
#include <linux/jankguardconstants.h>

#define LOG_JANK_FS "/dev/log/jank"
#define JANK_LOG_PRIO_VERBOSE  (2)
#define JANK_LOG_PRIO_DEBUG (3)
#define JANK_LOG_PRIO_INFO (4)
#define JANK_LOG_PRIO_WARN (5)
#define JANK_LOG_PRIO_ERROR (6)

#if defined (CONFIG_LOG_JANK)
static inline unsigned long getrealtime(void)
{
    unsigned long long t;
    int this_cpu;
    this_cpu = raw_smp_processor_id();
    t = cpu_clock(this_cpu);
    do_div(t,1000000);
    return (unsigned long) t;
}
int log_to_jank(char* tag, int prio,const char* fmt, ...);
//for the forward compatibility ,JANK_LOG_PRIO_DEBUG level is just for jank service.
//and the interface name stay the same "pr_jank"
//se LOG_JANK_W LOG_JANK_V  LOG_JANK_I   LOG_JANK_E  for other purpose
#ifndef pr_jank
#define pr_jank(tag, fmt, ...) (log_to_jank(tag,JANK_LOG_PRIO_DEBUG,fmt,##__VA_ARGS__))
#endif
#ifndef LOG_JANK_W
#define LOG_JANK_W(tag, fmt, ...) (log_to_jank(tag,JANK_LOG_PRIO_WARN,fmt,##__VA_ARGS__))
#endif
#ifndef LOG_JANK_V
#define LOG_JANK_V(tag, fmt, ...) (log_to_jank(tag,JANK_LOG_PRIO_VERBOSE,fmt,##__VA_ARGS__))
#endif
#ifndef LOG_JANK_I
#define LOG_JANK_I(tag, fmt, ...) (log_to_jank(tag,JANK_LOG_PRIO_INFO,fmt,##__VA_ARGS__))
#endif
#ifndef LOG_JANK_E
#define LOG_JANK_E(tag, fmt, ...) (log_to_jank(tag,JANK_LOG_PRIO_ERROR,fmt,##__VA_ARGS__))
#endif
#else
static inline unsigned long getrealtime(void)
{
    return 0;
}
#define pr_jank(tag, fmt, ...)
#define LOG_JANK_W(tag, fmt, ...)
#define LOG_JANK_V(tag, fmt, ...)
#define LOG_JANK_I(tag, fmt, ...)
#define LOG_JANK_E(tag, fmt, ...)
#endif

#endif
