#ifndef __HUAWEI_APANIC_H__
#define __HUAWEI_APANIC_H__

//#include <mach/msm_iomap.h>
#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/of.h>

#define HUAWEI_PANIC_TAG "[HUAWEI_PANIC]"

#define CRASH_LOG_MAGIC_NUM               (0xCACADEAD)
//#define CRASH_LOG_MAGIC_NUM_ADDR          (MSM_IMEM_BASE + 0xB18)
//#define CRASH_LOG_LENGTH_ADDR             (MSM_IMEM_BASE + 0xB1C)

extern void *hw_reset_magic_addr;
#define HW_RESET_LOG_MAGIC_NUM               (0xCACADEAD)
//#define HW_RESET_LOG_MAGIC_NUM_ADDR          (MSM_IMEM_BASE + 0xB20)
#define HW_RESET_LOG_MAGIC_NUM_LEN   (4)

#define DEFAULT_PANIC_LOG_LEN             (1024)
#define MAX_LOG_BUF_LENGTH                (0x800000) /* 8M */

#ifdef CONFIG_PRINTK
void* huawei_get_log_buf_addr(void);
int huawei_get_log_buf_len(void);
#endif
void clear_hw_reset(void);
#endif
