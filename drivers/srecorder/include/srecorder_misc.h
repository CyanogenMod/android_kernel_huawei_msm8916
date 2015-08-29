/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_misc.h
    
    @brief: 
    
    @version: 2.0 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2014-12-05
    
    @history:
*/

#ifndef SRECORDER_MISC_H
#define SRECORDER_MISC_H

#include <linux/spinlock.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INVALID_KSYM_ADDR (0UL) 
#define DMESG_MAX_LENGTH (0x20000) /*128KB*/

#ifdef CONFIG_DEBUG_SRECORDER
#define DEBUG_SRECORDER (CONFIG_DEBUG_SRECORDER) 
#else
#define DEBUG_SRECORDER 1 
#endif

#define DEBUG_KERNEL_SYMBOLS 1
#define DEBUG_CRASH_TIME 1
#define DEBUG_SYS_INFO 1
#define DEBUG_DMESG 1
#define DEBUG_ALLCPU_STACK 1
#define DEBUG_ALLPS_INFO 1
#define DEBUG_CURRENT_PS_BACKTRACE 1
#define DEBUG_SLAB_INFO 1

#define K(x) ((x) << (PAGE_SHIFT - 10)) 
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
#define MIN(a, b) ((a) > (b)) ? (b) : (a)
#define LOG_BUF_MASK(log_buf_len) ((log_buf_len) - 1)

/* log_buf_len must equal 2 ^ n ((n >= 0) && (n <= 32)) or the result may be wrong*/
#define LOG_OFFSET(len, log_buf_len) ((len) & LOG_BUF_MASK(log_buf_len))

typedef unsigned long srec_ul32;
typedef unsigned long long srec_ul64;
typedef srec_ul32 srec_ksym_addr_t;
typedef srec_ul32 srec_reserved_mem_t;

typedef struct __srecorder_virt_zone_info_t
{
    struct page *start_page;
    char *virt_start;
    char *ptr;
    unsigned long phys_addr;
    unsigned long size;
    unsigned long page_delta;
    unsigned long virt_page_count;
    int mapped_size;
} srecorder_virt_zone_info_t, *psrecorder_virt_zone_info_t;

/**
    @function: int srecorder_write_data_by_page(unsigned long phys_dst, size_t phys_mem_size, 
        char *psrc, size_t bytes_to_write)
    @brief: 
    @param: phys_dst 
    @param: phys_mem_size 
    @param: psrc 
    @param: bytes_to_write 
    @return: 
    @note: 
**/
int srecorder_write_data_to_phys_page(unsigned long phys_dst, size_t phys_mem_size, 
    char *psrc, size_t bytes_to_write);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_MISC_H */
