/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_current_ps_backtrace.c
    
    @brief: 
    
    @version: 2.0 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2014-12-05
    
    @history:
**/

#include <linux/ioport.h>
#include <linux/bootmem.h>
#include <linux/vmalloc.h>

#define SRECORDER_IO_RESOURCE_NAME "srecorder"

#include "srecorder_log.h"
#include "srecorder_reason_time.h"
#include "srecorder_dmesg.h"
#include "srecorder_crc32.h"

struct resource res;

/**
    @function: void __init srecorder_reserve_special_mem(void)
    @brief: 
    @return: 
    @note: 
**/
void __init srecorder_reserve_special_mem(void)
{
    unsigned long phs_addr = CONFIG_SRECORDER_LOG_HEADER_H_PHYS_ADDR;
    unsigned phs_len = 4096;

    if (reserve_bootmem(phs_addr, phs_len, BOOTMEM_EXCLUSIVE) < 0)
    {
        SRECORDER_PRINTK("Cannot reserve bootmemory for the start-up header.\n");
        return;
    }

    res.name  = SRECORDER_IO_RESOURCE_NAME;
    res.flags = IORESOURCE_BUSY | IORESOURCE_MEM;
    res.start = phs_addr;
    res.end = phs_addr + phs_len - 1;
    insert_resource(&iomem_resource, &res);

    return;
}

/**
    @function: void srecorder_move_previous_log(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_move_previous_log(void)
{
    unsigned long addr;
    log_header_m_t* p_log_header_m_temp = (log_header_m_t *)__va(CONFIG_SRECORDER_LOG_HEADER_M_PHYS_ADDR);
    log_header_m_t* p_log_header_m_prev = srecorder_get_log_header_m(IDX_PREV);

    SRECORDER_INFO("Temp log header fields: %x %x %lx %lx %x %x %x %x\n", 
        p_log_header_m_temp->version, p_log_header_m_temp->magic_num, 
        p_log_header_m_temp->log_buf.pa, p_log_header_m_temp->log_buf.va, 
        p_log_header_m_temp->log_len, p_log_header_m_temp->log_buf_len, p_log_header_m_temp->crc32,
        srecorder_calculate_crc32((unsigned char const *)p_log_header_m_temp, sizeof(log_header_m_t) - sizeof(unsigned)));

    if (srecorder_verify_log_header_m(p_log_header_m_temp, true))
    {
        SRECORDER_PRINTK("Temp log header is messed up.\n");
        return;
    }

    /* start to update p_log_header_m_prev */
    memset(p_log_header_m_prev, 0, sizeof(log_header_m_t));

    /* header verification functions ensure the length isn't zero */
    addr = (unsigned long)vmalloc(p_log_header_m_temp->log_len);
    if (unlikely(NULL == (void*)addr))
    {
        SRECORDER_PRINTK("vmalloc failed for temp log buffer.\n");
        return;
    }

    /* initialize previous settings in log_header_m_src */
    p_log_header_m_prev->version = CONFIG_SRECORDER_VERSION;
    p_log_header_m_prev->magic_num = SRECORDER_MAGIC_NUMBER;
    p_log_header_m_prev->log_buf.pa = 0;
    p_log_header_m_prev->log_buf.va = addr;
    p_log_header_m_prev->log_len = p_log_header_m_temp->log_len;
    p_log_header_m_prev->log_buf_len = p_log_header_m_temp->log_len;
    srecorder_update_log_header_m(p_log_header_m_prev);

    /* now vms is ready, we can use vitrual addresses */
    memset((void*)(p_log_header_m_prev->log_buf.va), 0, p_log_header_m_temp->log_len);
    memcpy((void*)(p_log_header_m_prev->log_buf.va), (void*)(__va(p_log_header_m_temp->log_buf.pa + sizeof(log_header_m_t))), p_log_header_m_temp->log_len);

    return;
}
