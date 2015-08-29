/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_memory.c
    
    @brief: 
    
    @version: 2.0 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2014-12-05
    
    @history:
**/

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/highmem.h>
#include <linux/version.h>
#include <asm/page.h>

#include "srecorder_misc.h"

static bool s_srecorder_has_contiguous_virt_addr = false;

/**
    @function: void srecorder_read_data_from_phys_page(char *pdst, unsigned long phys_src, size_t bytes_to_read)
    @brief: 
    @param: pdst 
    @param: phys_src 
    @param: bytes_to_read 
    @return: 
    @note: 
**/
int srecorder_read_data_from_phys_page(char *pdst, unsigned long phys_src, size_t bytes_to_read)
{
    int bytes_read_total = 0;
    
    if (unlikely(NULL == pdst))
    {
        return 0;
    }
    
    while (bytes_to_read > 0)
    {
        struct page *page = NULL;
        char *ptr = NULL;
        char *mapped_virt_addr = NULL;
        size_t bytes_read_this_time = 0;
        size_t bytes_left_in_this_phys_page = 0;
        
        page = pfn_to_page(phys_src >> PAGE_SHIFT);
        if (NULL == page) 
        {
            return bytes_read_total;
        }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
        ptr = kmap_atomic(page);
#else
        ptr = kmap_atomic(page, KM_USER0);
#endif
        if (unlikely(NULL == ptr))
        {
            return bytes_read_total;
        }
        
        mapped_virt_addr = ptr;
        ptr += phys_src & ~PAGE_MASK;
        bytes_left_in_this_phys_page = PAGE_SIZE - (phys_src & ~PAGE_MASK);
        bytes_read_this_time = MIN(bytes_left_in_this_phys_page,  bytes_to_read);
        memcpy(pdst, ptr, bytes_to_read);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
        kunmap_atomic(mapped_virt_addr);
#else
        kunmap_atomic(mapped_virt_addr, KM_USER0);
#endif
        flush_dcache_page(page);
        bytes_read_total += bytes_read_this_time;
        bytes_to_read -= bytes_read_this_time;
        phys_src += bytes_read_this_time;
        pdst += bytes_read_this_time;
    }

    return bytes_read_total;
}

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
    char *psrc, size_t bytes_to_write)
{
    int bytes_write_total = 0;
    
    if (unlikely(NULL == psrc))
    {
        return 0;
    }
    
    while ((bytes_to_write > 0) && (phys_mem_size > 0))
    {
        struct page *page = NULL;
        char *ptr = NULL;
        char *mapped_virt_addr = NULL;
        size_t bytes_write_this_time = 0;
        size_t bytes_left_in_this_phys_page = 0;
        
        page = pfn_to_page(phys_dst >> PAGE_SHIFT);
        if (unlikely(NULL == page))
        {
            return bytes_write_total;
        }
        
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
        ptr = kmap_atomic(page);
#else
        ptr = kmap_atomic(page, KM_USER0);
#endif
        if (unlikely(NULL == ptr))
        {
            return bytes_write_total;
        }
        mapped_virt_addr = ptr;
        ptr += phys_dst & ~PAGE_MASK;
        bytes_left_in_this_phys_page = MIN(phys_mem_size, (PAGE_SIZE - (phys_dst & ~PAGE_MASK)));
        bytes_write_this_time = MIN(bytes_left_in_this_phys_page,  bytes_to_write);
        memcpy(ptr, psrc, bytes_write_this_time);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
        kunmap_atomic(mapped_virt_addr);
#else
        kunmap_atomic(mapped_virt_addr, KM_USER0);
#endif

        bytes_write_total += bytes_write_this_time;
        bytes_to_write -= bytes_write_this_time;
        phys_dst += bytes_write_this_time;
        phys_mem_size -= bytes_write_this_time;
        psrc += bytes_write_this_time;
    }

    return bytes_write_total;
}

/**
    @function: void srecorder_release_virt_addr(psrecorder_virt_zone_info_t pzone_info)
    @brief: 
    @param: pzone_info 
    @return: none
    @note: 
**/
void srecorder_release_virt_addr(psrecorder_virt_zone_info_t pzone_info)
{
    int i = 0;
    
    if (unlikely(NULL == pzone_info || NULL == pzone_info->virt_start))
    {
        return;
    }
    
    for (i = 0; i < pzone_info->virt_page_count; i++)
    {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
        kunmap_atomic((char *)((unsigned long)(pzone_info->virt_start) + i * PAGE_SIZE));
#else
        kunmap_atomic((char *)((unsigned long)(pzone_info->virt_start) + i * PAGE_SIZE), KM_USER0);
#endif
    }
}

/**
    @function: bool srecorder_map_phys_addr(psrecorder_virt_zone_info_t pzone_info)
    @brief: 
    @param: pzone_info 
    @return: 
    @note: 
**/
bool srecorder_map_phys_addr(psrecorder_virt_zone_info_t pzone_info)
{
    int bytes_to_mapped = (int)pzone_info->size;
    int bytes_left_in_this_phys_page = 0;
    struct page *prev_page = NULL;
    struct page *page = NULL;
    char *prev_virt_addr = NULL;
    char *paddr = NULL;

    if (unlikely(NULL == pzone_info))
    {
        return false;
    }
    
    while (bytes_to_mapped > 0)
    {
        page = phys_to_page(pzone_info->phys_addr);
        if (unlikely(NULL == page))
        {
            break;
        }
        
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
        paddr = kmap_atomic(page);
#else
        paddr = kmap_atomic(page, KM_USER0);
#endif
        if (unlikely(NULL == paddr))
        {
            break;
        }
        
        if (NULL == pzone_info->virt_start) 
        {
            pzone_info->virt_start = paddr;
            pzone_info->ptr = pzone_info->virt_start + (pzone_info->phys_addr & ~PAGE_MASK);
            pzone_info->start_page = page;
        }
        bytes_left_in_this_phys_page = PAGE_SIZE - (pzone_info->phys_addr & ~PAGE_MASK);
        bytes_to_mapped -= bytes_left_in_this_phys_page;
        pzone_info->virt_page_count++;
        pzone_info->mapped_size += bytes_left_in_this_phys_page;
        
        if (NULL != prev_virt_addr)
        {
            if (PAGE_SIZE != (paddr - prev_virt_addr))
            {
                break;
            }
        }
        prev_virt_addr = paddr;
        
        if (NULL != prev_page)
        {
            pzone_info->page_delta = (unsigned long)page - (unsigned long)prev_page;
        }
        prev_page = page;
        
        pzone_info->phys_addr += bytes_left_in_this_phys_page;
    }

    return (bytes_to_mapped <= 0);
}

/**
    @function: void srecorder_set_contiguous_virt_addr_flag(void)
    @brief: 
    @param: flag 
    @return: none
    @note: 
**/
void srecorder_set_contiguous_virt_addr_flag(bool flag)
{
    s_srecorder_has_contiguous_virt_addr = flag;
}

/**
    @function: bool srecorder_has_contiguous_virt_addr(void)
    @brief: 
    @param: none 
    @return: 
    @note: 
**/
bool srecorder_has_contiguous_virt_addr(void)
{
    return s_srecorder_has_contiguous_virt_addr;
}
