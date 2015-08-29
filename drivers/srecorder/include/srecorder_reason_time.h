/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_crash_reason.h
    
    @brief: 
    
    @version: 2.0 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2014-12-05
    
    @history:
**/

#ifndef SRECORDER_REASON_TIME_H
#define SRECORDER_REASON_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "srecorder_log.h"

/**
    @function: void srecorder_enable_reason_time()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_reason_time(void);

/**
    @function: void srecorder_disable_reason_time()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_reason_time(void);

/**
    @function: int srecorder_dump_reason_time()
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_reason_time(void);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_REASON_TIME_H */
