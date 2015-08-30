
#ifndef _HW_AUDIO_INFO_H
#define _HW_AUDIO_INFO_H
#include <linux/dsm_pub.h>


#define DSM_AUDIO_BUF_SIZE           (1024)   /*Byte*/
#define DSM_REPORT_DELAY_TIME        (20)

#define DSM_AUDIO_MOD_NAME           "dsm_audio_info"

int audio_dsm_register(void);
int audio_dsm_report_num(int error_no, unsigned int mesg_no);
int audio_dsm_report_info(int error_no, void *log, int size);


/* DSM audio error message level */
#define DSM_AUDIO_MESG_LEVEL_ERROR           (1)
#define DSM_AUDIO_MESG_LEVEL_INFO            (2)

/* DSM audio error message code */

#define DSM_AUDIO_MESG_GET_PINCRTL_FAIL    ((DSM_AUDIO_MESG_LEVEL_ERROR<<24) + 1)
#define DSM_AUDIO_MESG_MACH_PROBE_FAIL     ((DSM_AUDIO_MESG_LEVEL_ERROR<<24) + 2)
#define DSM_AUDIO_MESG_MEDOM_LOAD_FAIL     ((DSM_AUDIO_MESG_LEVEL_ERROR<<24) + 3)
#define DSM_AUDIO_MESG_SPMI_GET_FAIL       ((DSM_AUDIO_MESG_LEVEL_ERROR<<24) + 4)
#define DSM_AUDIO_MESG_SPMI_PROBE_FAIL     ((DSM_AUDIO_MESG_LEVEL_ERROR<<24) + 5)
#define DSM_AUDIO_MESG_ALLOC_MEM_FAIL      ((DSM_AUDIO_MESG_LEVEL_ERROR<<24) + 6)
#define DSM_AUDIO_MESG_IOREMAP_FAIL        ((DSM_AUDIO_MESG_LEVEL_ERROR<<24) + 7)
#define DSM_AUDIO_MESG_MEDOM_REGIST_FAIL   ((DSM_AUDIO_MESG_LEVEL_ERROR<<24) + 8)
#define DSM_AUDIO_MESG_HS_TYPE_DECT_FAIL   ((DSM_AUDIO_MESG_LEVEL_ERROR<<24) + 9)
#endif
