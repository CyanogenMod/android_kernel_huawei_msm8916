/************************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  FileName: sensor_otp_s5k5e2.c
  Version :Initial Draft
  Date: 2014/10/11
  Description:    this file contion several functions to detect otp_s5k5e2 properties
  Version:         Initial Draft
  History:
   History        :
   1.Date        : 2014/10/11
   Modification : Created function
***********************************************************/
#define HW_CMR_LOG_TAG "sensor_otp_s5k5e2"
#include <media/msm_cam_sensor_cherry.h>
#include "msm_cci.h"
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

#ifndef true
	#define true  1
#endif

#ifndef false
	#define false 0
#endif

#define S5K5E2_OTP_ERROR          (-1)
#define S5K5E2_OTP_OK              0 

#define S5K5E2_GROUP_DATA_EMPTY    0
#define S5K5E2_GROUP_DATA_INVALID  1
#define S5K5E2_GROUP_DATA_VALID    2

#define S5K5E2_OTP_GROUP1          1
#define S5K5E2_OTP_GROUP2          2
#define S5K5E2_OTP_GROUP3          3
#define S5K5E2_OTP_GROUPMAX        4
           
#define S5K5E2_OTP_ID_READ_FLAG      (1 << 0)
#define S5K5E2_OTP_AWB_READ_FLAG     (1 << 1)
#define S5K5E2_OTP_LSC_READ_FLAG     (1 << 2)
#define S5K5E2_OTP_VCM_READ_FLAG     (1 << 3)
#define S5K5E2_OTP_FF_READ_ALL_FLAG  (S5K5E2_OTP_ID_READ_FLAG|S5K5E2_OTP_AWB_READ_FLAG)
#define S5K5E2_OTP_AF_READ_ALL_FLAG  (S5K5E2_OTP_FF_READ_ALL_FLAG|S5K5E2_OTP_VCM_READ_FLAG)
#define OTP_INIT_STATE                0x04
#define OTP_READ_ENABLE               0x01	
#define OTP_READ_DISABLE              0x00   

#define OPT_LSC_MAX_LENGTH            110

/* S5K5E2 OTP REG */
#define S5K5E2_OTP_INIT_STATE_REG             0x0A00
#define S5K5E2_OTP_PAGE_NUM_REG               0x0A02
#define S5K5E2_OTP_READ_MODE_REG              0x0A00

#define S5K5E2_OTP_ID_FLAG_REG                0x0A04  
#define S5K5E2_OTP_ID_GROUP1_BASE_REG         0x0A05
#define S5K5E2_OTP_ID_GROUP2_BASE_REG         0x0A0A
#define S5K5E2_OTP_ID_GROUP3_BASE_REG         0x0A0F

#define S5K5E2_OTP_AWB_FLAG_REG	           	  0x0A13
#define S5K5E2_OTP_AWB_GROUP1_BASE_REG        0x0A14
#define S5K5E2_OTP_AWB_GROUP2_BASE_REG        0x0A1A
#define S5K5E2_OTP_AWB_GROUP3_BASE_REG        0x0A20

#define S5K5E2_OTP_VCM_FLAG_REG	           	  0x0A04
#define S5K5E2_OTP_VCM_GROUP1_BASE_REG        0x0A05
#define S5K5E2_OTP_VCM_GROUP2_BASE_REG        0x0A09
#define S5K5E2_OTP_VCM_GROUP3_BASE_REG        0x0A0D

#define S5K5E2_OTP_RG_H_REG                   0x0210
#define S5K5E2_OTP_RG_L_REG                   0x0211
#define S5K5E2_OTP_BG_H_REG                   0x0212
#define S5K5E2_OTP_BG_L_REG                   0x0213
#define S5K5E2_OTP_GB_H_REG                   0x0214
#define S5K5E2_OTP_GB_L_REG                   0x0215
#define S5K5E2_OTP_GR_H_REG                   0x020E
#define S5K5E2_OTP_GR_L_REG                   0x020F

#define S5K5E2_OTP_LSC_FLAG1_REG              0x0a04
#define S5K5E2_OTP_LSC_FLAG2_REG              0x0a05
#define S5K5E2_OTP_LSC_FLAG3_REG              0x0a06

#define OTP_ID_PAGE_NUM                       0x02
#define OTP_VCM_PAGE_NUM                      0x04                                                    
#define OTP_AWB_PAGE_NUM                      0x05
#define OTP_LSC_FLAG_PAGE_NUM                 0x05

#define ULC2_HUAWEI_CAMERA_FF_NUM             0xB2  /* 23060178  */
#define ULC2_HUAWEI_CAMERA_AF_NUM             0xB3  /* 23060179  */
#define S5K5E2_OTP_VCM_OFFSET_VALUE           100

typedef enum {
	SUNNY_MODULE_VENDOR_ID = 1,
	FOXCONN_MODULE_VENDOR_ID,
	LITEON_MODULE_VENDOR_ID,
	SEMCO_MODULE_VENDOR_ID,
	BYD_MODULE_VENDOR_ID,
	OFILM_MODULE_VENDOR_ID
}camera_module_vendor_id;

struct S5K5E2_OTP_STRUCT {
    uint16_t year;
    uint16_t month;
    uint16_t day;
    uint16_t camera_id;      
    uint16_t vendor_id;  
    uint16_t wb_rg_h; 
    uint16_t wb_rg_l; 
    uint16_t wb_bg_h; 
    uint16_t wb_bg_l; 
    uint16_t wb_gbgr_h; 
    uint16_t wb_gbgr_l; 
    uint16_t lenc[OPT_LSC_MAX_LENGTH];
    uint16_t VCM_start;
    uint16_t VCM_max;
};

// otp params global varible
struct S5K5E2_OTP_STRUCT g_cur_opt_params = { 0 };

static uint8_t  s5k5e2_otp_read_flag = 0;
static bool  s5k5e2_otp_is_af_module = false;          /* AF = true, FF = false */

static uint32_t rg_ratio_typical  = 0x035c;  // the average of 4 Golden samples' RG ratio
static uint32_t bg_ratio_typical  = 0x02b8;  // the average of 4 Golden samples' BG ratio

/****************************************************************************
* FunctionName: s5k5e2_ofilm_ohw5f02_cci_i2c_write;
* Description : i2c write interface;
***************************************************************************/
static int32_t s5k5e2_ofilm_ohw5f02_cci_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, u16 data)
{
	int32_t rc = -EFAULT;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);

	if (rc < 0)
	{
		pr_err("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;
}

/****************************************************************************
* FunctionName: s5k5e2_ofilm_ohw5f02_cci_i2c_read;
* Description : i2c read interface;
***************************************************************************/
static int32_t s5k5e2_ofilm_ohw5f02_cci_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t *data)
{
	int32_t rc = -EFAULT;
	uint16_t temp_data = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			addr,
			&temp_data, MSM_CAMERA_I2C_BYTE_DATA);

	if (rc < 0)
	{
		pr_err("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, temp_data);
	}

	*data = temp_data;

	return rc;
}

/****************************************************************************
* FunctionName: s5k5e2_read_otp;
* Description : i2c api used to read information from eeprom.
* Input       : NA;
* Output      : s5k5e2_otp;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static bool s5k5e2_read_otp(struct msm_sensor_ctrl_t *s_ctrl,uint16_t reg,uint8_t *buf,uint16_t count)
{
	u16 i   = 0;
	int ret = 0;
	u16 val = 0;

	for (i=0; i<count; i++)
	{
		ret =s5k5e2_ofilm_ohw5f02_cci_i2c_read(s_ctrl,(reg+i) ,&val);
		
		if (ret !=0)
		{
			pr_err("%s fail to read otp with error code %d, reg_addr=0x%x\n", __func__,ret,reg+i);
			return false;
		}
		
		buf[i] = (val&0xff);
	}
     
	return true;
}

/****************************************************************************
* FunctionName: s5k5e2_otp_read_page_data_enable;
* Description : make initial state and set read mode of NVM controller interface1.
* Input       : page_num;
* Output      : NONEl;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static void s5k5e2_otp_read_page_data_enable(struct msm_sensor_ctrl_t * s_ctrl,  uint16_t page_num)
{
	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_INIT_STATE_REG,OTP_INIT_STATE);

	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_PAGE_NUM_REG,page_num); 

	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_READ_MODE_REG,OTP_READ_ENABLE); 
	
    /* to wait Tmin(47us the time to transfer 1 page data from OTP to buffer) */
	udelay(150); 
	
	return;
}

/****************************************************************************
* FunctionName: s5k5e2_otp_read_page_data_disable;
* Description : make initial state and disable NVM controller.
* Input       : NONEl;
* Output      : NONEl;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static void s5k5e2_otp_read_page_data_disable(struct msm_sensor_ctrl_t * s_ctrl)
{
	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_INIT_STATE_REG,OTP_INIT_STATE);
	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_READ_MODE_REG,OTP_READ_DISABLE); 
	return;
}

/******************************************************************************
  Function:    select_otp_group                
  Description: select the otp group  
  Input:       index
               flag
  Output:      reutrn value
  return:      KAL_FALSE  --select fail
               GROUP_DATA_EMPTY 0, group index is empty
               GROUP_DATA_INVALID 1, group index has invalid data
               GROUP_DATA_VALID 2, group index has valid data
  Others:      NULL  
*******************************************************************************/
static int s5k5e2_select_otp_group(struct msm_sensor_ctrl_t * s_ctrl, uint8_t *index, uint8_t flag)
{
	int ret = 0xFF;
	uint8_t group_num = 0;
	uint8_t group_flag  = 0;
	uint8_t avalid_data_flag = 0;
	
	*index = S5K5E2_OTP_GROUPMAX;
	for (group_num=S5K5E2_OTP_GROUP1; group_num<=S5K5E2_OTP_GROUP3; group_num++)
	{
		group_flag = (flag>>(2 * (S5K5E2_OTP_GROUP3 - group_num + 1))) & 0x03;
		pr_info("[select_otp_group]: group_num=%d;flag=%d.\n ", group_num, group_flag);
		if (0x01 == group_flag)
		{
			(*index) = group_num;
			avalid_data_flag++;
		}
	}

	if (1 == avalid_data_flag)
	{
		ret = S5K5E2_GROUP_DATA_VALID;
	}
	else
	{
		ret = S5K5E2_GROUP_DATA_INVALID;
	}
	
	pr_info("[select_otp_group]: *index=%d; ret=%d.\n ", *index, ret);

	return ret;

}

/******************************************************************************
  Function:    s5k5e2_otp_read_id_group_reg_data                
  Description: get the group reg data.
*******************************************************************************/
static bool s5k5e2_otp_read_id_group_reg_data(struct msm_sensor_ctrl_t *s_ctrl, struct S5K5E2_OTP_STRUCT *otp_ptr)
{
	uint8_t buf[5] = { 0 };
	uint8_t module_id_flag = 0;
	uint8_t otp_group_num= 0;
	uint16_t address_start = 0; 
	int tmpret = 0;
		
	s5k5e2_read_otp(s_ctrl,S5K5E2_OTP_ID_FLAG_REG,&module_id_flag,1);
	
       tmpret = s5k5e2_select_otp_group(s_ctrl,&otp_group_num,module_id_flag);

	if (S5K5E2_GROUP_DATA_VALID != tmpret)
	{
		pr_err("%s s5k5e2_select otp group false.\n",__func__);
		return false;
	}

	switch (otp_group_num)
	{
		case S5K5E2_OTP_GROUP1:
		{      
			address_start = S5K5E2_OTP_ID_GROUP1_BASE_REG;
			break;
		}
		case S5K5E2_OTP_GROUP2:
		{
			address_start = S5K5E2_OTP_ID_GROUP2_BASE_REG;
			break;
		}
		case S5K5E2_OTP_GROUP3:
		{
			address_start = S5K5E2_OTP_ID_GROUP3_BASE_REG;
			break;
		}
		default:
		{
			pr_err("[s5k5e2_otp_read_id]:Error: otp_group_num %d is beyond the limit of the 3 group!\n", otp_group_num);
			break;
		}	
	}

	s5k5e2_read_otp(s_ctrl,address_start,buf,5);

	otp_ptr->year = buf[0];
	otp_ptr->month = buf[1];
	otp_ptr->day = buf[2];
	otp_ptr->camera_id = buf[3];
	otp_ptr->vendor_id = buf[4];
	
	return true;
}

/******************************************************************************
  Function:    s5k5e2_otp_read_awb_group_reg_data                
  Description: get the group reg data.
*******************************************************************************/
static bool s5k5e2_otp_read_awb_group_reg_data(struct msm_sensor_ctrl_t *s_ctrl, struct S5K5E2_OTP_STRUCT *otp_ptr)
{
	uint8_t buf[6] = { 0 };
	uint8_t module_id_flag = 0;
	uint8_t otp_group_num= 0;
	uint16_t address_start = 0; 
	uint32_t  rg = 0;
	uint32_t  bg = 0;
	int tmpret = 0;
		
	s5k5e2_read_otp(s_ctrl,S5K5E2_OTP_AWB_FLAG_REG,&module_id_flag,1);
	
	tmpret = s5k5e2_select_otp_group(s_ctrl,&otp_group_num,module_id_flag);

	if (S5K5E2_GROUP_DATA_VALID != tmpret)
	{
		pr_err("%s s5k5e2_select otp group false.\n",__func__);
		return false;
	}

	switch (otp_group_num)
	{
		case S5K5E2_OTP_GROUP1:
		{      
			address_start = S5K5E2_OTP_AWB_GROUP1_BASE_REG;
			break;
		}
		case S5K5E2_OTP_GROUP2:
		{
			address_start = S5K5E2_OTP_AWB_GROUP2_BASE_REG;
			break;
		}
		case S5K5E2_OTP_GROUP3:
		{
			address_start = S5K5E2_OTP_AWB_GROUP3_BASE_REG;
			break;
		}
		default:
		{
			pr_err("[s5k5e2_otp_read_id]:Error: otp_group_num %d is beyond the limit of the 3 group!\n", otp_group_num);
			break;
		}	
	}

	s5k5e2_read_otp(s_ctrl,address_start,buf,6);

	otp_ptr->wb_rg_h = buf[0];
	otp_ptr->wb_rg_l = buf[1];
	otp_ptr->wb_bg_h = buf[2]; 
	otp_ptr->wb_bg_l = buf[3];
	otp_ptr->wb_gbgr_h = buf[4];
	otp_ptr->wb_gbgr_l = buf[5];
	
	rg = ((otp_ptr->wb_rg_h<<8) | (otp_ptr->wb_rg_l));
	bg = ((otp_ptr->wb_bg_h<<8) | (otp_ptr->wb_bg_l));

	if ((0 == rg) || (0 == bg))
	{
		pr_err("%s error: rg || bg is zero.\n",__func__);
		return false;
	}
	return true;
}
/******************************************************************************
  Function:    s5k5e2_otp_read_awb_group_reg_data                
  Description: get the group reg data.
*******************************************************************************/
static bool s5k5e2_otp_read_vcm_group_reg_data(struct msm_sensor_ctrl_t *s_ctrl, struct S5K5E2_OTP_STRUCT *otp_ptr)
{
	uint8_t buf[4] = { 0 };
	uint8_t module_id_flag = 0;
	uint8_t otp_group_num= 0; 
	uint16_t address_start = 0; 
	int tmpret = 0;
		
	s5k5e2_read_otp(s_ctrl,S5K5E2_OTP_VCM_FLAG_REG,&module_id_flag,1);
	
	pr_info("%s,module_id_flag = 0x%x\n",__func__,module_id_flag);
	tmpret = s5k5e2_select_otp_group(s_ctrl,&otp_group_num,module_id_flag);

	if (S5K5E2_GROUP_DATA_VALID != tmpret)
	{
		pr_err("%s s5k5e2_select otp group false.\n",__func__);
		return false;
	}

	switch (otp_group_num) 
	{
		case S5K5E2_OTP_GROUP1:
		{      
			address_start = S5K5E2_OTP_VCM_GROUP1_BASE_REG;
			break;
		}
		case S5K5E2_OTP_GROUP2:
		{
			address_start = S5K5E2_OTP_VCM_GROUP2_BASE_REG;
			break;
		}
		case S5K5E2_OTP_GROUP3:
		{
			address_start = S5K5E2_OTP_VCM_GROUP3_BASE_REG;
			break;
		}
		default:
		{
			pr_err("[s5k5e2_otp_read_vcm]:Error: otp_group_num %d is beyond the limit of the 3 group!\n", otp_group_num);
			break;
		}	
	}

	s5k5e2_read_otp(s_ctrl,address_start,buf,4);
	
	otp_ptr->VCM_start = ((buf[0] << 2) | ((buf[1] >> 6) & 0x03));
	otp_ptr->VCM_max =  ((buf[2] << 2) | ((buf[3] >> 6) & 0x03));

	if ((0 == otp_ptr->VCM_start) || (0 == otp_ptr->VCM_max) || (otp_ptr->VCM_max <= otp_ptr->VCM_start))
	{
		pr_err("VCM_start-MSB = 0x%x\n",(buf[0] << 2));
		pr_err("VCM_start-LSB = 0x%x\n",((buf[1]>>6) & 0x03));
		pr_err("VCM_MAX-MSB = 0x%x\n",(buf[2] << 2));
		pr_err("VCM_MAX-LSB  = 0x%x\n",((buf[3]>>6) & 0x03));
		pr_err("VCM_START = 0x%x\n",otp_ptr->VCM_start);
		pr_err("VCM_MAX  = 0x%x\n",otp_ptr->VCM_max);
		
		return false;
	}
	
	if (0 > (int)(otp_ptr->VCM_start - S5K5E2_OTP_VCM_OFFSET_VALUE))
	{
		pr_err("%s, otp_ptr->VCM_start = 0x%x\n", __func__,otp_ptr->VCM_start);
		otp_ptr->VCM_start = 0;
	}
	else
	{
		otp_ptr->VCM_start -= S5K5E2_OTP_VCM_OFFSET_VALUE;
	}
			
	otp_ptr->VCM_max += S5K5E2_OTP_VCM_OFFSET_VALUE;

	return true;
}

/****************************************************************************
* FunctionName: s5k5e2_otp_read_id;
* Description : read the otp module id data.
* Input       : NONEl;
* Output      : NONEl;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static bool s5k5e2_otp_read_id(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *otp_ptr)
{ 	 
	bool ret = false;
	
	s5k5e2_otp_read_page_data_enable(s_ctrl,OTP_ID_PAGE_NUM);
	ret = s5k5e2_otp_read_id_group_reg_data(s_ctrl,otp_ptr);
	s5k5e2_otp_read_page_data_disable(s_ctrl);
       
	return ret;
}

/****************************************************************************
* FunctionName: s5k5e2_otp_check_id;
* Description : check the s5k5e2 module id data.
* Input       : NONEl;
* Output      : NONEl;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static bool s5k5e2_otp_check_id(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *current_otp)
{
	bool ret = false;
	uint8_t vendor_id = 0;

	
	if (true == s5k5e2_otp_read_id(s_ctrl,current_otp))
	{
	    pr_info("%s,s5k5e2_otp_read_id ok!\n",__func__);
		vendor_id = ((current_otp->vendor_id)>>4) & 0x0F;
		
		if ((OFILM_MODULE_VENDOR_ID == vendor_id) && (ULC2_HUAWEI_CAMERA_FF_NUM == current_otp->camera_id)) 
		{
			 ret = true;	
		}
		else if ((FOXCONN_MODULE_VENDOR_ID == vendor_id) && (ULC2_HUAWEI_CAMERA_AF_NUM == current_otp->camera_id))
		{
		     s5k5e2_otp_is_af_module =  true; 	
			 ret = true;
		}
		else
		{
			ret = false;
		}
	}
	else
	{
	    pr_err("%s,s5k5e2_otp_read_id error!\n",__func__);
		ret = false;
	}

	return ret;

}

/******************************************************************************
Function   :    s5k5e2_otp_read_awb
Description:   read the awb data from otp space .
******************************************************************************/
static bool s5k5e2_otp_read_awb(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *otp_ptr)
{ 	 
	bool ret = false;
     
	s5k5e2_otp_read_page_data_enable(s_ctrl,OTP_AWB_PAGE_NUM);
	ret = s5k5e2_otp_read_awb_group_reg_data(s_ctrl,otp_ptr);
	s5k5e2_otp_read_page_data_disable(s_ctrl);
       
	return ret;
}

/******************************************************************************
Function   :    s5k5e2_otp_check_awb
Description:  check the awb data is ok or not .
******************************************************************************/
static bool s5k5e2_otp_check_awb(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *current_otp)
{
	bool ret = false;
	  
	if (true == s5k5e2_otp_read_awb(s_ctrl,current_otp))
	{
		ret = true;
	}
	else
	{
		ret = false;
	}

	return ret;
}

/******************************************************************************
Function   :    s5k5e2_otp_read_awb
Description:   read the awb data from otp space .
******************************************************************************/
static bool s5k5e2_otp_read_vcm(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *otp_ptr)
{ 	 
	bool ret = false;
     
	s5k5e2_otp_read_page_data_enable(s_ctrl,OTP_VCM_PAGE_NUM);
	ret = s5k5e2_otp_read_vcm_group_reg_data(s_ctrl,otp_ptr);
	s5k5e2_otp_read_page_data_disable(s_ctrl);
       
	return ret;
}

/******************************************************************************
Function   :    s5k5e2_otp_check_awb
Description:  check the awb data is ok or not .
******************************************************************************/
static bool s5k5e2_otp_check_vcm(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *current_otp)
{
	bool ret = false;
	  
	if (true == s5k5e2_otp_read_vcm(s_ctrl,current_otp))
	{
		ret = true;
	}
	else
	{
		ret = false;
	}

	return ret;
}


/******************************************************************************
Function   :    s5k5e2_otp_check_lsc_autoload_flag
Description:  check the lsc autoload flag is ok or not .
******************************************************************************/
static bool s5k5e2_otp_check_lsc_autoload_flag(struct msm_sensor_ctrl_t * s_ctrl)
{ 	 
	bool ret = false;
	uint8_t autoload_disable_flag1 = 0xFF;
	uint8_t autoload_disable_flag2 = 0xFF;
	uint8_t autoload_disable_flag3 = 0xFF;
   
	s5k5e2_otp_read_page_data_enable(s_ctrl,OTP_LSC_FLAG_PAGE_NUM);
      	
	s5k5e2_read_otp(s_ctrl, S5K5E2_OTP_LSC_FLAG1_REG, &autoload_disable_flag1, 1);
	s5k5e2_read_otp(s_ctrl, S5K5E2_OTP_LSC_FLAG2_REG, &autoload_disable_flag2, 1);
	s5k5e2_read_otp(s_ctrl, S5K5E2_OTP_LSC_FLAG3_REG, &autoload_disable_flag3, 1);
	
    if (0 != (autoload_disable_flag1 | autoload_disable_flag2 | autoload_disable_flag3))
	{
		ret = false;
	}
	else
	{
		ret = true;
	}
	
	s5k5e2_otp_read_page_data_disable(s_ctrl);
	
	return ret;
}

/******************************************************************************
Function   :    s5k5e2_otp_check_lsc_data
Description:   check the lsc data is not loaded and then update it.
******************************************************************************/
static bool s5k5e2_otp_check_lsc(struct msm_sensor_ctrl_t * s_ctrl)
{
       bool ret = false;
	   
	if (false == s5k5e2_otp_check_lsc_autoload_flag(s_ctrl))
	{
		pr_err("%s: s5k5e2 otp lsc autoload disable flag is not zero.\n",__func__); 
		ret = false;
	}
	else
	{
		pr_info("%s: the lsc data have been autoloaded.\n",__func__);
		ret = true;
	}

	return ret;
}

/****************************************************************************
* FunctionName: s5k5e2_otp_set_flag;
* Description    : set the s5k5e2 otp flag;
* Input           : NA;
* Output         : NA;
* ReturnValue : NA;
* Other           : NA;
***************************************************************************/
static void s5k5e2_otp_set_flag(uint8_t flag)
{
	s5k5e2_otp_read_flag |= flag;
	return;
}

/****************************************************************************
* FunctionName: s5k5e2_otp_get_flag;
* Description : get the s5k5e2 flag;
* Input           : NA;
* Output         : NA;
* ReturnValue : NA;
* Other           : NA;
***************************************************************************/
static uint8_t s5k5e2_otp_get_flag(void)
{
	return s5k5e2_otp_read_flag;
}

/******************************************************************************
Function   :    s5k5e2_otp_update_lsc
Description:   update the lsc data to sensor
******************************************************************************/
static void s5k5e2_otp_update_lsc(struct msm_sensor_ctrl_t * s_ctrl)
{
	if (0 != (s5k5e2_otp_get_flag() & S5K5E2_OTP_LSC_READ_FLAG))
	{
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x340f, 0x81);   
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x0B00, 0x01);   
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3400, 0x00);
		   
		msleep(15);
		
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3B4C, 0x00);  
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3B4C, 0x01);   /* auto read on */
		
		pr_info("%s: lsc para have been autoloaded!\n",__func__);
	}
	else
	{
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x340f, 0x81);   
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x0B00, 0x01);   
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3400, 0x00);
		   
		msleep(15);
		
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3457, 0x04);    
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3B4C, 0x00);    /* auto read off */
		
		//GR 0xEED WRITE REGION  
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x7e);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x17);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xde);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xde);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x01);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x20);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xed);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x02);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xce);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x03);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x11);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x7b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfb);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x20);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x05);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xce);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xc5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x06);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x0c);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xaa);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x07);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe7);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xc4);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x08);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x16);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x74);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x09);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xff);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x1d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x0A);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x73);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x0B);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x2a);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xc5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x0C);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x24);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x0D);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x16);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x67);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x0E);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe7);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xa4);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x0F);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf0);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xe5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x10);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1e);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x35);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x11);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xdc);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x59);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x12);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xef);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x22);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x13);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf6);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x05);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x14);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x18);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x3d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x15);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x0d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x27);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x16);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe0);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xda);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x17);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x2b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x23);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x18);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x13);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xb1);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x19);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf8);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x30);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x1A);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xef);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x1e);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x1B);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x0c);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x99);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x1C);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x02);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x91);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x1D);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe2);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xf3);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x1E);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf6);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x8c);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x1F);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x12);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xcb);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x20);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf9);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x72);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x21);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xa1);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x22);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x09);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x88);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x23);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
                                                       
		//R 0xEED WRITE REGION
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x98);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xa2);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x24);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe1);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xec);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x25);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x10);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xad);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x26);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x46);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x27);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfd);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x59);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x28);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xff);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x62);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x29);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xcf);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x36);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x2A);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf3);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x9e);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x2B);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfe);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xcd);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x2C);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x02);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x96);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x2D);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x11);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x85);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x2E);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xef);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xfc);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x2F);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x12);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x30);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x30);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1e);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xe9);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x31);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x0b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xe4);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x32);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe4);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xb0);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x33);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe9);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x06);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x34);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x23);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xdf);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x35);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x03);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x81);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x36);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xcf);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xe6);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x37);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf6);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x4f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x38);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x2a);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xac);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x39);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x09);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xba);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x3A);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe2);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x80);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x3B);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x53);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x3C);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x29);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xe0);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x3D);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xed);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x44);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x3E);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf6);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x05);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x3F);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x06);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xae);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf7);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x88);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x41);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xd7);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x78);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x42);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf4);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xcb);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x43);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1e);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x1c);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x44);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x51);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x45);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfc);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x5d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x46);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x18);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x10);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x47);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
                                                       
		//B 0xEED WRITE REGION
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x73);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x78);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x48);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe4);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xa3);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x49);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1a);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x08);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x4A);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xec);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x0c);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x4B);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x0e);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x35);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x4C);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfa);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xca);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x4D);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xd8);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x49);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x4E);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x09);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x92);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x4F);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xeb);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xa4);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x50);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x15);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x77);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x51);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf8);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xa9);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x52);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfd);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xec);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x53);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x8a);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x54);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x02);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x3b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x55);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x13);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xc5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x56);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe8);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x02);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x57);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfb);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x23);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x58);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xd7);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x59);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe9);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x4c);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x5A);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xef);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xc5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x5B);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xf9);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x5C);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x14);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x06);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x5D);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x08);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xfa);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x5E);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xef);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x09);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x5F);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x22);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x60);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x10);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xc6);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x61);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfa);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xfc);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x62);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf9);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x56);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x63);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x02);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xa1);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x64);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfc);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xcd);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x65);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xeb);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x12);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x66);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf9);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x96);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x67);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x0d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x79);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x68);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x88);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x69);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfd);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x12);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x6A);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x0b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x3a);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x6B);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
                                                       
		//GB 0xEED WRITE REGION                    
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x7d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xab);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x6C);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xdd);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xf7);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x6D);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x22);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xf1);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x6E);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe2);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x0d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x6F);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x15);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xb5);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x70);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf9);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x0e);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x71);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xcf);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x4c);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x72);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x2d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x73);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe1);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xd3);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x74);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x9d);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x75);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf6);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xc6);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x76);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf8);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xb8);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x77);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x2b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x9c);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x78);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xfb);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x29);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x79);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xa1);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x7A);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xdd);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x19);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x7B);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf6);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x3b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x7C);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x1f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x09);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x7D);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xda);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xb8);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x7E);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf2);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x03);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x7F);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf3);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x25);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x80);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x18);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x65);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x81);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x11);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xb4);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x82);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xdc);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x26);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x83);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x2b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x5e);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x84);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x18);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x95);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x85);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xeb);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x5a);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x86);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xff);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xe3);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x87);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x02);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x8a);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x88);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x01);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xce);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x89);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xe3);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0xdf);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x8A);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xef);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x81);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x8B);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x21);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x8C);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xec);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x21);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x8D);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x0f);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0xf4);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x9b);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x8E);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3415, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3416, 0x14);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3417, 0x77);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3418, 0x8F);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3419, 0x04);
                                                       
		//ALPHA WRITE REGION
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x341A, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x341B, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x341C, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x341D, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x341E, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x341F, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3420, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3421, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3422, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3423, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3424, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3425, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3426, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3427, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3428, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3429, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x342A, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x342B, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x342C, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x342D, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x342E, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x342F, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3430, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3431, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3432, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3433, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3434, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3435, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3436, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3437, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3438, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3439, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x343A, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x343B, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x343C, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x343D, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x343E, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x343F, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3440, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3441, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3442, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3443, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3444, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3445, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3446, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3447, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3448, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3449, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x344A, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x344B, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x344C, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x344D, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x344E, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x344F, 0x00);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3450, 0x40);
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3451, 0x00);
	 
		s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl, 0x3457, 0x0C);    

		msleep(15);
		
		pr_info("%s: lsc para have been mannal loaded successful!\n",__func__);
	}
	
	return;
}

/******************************************************************************
Function   :    s5k5e2_otp_update_awb
Description:   update the awb data to sensor
******************************************************************************/
static void s5k5e2_otp_update_awb(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *current_otp)
{
	uint32_t r_gain = 0;
	uint32_t g_gain = 0;
	uint32_t b_gain = 0;
	uint32_t g_gain_r = 0;
	uint32_t g_gain_b = 0;

	uint32_t  rg = (current_otp->wb_rg_h<<8) | current_otp->wb_rg_l;
	uint32_t  bg = (current_otp->wb_bg_h<<8) | current_otp->wb_bg_l;
        
	if (bg < bg_ratio_typical) 
	{
		if (rg< rg_ratio_typical)
		{
			g_gain = 0x100;
			b_gain = (0x100 * bg_ratio_typical) / bg;
			r_gain = (0x100 * rg_ratio_typical) / rg;
		}
		else
		{
			r_gain = 0x100;
			g_gain = (0x100 * rg) / rg_ratio_typical;
			b_gain = (g_gain * bg_ratio_typical) / bg;
		}
	}
	else 
	{
		if (rg < rg_ratio_typical)
		{
			b_gain = 0x100;
			g_gain = (0x100 * bg) / bg_ratio_typical;
			r_gain = (g_gain * rg_ratio_typical) / rg;
		}
		else
		{
			g_gain_b = (0x100 * bg) / bg_ratio_typical;
			g_gain_r = (0x100 * rg) / rg_ratio_typical;
			
			if(g_gain_b > g_gain_r )
			{
				b_gain = 0x100;
				g_gain = g_gain_b;
				r_gain = (g_gain * rg_ratio_typical) / rg;
			}
			else
			{
				r_gain = 0x100;
				g_gain = g_gain_r;
				b_gain = (g_gain * bg_ratio_typical) / bg;
			}
		}
	}

	pr_info("%s: R_gain:0x%x,G_gain:0x%x,B_gain:0x%x\n", __func__, r_gain, g_gain, b_gain);

	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_RG_H_REG,((r_gain>>8) & 0xff));
	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_RG_L_REG,(r_gain & 0xff));

	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_GR_H_REG,((g_gain>>8) & 0xff));
	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_GR_L_REG,(g_gain & 0xff));

	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_GB_H_REG,((g_gain>>8) & 0xff));
	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_GB_L_REG,(g_gain & 0xff));

	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_BG_H_REG,((b_gain>>8) & 0xff));
	s5k5e2_ofilm_ohw5f02_cci_i2c_write(s_ctrl,S5K5E2_OTP_BG_L_REG,(b_gain & 0xff));
	 
	return;
}

/******************************************************************************
Function   :    s5k5e2_otp_update_vcm
Description:   update the s5k5e2 vcm otp data
******************************************************************************/
static void s5k5e2_otp_update_vcm(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *current_otp)
{  
	s_ctrl->afc_otp_info.starting_dac = current_otp->VCM_start;
	s_ctrl->afc_otp_info.infinity_dac = current_otp->VCM_start;
	s_ctrl->afc_otp_info.macro_dac = current_otp->VCM_max;
}

/******************************************************************************
Function   :    s5k5e2_set_otp_data_to_sensor
Description:   set the otp data to sensor space
******************************************************************************/
static void s5k5e2_set_otp_data_to_sensor(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *current_otp)
{
       
	s5k5e2_otp_update_lsc(s_ctrl);
	s5k5e2_otp_update_awb(s_ctrl, current_otp);
	s5k5e2_otp_update_vcm(s_ctrl, current_otp);

	return;
}

/******************************************************************************
Function   :   s5k5e2_otp_debug
Description:   Only for debug use
******************************************************************************/
void s5k5e2_otp_debug(struct S5K5E2_OTP_STRUCT *otp_ptr)
{
	pr_info("%s,otp_ptr->year:0x%x\n",__func__,otp_ptr->year);
	pr_info("%s,otp_ptr->month:0x%x\n",__func__,otp_ptr->month);
	pr_info("%s,otp_ptr->day:0x%x\n",__func__,otp_ptr->day);
	pr_info("%s,otp_ptr->camera_id:0x%x\n",__func__,otp_ptr->camera_id);
	pr_info("%s,otp_ptr->vendor_id:0x%x\n",__func__,otp_ptr->vendor_id);
	pr_info("%s,otp_ptr->wb_rg_h:0x%x\n",__func__,otp_ptr->wb_rg_h);
	pr_info("%s,otp_ptr->wb_rg_l:0x%x\n",__func__,otp_ptr->wb_rg_l);
	pr_info("%s,otp_ptr->wb_bg_h:0x%x\n",__func__,otp_ptr->wb_bg_h);
	pr_info("%s,otp_ptr->wb_bg_l:0x%x\n",__func__,otp_ptr->wb_bg_l);
	pr_info("%s,otp_ptr->wb_gbgr_h:0x%x\n",__func__,otp_ptr->wb_gbgr_h);
	pr_info("%s,otp_ptr->wb_gbgr_l:0x%x\n",__func__,otp_ptr->wb_gbgr_l);
	if (s5k5e2_otp_is_af_module)
	{
		pr_info("%s,otp_ptr->VCM_start:0x%x\n",__func__,otp_ptr->VCM_start);
		pr_info("%s,otp_ptr->VCM_max:0x%x\n",__func__,otp_ptr->VCM_max);
	}
	pr_info("%s,s5k5e2_otp_get_flag:0x%x\n",__func__,s5k5e2_otp_get_flag());
	return;
}

/******************************************************************************
Function   :   s5k5e2_get_otp_from_sensor
Description:   read and check OTP data
******************************************************************************/
static void s5k5e2_get_otp_from_sensor(struct msm_sensor_ctrl_t * s_ctrl, struct S5K5E2_OTP_STRUCT *current_otp)
{
	bool ret = true;
		
	ret = s5k5e2_otp_check_id(s_ctrl, current_otp);
	if (ret)
	{
		s5k5e2_otp_set_flag(S5K5E2_OTP_ID_READ_FLAG);
		pr_info("%s: check id ok!\n",__func__);
	}
	else
	{
		pr_err("%s: check id error!\n",__func__);
		return;
	}
	
	ret = s5k5e2_otp_check_lsc(s_ctrl);
	if (ret)
	{
		s5k5e2_otp_set_flag(S5K5E2_OTP_LSC_READ_FLAG);
		pr_info("%s: check lsc ok!\n",__func__);
	}
	else
	{
		pr_err("%s: check lsc error!\n",__func__);
	}
	
	ret = s5k5e2_otp_check_awb(s_ctrl, current_otp);
	if (ret)
	{
		s5k5e2_otp_set_flag(S5K5E2_OTP_AWB_READ_FLAG);
		pr_info("%s: check awb ok!\n",__func__);
	}
	else
	{
		pr_err("%s: check awb error!\n",__func__);
		return;
	}
	
	if (s5k5e2_otp_is_af_module)
	{
		ret = s5k5e2_otp_check_vcm(s_ctrl, current_otp);
		if (ret)
		{
			s5k5e2_otp_set_flag(S5K5E2_OTP_VCM_READ_FLAG);
			pr_info("%s: check vcm ok!\n",__func__);
		}
		else
		{
			pr_err("%s: check vcm error!\n",__func__);
		}
	}
	
	return;
}

/******************************************************************************
Function   :    s5k5e2_otp_func
Description:   the interface of s5k5e2 OTP
******************************************************************************/
int s5k5e2_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index)
{     
	uint8_t   otp_read_flag = 0;
	
	pr_info("%s enters!\n",__func__); 

	if (otp_function_lists[index].rg_ratio_typical)	
	{		
		rg_ratio_typical = otp_function_lists[index].rg_ratio_typical;	
	}
       
	if (otp_function_lists[index].bg_ratio_typical)	
	{		
		bg_ratio_typical = otp_function_lists[index].bg_ratio_typical;	
	}

	pr_debug("%s, rg_ratio_typical=%04x,bg_ratio_typical=%04x\n", __func__,rg_ratio_typical,bg_ratio_typical);

	otp_read_flag = s5k5e2_otp_is_af_module ? S5K5E2_OTP_AF_READ_ALL_FLAG : S5K5E2_OTP_FF_READ_ALL_FLAG;

	if (0 == (s5k5e2_otp_get_flag()&otp_read_flag))
	{
		s5k5e2_get_otp_from_sensor(s_ctrl, &g_cur_opt_params);
		s5k5e2_otp_debug(&g_cur_opt_params);
	}
	else
	{
		pr_info("%s: s5k5e2 OTP has already been read out!\n",__func__);
	}
	
	if (otp_read_flag != (s5k5e2_otp_get_flag()&otp_read_flag))
	{
		pr_err("%s: get OTP info error,s5k5e2_otp_read_flag = 0x%x\n", __func__,s5k5e2_otp_get_flag());
		return S5K5E2_OTP_ERROR;
	}
	  
	s5k5e2_set_otp_data_to_sensor(s_ctrl, &g_cur_opt_params);
	
	return S5K5E2_OTP_OK;	

}
