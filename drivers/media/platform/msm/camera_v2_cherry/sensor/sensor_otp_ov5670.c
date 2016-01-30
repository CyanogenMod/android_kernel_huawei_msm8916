/************************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  FileName: sensor_otp_ov5670.c
  Version :Initial Draft
  Date: 2014/10/11
  Description:    this file contion several functions to detect otp_ov5670 properties
  Version:         Initial Draft
  History:
   History        :
   1.Date        : 2014/10/11
   Modification : Created function
***********************************************************/
#define HW_CMR_LOG_TAG "sensor_otp_ov5670"
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

#define OV5670_OTP_ERROR          (-1)
#define OV5670_OTP_OK              0 

#define OV5670_OTP_READ_ENABLE     1
#define OV5670_OTP_READ_DISABLE    0

#define OV5670_GROUP_DATA_EMPTY    0
#define OV5670_GROUP_DATA_INVALID  1
#define OV5670_GROUP_DATA_VALID    2

#define OV5670_OTP_GROUP1          0
#define OV5670_OTP_GROUP2          1
#define OV5670_OTP_GROUP3          2
#define OV5670_OTP_GROUPMAX        3
           
#define OV5670_OTP_ID_READ_FLAG      (1 << 0)
#define OV5670_OTP_AWB_READ_FLAG     (1 << 1)
#define OV5670_OTP_LSC_READ_FLAG     (1 << 2)
#define OV5670_OTP_VCM_READ_FLAG     (1 << 3)

#define OV5670_OTP_FF_READ_ALL_FLAG     (OV5670_OTP_ID_READ_FLAG|OV5670_OTP_AWB_READ_FLAG|OV5670_OTP_LSC_READ_FLAG)
#define OV5670_OTP_AF_READ_ALL_FLAG 	  (OV5670_OTP_FF_READ_ALL_FLAG|OV5670_OTP_VCM_READ_FLAG)

#define OPT_LSC_MAX_LENGTH                     16

/* OV5670 OTP REG */                         
#define OV5670_OTP_ID_FLAG_REG                 0x7010 
#define OV5670_OTP_ID_GROUP1_START_REG         0x7011
#define OV5670_OTP_ID_GROUP1_END_REG	       0x7015
#define OV5670_OTP_ID_GROUP2_START_REG         0x7016
#define OV5670_OTP_ID_GROUP2_END_REG           0x701A
#define OV5670_OTP_ID_GROUP3_START_REG         0x701B
#define OV5670_OTP_ID_GROUP3_END_REG           0x701F

#define OV5670_OTP_AWB_FLAG_REG                0x7020
#define OV5670_OTP_AWB_GROUP1_START_REG        0x7021
#define OV5670_OTP_AWB_GROUP1_END_REG          0x7026
#define OV5670_OTP_AWB_GROUP2_START_REG        0x7027
#define OV5670_OTP_AWB_GROUP2_END_REG          0x702C
#define OV5670_OTP_AWB_GROUP3_START_REG        0x702D
#define OV5670_OTP_AWB_GROUP3_END_REG          0x7032

#define OV5670_OTP_LSC_FLAG_REG                0x7040
#define OV5670_OTP_LSC_GROUP1_START_REG        0x7041
#define OV5670_OTP_LSC_GROUP1_END_REG          0x7050
#define OV5670_OTP_LSC_GROUP2_START_REG        0x7051
#define OV5670_OTP_LSC_GROUP2_END_REG          0x7060
#define OV5670_OTP_LSC_GROUP3_START_REG        0x7061
#define OV5670_OTP_LSC_GROUP3_END_REG          0x7070

#define OV5670_OTP_VCM_FLAG_REG                0x7033
#define OV5670_OTP_VCM_GROUP1_START_REG        0x7034
#define OV5670_OTP_VCM_GROUP1_END_REG          0x7037
#define OV5670_OTP_VCM_GROUP2_START_REG        0x7038
#define OV5670_OTP_VCM_GROUP2_END_REG          0x703b
#define OV5670_OTP_VCM_GROUP3_START_REG        0x703c
#define OV5670_OTP_VCM_GROUP3_END_REG          0x703f

#define OV5670_OTP_DPC_REG                     0x5002  
#define OV5670_OTP_SENSOR_CONTRL_REG           0x100
#define OV5670_OTP_READ_ENABLE_REG             0x3D81
#define OV5670_OTP_RW_MODE_REG                 0x3D84

#define OV5670_OTP_RW_START_H_ADDR_REG         0x3d88  
#define OV5670_OTP_RW_START_L_ADDR_REG         0x3d89 
#define OV5670_OTP_RW_END_H_ADDR_REG           0x3d8A
#define OV5670_OTP_RW_END_L_ADDR_REG           0x3d8B

#define OV5670_OTP_RG_H_REG                    0x5032
#define OV5670_OTP_RG_L_REG                    0x5033
#define OV5670_OTP_GBGR_H_REG                  0x05034
#define OV5670_OTP_GBGR_L_REG                  0x5035
#define OV5670_OTP_BG_H_REG                    0x5036
#define OV5670_OTP_BG_L_REG                    0x5037

#define ULC2_HUAWEI_CAMERA_FF_NUM              0xB2  
#define BYD_HUAWEI_CAMERA_FF_TMP_NUM           0x4E 
#define ULC2_HUAWEI_CAMERA_AF_NUM              0xB3 
#define OV5670_OTP_VCM_OFFSET_VALUE            100

typedef enum {
	SUNNY_MODULE_VENDOR_ID = 1,
	FOXCONN_MODULE_VENDOR_ID,
	LITEON_MODULE_VENDOR_ID,
	SEMCO_MODULE_VENDOR_ID,
	BYD_MODULE_VENDOR_ID,
	OFILM_MODULE_VENDOR_ID
}camera_module_vendor_id;

typedef enum {
	OV5670_ID_OTP,
	OV5670_AWB_OTP,
	OV5670_LSC_OTP,
	OV5670_VCM_OTP
}ov5670_otp_type;

struct OV5670_OTP_STRUCT {
    uint16_t year;
    uint16_t month;
    uint16_t day;
    uint16_t camera_num;
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
struct OV5670_OTP_STRUCT g_cur_ov5670_otp_params = { 0 };

typedef struct ov5670_otp_reg_addr {
	uint16_t start_address;
	uint16_t end_address;
}OV5670_GROUP_REG_ADDR;

static OV5670_GROUP_REG_ADDR ov5670_id_read_group_addr[OV5670_OTP_GROUPMAX] = {
	{OV5670_OTP_ID_GROUP1_START_REG,OV5670_OTP_ID_GROUP1_END_REG},
	{OV5670_OTP_ID_GROUP2_START_REG,OV5670_OTP_ID_GROUP2_END_REG},
	{OV5670_OTP_ID_GROUP3_START_REG,OV5670_OTP_ID_GROUP3_END_REG}
};

static OV5670_GROUP_REG_ADDR ov5670_awb_read_group_addr[OV5670_OTP_GROUPMAX] = {
	{OV5670_OTP_AWB_GROUP1_START_REG,OV5670_OTP_AWB_GROUP1_END_REG},
	{OV5670_OTP_AWB_GROUP2_START_REG,OV5670_OTP_AWB_GROUP2_END_REG},
	{OV5670_OTP_AWB_GROUP3_START_REG,OV5670_OTP_AWB_GROUP3_END_REG}
};

static OV5670_GROUP_REG_ADDR ov5670_lsc_read_group_addr[OV5670_OTP_GROUPMAX] = {
	{OV5670_OTP_LSC_GROUP1_START_REG,OV5670_OTP_LSC_GROUP1_END_REG},
	{OV5670_OTP_LSC_GROUP2_START_REG,OV5670_OTP_LSC_GROUP2_END_REG},
	{OV5670_OTP_LSC_GROUP3_START_REG,OV5670_OTP_LSC_GROUP3_END_REG},
};

static OV5670_GROUP_REG_ADDR ov5670_vcm_read_group_addr[OV5670_OTP_GROUPMAX] = {
	{OV5670_OTP_VCM_GROUP1_START_REG,OV5670_OTP_VCM_GROUP1_END_REG},
	{OV5670_OTP_VCM_GROUP2_START_REG,OV5670_OTP_VCM_GROUP2_END_REG},
	{OV5670_OTP_VCM_GROUP3_START_REG,OV5670_OTP_VCM_GROUP3_END_REG},
};

static uint8_t  ov5670_otp_read_flag = 0;
static bool      ov5670_otp_is_af_module = false;   /* AF = true, FF = false */
static uint32_t rg_ratio_typical  = 0x0274;  //the average of 4 Golden samples' RG ratio
static uint32_t bg_ratio_typical  = 0x029c; //the average of 4 Golden samples' BG ratio

/****************************************************************************
* FunctionName: ov5670_ofilm_ohw5f02_cci_i2c_write;
* Description : i2c write interface;
***************************************************************************/
static int32_t ov5670_ofilm_ohw5f02_cci_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, u16 data)
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
* FunctionName: ov5670_ofilm_ohw5f02_cci_i2c_read;
* Description : i2c read interface;
***************************************************************************/
static int32_t ov5670_ofilm_ohw5f02_cci_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t *data)
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
* FunctionName: ov5670_read_otp;
* Description : i2c api used to read information from eeprom.
* Input       : NA;
* Output      : ov5670_otp;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static bool ov5670_read_otp(struct msm_sensor_ctrl_t *s_ctrl,uint16_t reg,uint8_t *buf,uint16_t count)
{
	uint16_t i   = 0;
	int ret = 0;
	uint16_t val = 0;

	for (i=0; i<count; i++)
	{
		ret =ov5670_ofilm_ohw5f02_cci_i2c_read(s_ctrl,(reg+i) ,&val);
		
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
* FunctionName: ov5670_otp_enable_DPC;
* Description : enabled the DPC;
***************************************************************************/
static void ov5670_otp_enable_DPC(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t data = 0;
	
	ov5670_read_otp(s_ctrl, OV5670_OTP_DPC_REG, &data,1);
	pr_info("%s change 0x%x value from 0x%x to 0x%x\n", __func__,OV5670_OTP_DPC_REG, data,  data | 0x08);
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_DPC_REG, data | 0x08);
	
	return;
}

/****************************************************************************
* FunctionName: ov5670_otp_disable_DPC;
* Description : disabled the DPC;
***************************************************************************/
static void ov5670_otp_disable_DPC(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t data = 0;
	
	ov5670_read_otp(s_ctrl, OV5670_OTP_DPC_REG, &data,1);
	pr_info("%s change 0x%x value from 0x%x to 0x%x\n",__func__, OV5670_OTP_DPC_REG, data,  data & 0xF7);
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_DPC_REG, data & 0xF7);
	
	return;
}

/****************************************************************************
* FunctionName: ov5670_otp_set_read_mode;
* Description : set the ov5670 enter read mode;
***************************************************************************/
static void ov5670_otp_set_read_mode(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t data = 0;
	
	ov5670_read_otp(s_ctrl, OV5670_OTP_RW_MODE_REG, &data,1);
	pr_info("%s change 0x%x value from 0x%x to 0x%x\n",__func__, OV5670_OTP_RW_MODE_REG, data,  data | 0xC0);
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_RW_MODE_REG, data | 0xC0);
	
	return ;
}

/****************************************************************************
* FunctionName: ov5670_otp_read_enable;
* Description : read enabled;
***************************************************************************/
static void ov5670_otp_read_enable(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t data = 0;

	ov5670_read_otp(s_ctrl, OV5670_OTP_READ_ENABLE_REG, &data,1);
	pr_info("%s change 0x%x value from 0x%x to 0x%x\n",__func__, OV5670_OTP_READ_ENABLE_REG, data,  data | 0x01);
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_READ_ENABLE_REG, data | 0x01 );

	return;
}

/****************************************************************************
* FunctionName: ov5670_otp_read_disable;
* Description : read disabled;
***************************************************************************/	
static void ov5670_otp_read_disable(struct msm_sensor_ctrl_t *s_ctrl)
{
	   uint8_t data = 0;
         ov5670_read_otp(s_ctrl, OV5670_OTP_READ_ENABLE_REG, &data,1);

        pr_info("%s change 0x%x value from 0x%x to 0x%x\n",__func__, OV5670_OTP_READ_ENABLE_REG, data,  data & 0xFE);
		
	  ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_READ_ENABLE_REG, data & 0xFE);
	  
	return;
}

/****************************************************************************
* FunctionName: ov5670_otp_set_sensor_active;
* Description : set the ov5670 sensor active;
***************************************************************************/
static void ov5670_otp_set_sensor_active(struct msm_sensor_ctrl_t *s_ctrl)
{
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_SENSOR_CONTRL_REG, 0x1);
}

/****************************************************************************
* FunctionName: ov5670_otp_set_sensor_standby;
* Description : set the ov5670 enter standby;
***************************************************************************/
static void ov5670_otp_set_sensor_standby(struct msm_sensor_ctrl_t *s_ctrl)
{
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_SENSOR_CONTRL_REG, 0x0);
}

/****************************************************************************
* FunctionName: ov5670_otp_clean_buf;
* Description : make initial state and set read mode of NVM controller interface1.
* Input       : page_num;
* Output      : NONEl;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static void ov5670_otp_clean_buf(struct msm_sensor_ctrl_t * s_ctrl, uint32_t address_start, uint32_t address_end)
{
	uint32_t regaddr = 0;

	for (regaddr = address_start; regaddr<=address_end; regaddr++)
	{
		ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, regaddr, 0x00);
	}
	
	return;
}

/****************************************************************************
* FunctionName: ov5670_otp_read_pre_process;
* Description : ov5670 read pre-process;
***************************************************************************/
static void ov5670_otp_read_pre_process(struct msm_sensor_ctrl_t * s_ctrl, uint32_t address_start, uint32_t address_end)
{ 
	ov5670_otp_clean_buf(s_ctrl,address_start,address_end);
	
	ov5670_otp_set_read_mode(s_ctrl);
	
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_RW_START_H_ADDR_REG, (address_start>>8));
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_RW_START_L_ADDR_REG, (address_start&0xff));
	
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_RW_END_H_ADDR_REG, (address_end>>8));
	ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_RW_END_L_ADDR_REG, (address_end&0xff));
	
	ov5670_otp_read_enable(s_ctrl);
	
	msleep(25);
	
	return;
}

/****************************************************************************
* FunctionName: ov5670_otp_read_post_process;
* Description : ov5670 read post-process;
***************************************************************************/
static void ov5670_otp_read_post_process(struct msm_sensor_ctrl_t * s_ctrl, uint32_t address_start, uint32_t address_end)
{
	ov5670_otp_read_disable(s_ctrl);
	ov5670_otp_clean_buf(s_ctrl,address_start,address_end);
	return;
}

/****************************************************************************
* FunctionName: ov5670_otp_read_flag_process;
* Description : ov5670 read flag process;
***************************************************************************/
static void ov5670_otp_read_flag_process(struct msm_sensor_ctrl_t * s_ctrl, uint16_t flag_reg, uint8_t *flag_value)
{
	ov5670_otp_read_pre_process(s_ctrl,flag_reg,flag_reg);
	
	ov5670_read_otp(s_ctrl,flag_reg,flag_value,1);
	
	ov5670_otp_read_post_process(s_ctrl,flag_reg,flag_reg);
	
	return;
}

/****************************************************************************
* FunctionName: ov5670_otp_read_group_data_process;
* Description : ov5670 read group data process;
***************************************************************************/
static void ov5670_otp_read_group_data_process(struct msm_sensor_ctrl_t * s_ctrl, uint32_t address_start, uint32_t address_end, uint8_t *buf,uint16_t count)
{
	ov5670_otp_read_pre_process(s_ctrl,address_start,address_end);

	ov5670_read_otp(s_ctrl,address_start,buf,count);

	ov5670_otp_read_post_process(s_ctrl,address_start,address_end);
	
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
static int ov5670_select_otp_group(struct msm_sensor_ctrl_t * s_ctrl, uint8_t *index, uint8_t flag)
{
	int ret = 0xFF;
	uint8_t group_num = 0;
	uint8_t group_flag  = 0;
	uint8_t avalid_data_flag = 0;
	
	*index = OV5670_OTP_GROUPMAX;
	for (group_num=OV5670_OTP_GROUP1; group_num<OV5670_OTP_GROUPMAX; group_num++)
	{
		group_flag = (flag>>(2 * (OV5670_OTP_GROUPMAX - group_num))) & 0x03;
		pr_info("[select_otp_group]: group_num=%d;flag=%d.\n ", group_num, group_flag);
		if (0x01 == group_flag)
		{
			(*index) = group_num;
			avalid_data_flag++;
		}
	}

	if (1 == avalid_data_flag)
	{
		ret = OV5670_GROUP_DATA_VALID;
	}
	else
	{
		ret = OV5670_GROUP_DATA_INVALID;
	}
	
	pr_info("[select_otp_group]: *index=%d; ret=%d.\n ", *index, ret);

	return ret;

}

/******************************************************************************
  Function:    ov5670_otp_read_id_group_reg_data                
  Description: get the group reg data.
*******************************************************************************/
static bool ov5670_otp_read_group_data(struct msm_sensor_ctrl_t *s_ctrl, ov5670_otp_type otp_type, struct OV5670_OTP_STRUCT *otp_ptr)
{
	uint8_t otp_buf[OPT_LSC_MAX_LENGTH] = { 0 };
	uint8_t module_id_flag = 0;
	uint8_t buf_count = 0;
	uint8_t otp_group_num = 0;
	uint16_t start_address = 0;
	uint16_t end_address = 0;
	uint16_t check_flag_reg = 0;
	int tmpret = 0;
	uint32_t  rg = 0;
	uint32_t  bg = 0; 
	OV5670_GROUP_REG_ADDR  *pcur_addr_array = NULL;

	switch(otp_type)
	{
		case OV5670_ID_OTP:
		{
			check_flag_reg    = OV5670_OTP_ID_FLAG_REG;
			pcur_addr_array = ov5670_id_read_group_addr;
			break;
		}
		case OV5670_AWB_OTP:
		{
			check_flag_reg    = OV5670_OTP_AWB_FLAG_REG;
			pcur_addr_array = ov5670_awb_read_group_addr;
			break;
		}
		case OV5670_LSC_OTP:
		{
			check_flag_reg    = OV5670_OTP_LSC_FLAG_REG;
			pcur_addr_array = ov5670_lsc_read_group_addr;
			break;
		}
		case OV5670_VCM_OTP:
		{
			check_flag_reg    = OV5670_OTP_VCM_FLAG_REG;
			pcur_addr_array = ov5670_vcm_read_group_addr;
			break;
		}
		default:
		{
			pr_err("%s: otp type error \n",__func__);
			return false;
		}
	}

	ov5670_otp_read_flag_process(s_ctrl, check_flag_reg, &module_id_flag);

	tmpret = ov5670_select_otp_group(s_ctrl,&otp_group_num,module_id_flag);

	if (OV5670_GROUP_DATA_VALID != tmpret || (OV5670_OTP_GROUPMAX <= otp_group_num))
	{
		pr_err("%s ov5670_select otp group false.\n",__func__);
		return false;
	}

	start_address = pcur_addr_array[otp_group_num].start_address;
	end_address = pcur_addr_array[otp_group_num].end_address;
	buf_count = end_address - start_address + 1;
	
	ov5670_otp_read_group_data_process(s_ctrl,start_address,end_address,otp_buf,buf_count);

	switch(otp_type)
	{
		case OV5670_ID_OTP:
		{
			otp_ptr->year = otp_buf[0];
			otp_ptr->month = otp_buf[1];
			otp_ptr->day = otp_buf[2];
			otp_ptr->camera_num = otp_buf[3];
			otp_ptr->vendor_id = otp_buf[4];
			break;
		}
		case OV5670_AWB_OTP:
		{
			otp_ptr->wb_rg_h = otp_buf[0];
			otp_ptr->wb_rg_l = otp_buf[1];
			otp_ptr->wb_bg_h= otp_buf[2];
			otp_ptr->wb_bg_l = otp_buf[3];
			otp_ptr->wb_gbgr_h = otp_buf[4];
			otp_ptr->wb_gbgr_l = otp_buf[5];
			rg = (otp_ptr->wb_rg_h<<8) | otp_ptr->wb_rg_l;
			bg = (otp_ptr->wb_bg_h<<8) | otp_ptr->wb_bg_l;
			if (0 == rg || 0 == bg)
		{
			pr_err("%s: error rg || bg is zero!\n",__func__);
			return false;
		}
			break;
		}
		case OV5670_LSC_OTP:
		{
			memcpy(otp_ptr->lenc, otp_buf, buf_count);
			break;
		}
		case OV5670_VCM_OTP:
		{
			otp_ptr->VCM_start = ((otp_buf[0] << 2) | ((otp_buf[1]>>6) & 0x03));
			otp_ptr->VCM_max =  ((otp_buf[2] << 2) | ((otp_buf[3]>>6) & 0x03));
			if ((0 == otp_ptr->VCM_start) || (0 == otp_ptr->VCM_max) || (otp_ptr->VCM_max <= otp_ptr->VCM_start))
			{
				pr_err("VCM_start-MSB = 0x%x\n",(otp_buf[0] << 2));
				pr_err("VCM_start-LSB = 0x%x\n",((otp_buf[1]>>6) & 0x03));
				pr_err("VCM_MAX-MSB = 0x%x\n",(otp_buf[2] << 2));
				pr_err("VCM_MAX-LSB  = 0x%x\n",((otp_buf[3]>>6) & 0x03));
				pr_err("VCM_START = 0x%x\n",otp_ptr->VCM_start);
				pr_err("VCM_MAX  = 0x%x\n",otp_ptr->VCM_max);
				return false;
			}
			
			if (0 > (int)(otp_ptr->VCM_start - OV5670_OTP_VCM_OFFSET_VALUE))
			{
				pr_err("%s, otp_ptr->VCM_start = 0x%x\n", __func__,otp_ptr->VCM_start);
				otp_ptr->VCM_start = 0;
			}
			else
			{
				otp_ptr->VCM_start -= OV5670_OTP_VCM_OFFSET_VALUE;
			}
			
			otp_ptr->VCM_max += OV5670_OTP_VCM_OFFSET_VALUE;
			
			break;
		}
		default:
		{
			pr_err("%s: otp type error \n",__func__);
			return false;
		}
	}

	return true;
	
}

/****************************************************************************
* FunctionName: ov5670_otp_read_id;
* Description : read the otp module id data.
* Input       : NONEl;
* Output      : NONEl;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static bool ov5670_otp_read_id(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *otp_ptr)
{ 	 
	return ov5670_otp_read_group_data(s_ctrl, OV5670_ID_OTP, otp_ptr);
}

/****************************************************************************
* FunctionName: ov5670_otp_check_id;
* Description : check the ov5670 module id data.
* Input       : NONEl;
* Output      : NONEl;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static bool ov5670_otp_check_id(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *current_otp)
{
	bool ret = false;
	uint8_t vendor_id = 0;
   
	if (true == ov5670_otp_read_id(s_ctrl,current_otp))
	{
	      pr_info("%s,ov5670_otp_read_id ok!\n",__func__);
		vendor_id = ((current_otp->vendor_id)>>4) & 0x0F;
		if ((BYD_MODULE_VENDOR_ID == vendor_id) && ((ULC2_HUAWEI_CAMERA_FF_NUM == current_otp->camera_num) || (BYD_HUAWEI_CAMERA_FF_TMP_NUM == current_otp->camera_num))) 
		{
		      ret = true;
		}
		else if ((OFILM_MODULE_VENDOR_ID == vendor_id) && (ULC2_HUAWEI_CAMERA_AF_NUM == current_otp->camera_num))
		{
		      ov5670_otp_is_af_module = true;
			  ret = true;
	    }
		else
		{
			ret = false;
		}
	}
	else
	{
	    pr_err("%s,ov5670_otp_read_id error!\n",__func__);
		ret = false;
	}
	return ret;

}

/******************************************************************************
Function   :    ov5670_otp_read_awb
Description:   read the awb data from otp space .
******************************************************************************/
static bool ov5670_otp_read_awb(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *otp_ptr)
{ 	   
	return ov5670_otp_read_group_data(s_ctrl, OV5670_AWB_OTP, otp_ptr); 
}

/******************************************************************************
Function   :    ov5670_otp_check_awb
Description:  check the awb data is ok or not .
******************************************************************************/
static bool ov5670_otp_check_awb(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *current_otp)
{
	bool ret = false;
	  
	if (true == ov5670_otp_read_awb(s_ctrl,current_otp))
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
Function   :    ov5670_otp_read_awb
Description:   read the awb data from otp space .
******************************************************************************/
static bool ov5670_otp_read_lsc(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *otp_ptr)
{ 	   
	return ov5670_otp_read_group_data(s_ctrl, OV5670_LSC_OTP, otp_ptr); 
}

/******************************************************************************
Function   :    ov5670_otp_check_awb
Description:  check the awb data is ok or not .
******************************************************************************/
static bool ov5670_otp_check_lsc(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *current_otp)
{
	bool ret = false;
	  
	if (true == ov5670_otp_read_lsc(s_ctrl,current_otp))
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
Function   :    ov5670_otp_read_vcm
Description:   read the awb data from otp space .
******************************************************************************/
static bool ov5670_otp_read_vcm(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *otp_ptr)
{ 	   
	return ov5670_otp_read_group_data(s_ctrl, OV5670_VCM_OTP, otp_ptr); 
}

/******************************************************************************
Function   :    ov5670_otp_check_awb
Description:  check the awb data is ok or not .
******************************************************************************/
static bool ov5670_otp_check_vcm(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *current_otp)
{
	bool ret = false;
	  
	if (true == ov5670_otp_read_vcm(s_ctrl,current_otp))
	{
		ret = true;
	}
	else
	{
		ret = false;
	}

	return ret;
}

/****************************************************************************
* FunctionName: ov5670_otp_set_flag;
* Description    : set the ov5670 otp flag;
* Input           : NA;
* Output         : NA;
* ReturnValue : NA;
* Other           : NA;
***************************************************************************/
static void ov5670_otp_set_flag(uint8_t flag)
{
	ov5670_otp_read_flag |= flag;
	return;
}

/****************************************************************************
* FunctionName: ov5670_otp_get_flag;
* Description : get the ov5670 flag;
* Input           : NA;
* Output         : NA;
* ReturnValue : NA;
* Other           : NA;
***************************************************************************/
static uint8_t ov5670_otp_get_flag(void)
{
	return ov5670_otp_read_flag;
}

/******************************************************************************
Function   :    ov5670_otp_update_awb
Description:   update the awb data to sensor
******************************************************************************/
static void ov5670_otp_update_awb(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *current_otp)
{
	uint32_t R_gain = 0;
	uint32_t G_gain = 0;
	uint32_t B_gain = 0;
	uint32_t Base_gain = 0;

	uint32_t  rg = (current_otp->wb_rg_h<<8) | current_otp->wb_rg_l;
	uint32_t  bg = (current_otp->wb_bg_h<<8) | current_otp->wb_bg_l;

	//calculate sensor WB gain, 0x400 = 1x gain
	R_gain = 0x400 * rg_ratio_typical / rg;
	G_gain = 0x400;
	B_gain = 0x400 * bg_ratio_typical / bg;

	// find gain<0x400
	Base_gain = G_gain;
	
	if (R_gain < Base_gain)
	{
		Base_gain = R_gain;
	}
	
	if (B_gain < Base_gain) 
	{
		Base_gain = B_gain;
	}
	
	// set min gain to 0x400
	R_gain = 0x400 * R_gain / Base_gain;
	G_gain = 0x400 * G_gain / Base_gain;
	B_gain = 0x400 * B_gain / Base_gain;
	
	// update sensor WB gain
	if (R_gain > 0x400)
	{
		ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_RG_H_REG, R_gain>>8);
		ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_RG_L_REG, R_gain & 0x00ff);
	}
	
	if (G_gain > 0x400) 
	{
		ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_GBGR_H_REG, G_gain>>8);
		ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_GBGR_L_REG, G_gain & 0x00ff);
	}
	
	if (B_gain > 0x400) 
	{
		ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_BG_H_REG, B_gain>>8);
		ov5670_ofilm_ohw5f02_cci_i2c_write(s_ctrl, OV5670_OTP_BG_L_REG, B_gain & 0x00ff);
	}
	
	return;
}
static void ov5670_otp_update_vcm(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *current_otp)
{  
	s_ctrl->afc_otp_info.starting_dac = current_otp->VCM_start;
	s_ctrl->afc_otp_info.infinity_dac = current_otp->VCM_start;
	s_ctrl->afc_otp_info.macro_dac = current_otp->VCM_max;
}

/******************************************************************************
Function   :    ov5670_set_otp_data_to_sensor
Description:   set the otp data to sensor space
******************************************************************************/
static void ov5670_set_otp_data_to_sensor(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *current_otp)
{
	ov5670_otp_update_awb(s_ctrl, current_otp);
	ov5670_otp_update_vcm(s_ctrl, current_otp);
	return;
}

/******************************************************************************
Function   :   ov5670_otp_debug
Description:   Only for debug use
******************************************************************************/
void ov5670_otp_debug(struct OV5670_OTP_STRUCT *otp_ptr)
{
  uint32_t count = 0;
	   
	pr_info("%s,otp_ptr->year:0x%x\n",__func__,otp_ptr->year);
	pr_info("%s,otp_ptr->month:0x%x\n",__func__,otp_ptr->month);
	pr_info("%s,otp_ptr->day:0x%x\n",__func__,otp_ptr->day);
	pr_info("%s,otp_ptr->camera_id:0x%x\n",__func__,otp_ptr->camera_num);
	pr_info("%s,otp_ptr->vendor_id:0x%x\n",__func__,otp_ptr->vendor_id);
	pr_info("%s,otp_ptr->wb_rg_h:0x%x\n",__func__,otp_ptr->wb_rg_h);
	pr_info("%s,otp_ptr->wb_rg_l:0x%x\n",__func__,otp_ptr->wb_rg_l);
	pr_info("%s,otp_ptr->wb_bg_h:0x%x\n",__func__,otp_ptr->wb_bg_h);
	pr_info("%s,otp_ptr->wb_bg_l:0x%x\n",__func__,otp_ptr->wb_bg_l);
	pr_info("%s,otp_ptr->wb_gbgr_h:0x%x\n",__func__,otp_ptr->wb_gbgr_h);
	pr_info("%s,otp_ptr->wb_gbgr_l:0x%x\n",__func__,otp_ptr->wb_gbgr_l);
	pr_info("%s,ov5670_otp_get_flag:0x%x\n",__func__,ov5670_otp_get_flag());
	if (ov5670_otp_is_af_module)
	{
		pr_info("%s,otp_ptr->VCM_start:0x%x\n",__func__,otp_ptr->VCM_start);
		pr_info("%s,otp_ptr->VCM_max:0x%x\n",__func__,otp_ptr->VCM_max);
	}
	for (count=0; count<OPT_LSC_MAX_LENGTH; count++)
	{
		pr_info("%s,otp_ptr.lenc[%d]:0x%x\n",__func__,count, otp_ptr->lenc[count]);
	}
	
	return;
}

/******************************************************************************
Function   :   ov5670_get_otp_from_sensor
Description:   read and check OTP data
******************************************************************************/
static void ov5670_get_otp_from_sensor(struct msm_sensor_ctrl_t * s_ctrl, struct OV5670_OTP_STRUCT *current_otp)
{
	bool ret = true;
	
	ov5670_otp_set_sensor_active(s_ctrl);
	ov5670_otp_disable_DPC(s_ctrl);
	
	ret = ov5670_otp_check_id(s_ctrl, current_otp);
	if (ret)
	{
		ov5670_otp_set_flag(OV5670_OTP_ID_READ_FLAG);
		pr_info("%s: check id ok!\n",__func__);
	}
	else
	{
		pr_err("%s: check id error!\n",__func__);
		goto exit;
	}

	ret = ov5670_otp_check_awb(s_ctrl, current_otp);
	if (ret)
	{
		ov5670_otp_set_flag(OV5670_OTP_AWB_READ_FLAG);
		pr_info("%s: check awb ok!\n",__func__);
	}
	else
	{
		pr_err("%s: check awb error!\n",__func__);
		goto exit;
	}
	if (ov5670_otp_is_af_module)
	{
		ret = ov5670_otp_check_vcm(s_ctrl, current_otp);
		if (ret)
		{
			ov5670_otp_set_flag(OV5670_OTP_VCM_READ_FLAG);
			pr_info("%s: check vcm ok!\n",__func__);
		}
		else
		{
			pr_err("%s: check vcm error!\n",__func__);
			goto exit;
		}
	}
	
	ret = ov5670_otp_check_lsc(s_ctrl, current_otp);
	if (ret)
	{
		ov5670_otp_set_flag(OV5670_OTP_LSC_READ_FLAG);
		pr_info("%s: check lsc ok!\n",__func__);
	}
exit:
	usleep_range(5000, 6000);
	ov5670_otp_enable_DPC(s_ctrl);
	ov5670_otp_set_sensor_standby(s_ctrl);
	
	return;
}

/******************************************************************************
Function   :    ov5670_otp_func
Description:   the interface of ov5670 OTP
******************************************************************************/
int ov5670_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index)
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

	otp_read_flag = ov5670_otp_is_af_module ? OV5670_OTP_AF_READ_ALL_FLAG : OV5670_OTP_FF_READ_ALL_FLAG;

	if (0 == (ov5670_otp_get_flag()&otp_read_flag))
	{
		ov5670_get_otp_from_sensor(s_ctrl, &g_cur_ov5670_otp_params);
		ov5670_otp_debug(&g_cur_ov5670_otp_params);
	}
	else
	{
		pr_info("%s: ov5670 OTP has already been read out!\n",__func__);
	}
	
	if (otp_read_flag != (ov5670_otp_get_flag()&otp_read_flag))
	{
		pr_err("%s: get OTP info error,ov5670_otp_read_flag = 0x%x\n", __func__,ov5670_otp_get_flag());
		return OV5670_OTP_ERROR;
	}

	ov5670_set_otp_data_to_sensor(s_ctrl, &g_cur_ov5670_otp_params);

	return OV5670_OTP_OK;	

}
