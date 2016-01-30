/*************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  File name        : Sensor_otp_common_if.c
  Version           : Initial Draft
  Date               : 2014/05/14
  Description   : this file is used by driver to check if the otp
            is supported by the driver.
  Function List    :
            is_exist_otp_function
  History            :
  1.Date            : 2014/05/14
     Modification  : Created File

*************************************************/

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_common_if"

#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

#define OV5648_SUNNY_P5V18G_RG_RATIO_TYPICAL 0x2eb
#define OV5648_SUNNY_P5V18G_BG_RATIO_TYPICAL 0x2dc

#define OV5648_SUNNY_P5V36D_RG_RATIO_TYPICAL 0x2e8
#define OV5648_SUNNY_P5V36D_BG_RATIO_TYPICAL 0x2dd

#define S5K4E1_LITEON_13P1_RG_RATIO_TYPICAL 0x2d0
#define S5K4E1_LITEON_13P1_BG_RATIO_TYPICAL 0x287

#define S5K4E1_FOXCONN_DC0301A_RG_RATIO_TYPICAL 0x34c
#define S5K4E1_FOXCONN_DC0301A_BG_RATIO_TYPICAL 0x2f3

#define OV8858_FOXCONN_RG_RATIO_TYPICAL 0x2FC
#define OV8858_FOXCONN_BG_RATIO_TYPICAL 0x2B5
//5M OV5648 Foxconn
#define OV5648_FOXCONN_SC0602_RG_RATIO_TYPICAL 0x2da
#define OV5648_FOXCONN_SC0602_BG_RATIO_TYPICAL 0x2c3
//5M s5k4e1 Sunny
#define S5K4E1_SUNNY_P5S07A_RG_RATIO_TYPICAL 0x317
#define S5K4E1_SUNNY_P5S07A_BG_RATIO_TYPICAL 0x2AE

#define OV13850_SUNNY_P13V01H_RG_RATIO_TYPICAL  0x248
#define OV13850_SUNNY_P13V01H_BG_RATIO_TYPICAL  0x257

/* Not used*/
#define IMX328_SUNNY_P13N10A_RG_RATIO_TYPICAL  0x01ED
#define IMX328_SUNNY_P13N10A_BG_RATIO_TYPICAL  0x0172

#define S5K4E1_SUNNY_132_RG_RATIO_TYPICAL            0x30c
#define S5K4E1_SUNNY_132_BG_RATIO_TYPICAL            0x2b1
#define OV5648_FOXCONN_132_RG_RATIO_TYPICAL          0x278
#define OV5648_FOXCONN_132_BG_RATIO_TYPICAL          0x2d2
#define OV5670_BYD_CHT854B_RG_RATIO_TYPICAL         0x0275  
#define OV5670_BYD_CHT854B_BG_RATIO_TYPICAL         0x02a3 

#define S5K5E2_OFILM_OHW5F02_RG_RATIO_TYPICAL         0x035c  
#define S5K5E2_OFILM_OHW5F02_BG_RATIO_TYPICAL         0x02b8   

#define OV5648_OFILM_OHW5F03_RG_RATIO_TYPICAL         0x02e8
#define OV5648_OFILM_OHW5F03_BG_RATIO_TYPICAL         0x0312
#define S5K5E2_FOXCONN_HC0806_RG_RATIO_TYPICAL		0x2e4
#define S5K5E2_FOXCONN_HC0806_BG_RATIO_TYPICAL		0x283

struct otp_function_t otp_function_lists []=
{
	{
		"ov5648_sunny_p5v18g",
		ov5648_sunny_p5v18g_otp_func,
		OV5648_SUNNY_P5V18G_RG_RATIO_TYPICAL,
		OV5648_SUNNY_P5V18G_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_sunny_p5v36d",
		ov5648_sunny_p5v18g_otp_func,
		OV5648_SUNNY_P5V36D_RG_RATIO_TYPICAL,
		OV5648_SUNNY_P5V36D_BG_RATIO_TYPICAL,
		false,
	},
	{
		"s5k4e1_liteon_13p1",
		s5k4e1_liteon_13p1_otp_func,
		S5K4E1_LITEON_13P1_RG_RATIO_TYPICAL,
		S5K4E1_LITEON_13P1_BG_RATIO_TYPICAL,
		false,
	},
	{
		"s5k4e1_foxconn_dc0301a",
		s5k4e1_liteon_13p1_otp_func,
		S5K4E1_FOXCONN_DC0301A_RG_RATIO_TYPICAL,
		S5K4E1_FOXCONN_DC0301A_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov8858_foxconn",
		ov8858_foxconn_otp_func,
		OV8858_FOXCONN_RG_RATIO_TYPICAL,
		OV8858_FOXCONN_BG_RATIO_TYPICAL,
		true,
	},

	//5M OV5648 Foxconn
	{
		"ov5648_foxconn",  //Temp
		ov5648_foxconn_sc0602_otp_func,
		OV5648_FOXCONN_SC0602_RG_RATIO_TYPICAL,
		OV5648_FOXCONN_SC0602_BG_RATIO_TYPICAL,
		false,
	},

	{
		"ov13850_sunny_p13v01h",
		ov13850_sunny_p13v01h_otp_func,
		OV13850_SUNNY_P13V01H_RG_RATIO_TYPICAL,
		OV13850_SUNNY_P13V01H_BG_RATIO_TYPICAL,
		false,
	},

	//5M s5k4e1 Sunny
	{
		"s5k4e1_sunny",  //Temp
		s5k4e1_sunny_p5s07a_otp_func,
		S5K4E1_SUNNY_P5S07A_RG_RATIO_TYPICAL,
		S5K4E1_SUNNY_P5S07A_BG_RATIO_TYPICAL,
		false,
	},
	{
		"imx328_sunny_p13n10a",
		imx328_sunny_p13n10a_otp_func,
		IMX328_SUNNY_P13N10A_RG_RATIO_TYPICAL,
		IMX328_SUNNY_P13N10A_BG_RATIO_TYPICAL,
		false,
	},
	{
		"s5k4e1_sunny_132",
		s5k4e1_sunny_132_otp_func,
		S5K4E1_SUNNY_132_RG_RATIO_TYPICAL,
		S5K4E1_SUNNY_132_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_foxconn_132",
		ov5648_foxconn_132_otp_func,
		OV5648_FOXCONN_132_RG_RATIO_TYPICAL,
		OV5648_FOXCONN_132_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_ofilm_ohw5f03",
		ov5648_ofilm_ohw5f03_otp_func,
		OV5648_OFILM_OHW5F03_RG_RATIO_TYPICAL,
		OV5648_OFILM_OHW5F03_BG_RATIO_TYPICAL,
		false,
	},
	{
	    "s5k5e2_ofilm_ohw5f02",
		s5k5e2_otp_func,
		S5K5E2_OFILM_OHW5F02_RG_RATIO_TYPICAL,
		S5K5E2_OFILM_OHW5F02_BG_RATIO_TYPICAL,
		false,
	},
	{
		"s5k5e2_foxconn_hc0806",
		s5k5e2_otp_func,
		S5K5E2_FOXCONN_HC0806_RG_RATIO_TYPICAL,
		S5K5E2_FOXCONN_HC0806_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5670_byd_cht854b",
		ov5670_otp_func,
		OV5670_BYD_CHT854B_RG_RATIO_TYPICAL,
		OV5670_BYD_CHT854B_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5670_ofilm_ohw5a03",
		ov5670_otp_func,
		OV5670_BYD_CHT854B_RG_RATIO_TYPICAL,
		OV5670_BYD_CHT854B_BG_RATIO_TYPICAL,
		false,
	}
};

/*************************************************
  Function    : is_exist_otp_function
  Description: Detect the otp we support
  Calls:
  Called By  : msm_sensor_config
  Input       : s_ctrl
  Output     : index
  Return      : true describe the otp we support
                false describe the otp we don't support

*************************************************/
bool is_exist_otp_function( struct msm_sensor_ctrl_t *s_ctrl, int32_t *index)
{
	int32_t i = 0;
	s_ctrl->afc_otp_info.starting_dac = 0;  
	s_ctrl->afc_otp_info.infinity_dac = 0;
	s_ctrl->afc_otp_info.macro_dac = 0;
	for (i=0; i<(sizeof(otp_function_lists)/sizeof(otp_function_lists[0])); ++i)
	{
		if (0 == strncmp(s_ctrl->sensordata->sensor_name, otp_function_lists[i].sensor_name, strlen(s_ctrl->sensordata->sensor_name)))
		{
			*index = i;
			pr_info("is_exist_otp_function success i = %d\n", i);
			return true;
		}
	}
	return false;
}

