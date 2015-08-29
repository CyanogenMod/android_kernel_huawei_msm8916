/*Add for huawei TP*/
/*
 * Copyright (c) 2014 Huawei Device Company
 *
 * This file provide common requeirment for different touch IC.
 * 
 * 2014-01-04:Add "tp_get_touch_screen_obj" by sunlibin
 *
 */
#ifndef __HW_TP_COMMON__
#define __HW_TP_COMMON__
/* delete 2 lines */
/*IC type*/
#define IC_TYPE_3207 3207

#define FW_OFILM_STR "000"
#define FW_EELY_STR "001"
#define FW_TRULY_STR "002"
#define FW_JUNDA_STR "005"
#define FW_GIS_STR  "004"
#define FW_YASSY_STR  "007"
#define FW_LENSONE_STR "006"

#define MODULE_STR_LEN 3

/* buffer size for dsm tp client */
#define TP_RADAR_BUF_MAX	4096

enum f54_product_module_name {
	FW_OFILM = 0,
	FW_EELY = 1,
	FW_TRULY = 2,
	/*Modify G760L tp_cap threshold get from V3*/
	FW_GIS = 4,
	FW_JUNDA = 5,
	FW_LENSONE = 6,
	FW_YASSY = 7,
	
	UNKNOW_PRODUCT_MODULE = 0xff,
};
struct holster_mode{
	unsigned long holster_enable;
	int top_left_x0;
	int top_left_y0;
	int bottom_right_x1;
	int bottom_right_y1;
};

struct kobject* tp_get_touch_screen_obj(void);
struct kobject* tp_get_virtual_key_obj(char *name);
struct kobject* tp_get_glove_func_obj(void);

/* delete the func declare */

/* add phone name so that a tp-driver can behave differentlly
accroding to different products*/
#define PHONE_NAME_Y550      "Y550"
#define PHONE_NAME_ULC02    "ULC02"
#define PHONE_NAME_ALICE	"Alice"
unsigned char already_has_tp_driver_running(void);
void set_tp_driver_running(void);
int get_tp_type(void);
void set_tp_type(int type);
#endif

