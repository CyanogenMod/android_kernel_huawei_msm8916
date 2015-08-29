/*
 *  max17048_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/irqreturn.h>
#include <linux/of_gpio.h>
#include <linux/of_batterydata.h>
#include <linux/mfd/max77819.h>
#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#include <linux/rtc.h>
#endif
/*******************************************************************************/
/* Debugging log maco defines */
#define log_level 1

#undef pr_err
#undef pr_info
#undef pr_debug
#undef  log_fmt

#define pr_err								log_err
#define pr_info								log_info
#define pr_debug							log_dbg
#define pr_vdebug							log_vdbg
#define log_fmt(format)						"Maxim:%s:%d: " format, __func__, __LINE__
/******************************************************************************/

/* registor address */
#define MAX17048_VCELL_REG					0x02
#define MAX17048_VCELL_LSB					0x03
#define MAX17048_SOC_REG					0x04
#define MAX17048_SOC_LSB					0x05
#define MAX17048_MODE_REG					0x06
#define MAX17048_MODE_LSB					0x07
#define MAX17048_VER_REG					0x08
#define MAX17048_VER_LSB					0x09
#define MAX17048_HIBRT_REG					0x0A
#define MAX17048_RCOMP_REG					0x0C
#define MAX17048_RCOMP_LSB					0x0D
#define MAX17048_CMD_REG					0xFE
#define MAX17048_CMD_LSB					0xFF
#define MAX17048_OCV_REG					0x0E
#define MAX17048_VRESET_REG					0x18
#define MAX17048_STATUS_REG					0x1A
#define MAX17048_MODEL_ACCESS_REG			0x3E
#define MAX17048_MODEL_DATA_REG_ADRR		0x40
#define MAX17048_MODEL_RCOMSEG_ADDR			0x80

#define MAX17048_MODEL_ACCESS_UNLOCK		0x4A57
#define MAX17048_MODEL_ACCESS_LOCK			0x0
#define MAX17048_MODEL_DATA_SIZE			64 		/*MAX17048_MODEL_DATA_REG_ADRR bytes (0x40 ~ 0x7f)*/
#define MAX17048_MODEL_RCOMSEG_SIZE			0x20 	/* MAX17048_MODEL_RCOMSEG_ADDR bytes (0x80 ~ 0x9f)*/
#define MAX17048_IC_STATUS_POR				0x0100	/* MAX17048_STATUS_REG por bit */
#define MAX17048_RCOMP_TEMP_CONST			200		/* 20 du */
#define MAX17048_RCOMP_TEMP_FACTORIAL		10
#define MAX17048_REG_INVALID_VALUE			0xFF
#define MAX17048_SOC_FULL					100
#define MAX17048_ALERT_SOC_LOWEST			1
#define MAX17048_ALERT_SOC_HIGHEST			32
#define MAX17048_SOC_BITS_TYPE_19			19
#define MAX17048_SOC_BITS_TYPE_18			18
#define MAX17048_ALERT_BITS					0x0020  /* MAX17048_RCOMP_REG ALERT bit */
#define MAX17048_VRESET_LOWEST_MV			2280	/* Soc reset, VRESET register value */
#define MAX17048_VRESET_HIGHEST_MV			3480
#define MAX17048_VRESET_STEP_MV				40		/* mv */
#define MAX17048_BATT_UV_PER_CELL			625/8	/* 78125/1000  = 625/8 * uv */

#define MAX17048_TIME_SECOND				1000
#define MAX17048_TIME_MINUTE				(60*MAX17048_TIME_SECOND)
#define MAX17048_TIME_HOUR					(60*MAX17048_TIME_MINUTE)
#define MAX17048_TIME_WORK_DELAY			(1 *MAX17048_TIME_SECOND) 	/* 1  sec */
#define MAX17048_DSM_WORK_TIME				(15*MAX17048_TIME_SECOND)
#define MAX17048_RCOMP_WORK_TIME			(10*MAX17048_TIME_SECOND)	/* 10 sec */
#define MAX17048_LOG_WORK_TIME				(15*MAX17048_TIME_SECOND)	/* 15 sec */
#define MAX17048_SOC_REPORT_WORK_TIME		(1 *MAX17048_TIME_SECOND)	/* 1  sec */
#define MAX17048_POR_WORK_TIME				(1 *MAX17048_TIME_SECOND)	/* 1  sec */
#define MAX17048_HANDLE_MODEL_DELAY			msecs_to_jiffies(MAX17048_TIME_HOUR) /* one hour */
#define MAX17048_SOC_ALRM_IRQ_NAME			"battery-alarm"
#define QCOM_POWR_SUPPLY_CAHRGER_NAME		"battery"
#define MAXIM_POWR_SUPPLY_CHARGER_NAME		"max77819-charger"

/* max times to retry to load model data if it is failed */
#define MAX17048_LOADMODLE_MAXTIMES			3
#define MAX17048_RCOMP_FACTORIAL			10000
#define MAX17048_VERIFY_FIX_MODEL				1
#define MAX17048_LOAD_MODEL					(!(MAX17048_VERIFY_FIX_MODEL))

#define MAX17048_CALC_SOC(socReg, bitType)\
	(((bitType) == 18) ? ((socReg) / 256) : ((socReg)/512))

/* VRESET regadrr = MAX17048_VRESET_REG */
/* VRESET regvalue use bit [7:1], step mv = MAX17048_VRESET_STEP_MV*/
#define MAX17048_VRESET_MV_TO_REGVAL(mv)\
	((((mv) /MAX17048_VRESET_STEP_MV) << 1))
	
#define MAX17048_VRESET_REGVAL_TO_MV(reg)\
	(((reg) >> 1) * MAX17048_VRESET_STEP_MV)

#define MAX17048_VBATT_REGVAL_TO_UV(reg)\
	(((u32)(reg)) * MAX17048_BATT_UV_PER_CELL)
	
#define MAX17048_RIGHT_VERSION(ver)\
	(((u16)(ver) == 0x0011) || ((u16)(ver) == 0x0012))

#define MAXIM_CURRENT_NOW_UV_TO_UA(vichguv)\
	((int)(vichguv) * 100 / 141)

/* the vichg real value is 0 when the adc read value is less than 50mv */
#define VADC_VICHG_ZERO_CHECK(vichguv)\
	((vichguv) = ((vichguv) <= 50000) ? 0 : (vichguv))

/* huawei system param define */
#define HW_VOLTAGE_FACTORIAL				1000
#define HW_BATT_FULL_SOC					100
#define HW_ALERT_LOW_SOC					2
#define HW_SYS_POWRUP_TIME					30		/* seconds */
#define HW_AVRG_BATT_MV_COUNT				3
#define HW_PROTECT_BATT_SOC					0
#define HW_PROTECT_BATT_BAD_MV 				3350	/* mv */
#define HW_PROTECT_BATT_BAD_CNT				3		/* continue times */
#define HW_CUTOFF_BATT_SOC					2
#define HW_CUTOFF_BATT_MV					3450	/* mv */
#define HW_CUTOFF_BATT_CNT					3
#define HW_MIN_SOC_GAP_SMOOTH_RISE			2		/* if > gap of display & truly soc, rise smoothly */
#define HW_SMOOTH_SOC_FALL_TIME				(3 *MAX17048_TIME_SECOND)
#define HW_SMOOTH_SOC_RISE_TIME				(6 *MAX17048_TIME_SECOND)
#define HW_REPORT_SOC_FAST_TIME				(5 *MAX17048_TIME_SECOND)
#define HW_REPORT_SOC_SLOW_TIME				(30*MAX17048_TIME_SECOND)
#define HW_REPORT_SOC_THRESHOLD				15		/* 15% */
#define HW_DANGER_USBIN_BATT_MV				3300	/* mv */
#define HW_DANGER_NOUSB_BATT_MV				3350	/* mv */

#define HIGH_WORD_VALUE(val)					((u8)((((u16)(val))&(0xFF00))>>8))
#define LOW_WORD_VALUE(val)					((u8)(((u16)(val))&(0x00FF)))
#define TWO_U8_TO_U16(high, low)				(((((u16)(high)) << 8)&(0xFF00))|((u16)(low)))

#define __lock(_chip)    mutex_lock(&(_chip)->lock)
#define __unlock(_chip)  mutex_unlock(&(_chip)->lock)
#define __lock_register(_chip)			max17048_write_reg((_chip)->client, MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_LOCK);
#define __unlock_register(_chip)		max17048_write_reg((_chip)->client, MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_UNLOCK);

#define loop_schedule_delayed_work(ptr_work, delaytime) \
		if (likely(!delayed_work_pending(ptr_work))){\
			schedule_delayed_work(ptr_work, delaytime);\
		}

static int max17048_write_reg(struct i2c_client *client, u8 reg, u16 value);
static int max17048_read_reg(struct i2c_client *client, u8 reg, u16* value);

extern bool get_max77819_charger_present(void);

enum soc_smooth_type{
	SOC_SMOOTH_NONE = 0,		/* no rise or fall */
	SOC_SMOOTH_FALL,			/* soc fall smoothly */
	SOC_SMOOTH_RISE,			/* soc rise smoothly */
	SOC_SMOOTH_SHUT,			/* soc jumps to zero to shut system down at once */
};

struct soc_smooth_info{
	enum soc_smooth_type smooth_type;
	u8 end_soc;					/* the soc will rises or falls to */
};

struct low_soc_check_info{
	int danger_mv;				/* the battery voltage low this threshold, must shut down device at once */
	int low_mv;					/* the battery low voltage threshold */
	u8 low_soc;					/* the battery low soc threshold */
	int check_count;			/* > this num, the low soc state changes */
};

struct max17048_platform_data{
	unsigned int			irq_gpio;
	u8						alert_soc;
	int						reset_mvlolt;
	u16						cutoff_batt_mv;
	u8						cutoff_batt_soc;
	u16						bad_batt_mv;
	struct max17048_batt_data*		pbat_data;
};

struct max17048_chip {
	struct mutex						lock;
	struct i2c_client*					client;
	struct delayed_work					work;
	struct delayed_work					hand_work;
	struct delayed_work					notifier_work;

	struct power_supply					fgbattery;
	struct power_supply*				psy_qcom_charger;

	struct qpnp_vadc_chip*				vadc_dev;

	struct max17048_platform_data*		pdata;
	struct soc_smooth_info				soc_smooth;

	bool	bbatt_alrm;
	int		irq;
	/* battery voltage */
	int		vcell;
	/* battery capacity */
	u8		soc;
#ifdef CONFIG_HUAWEI_DSM
	int		saved_soc;
#endif
};

static struct max17048_chip* global_chip = NULL;
/* check whether the system boots up by factory mode */
static bool factory_flag = false;

static struct max17048_batt_data gbat_ini_default = {
	.full_capacity = 3000000,	/* uAh */
	.ini_rcompseg = 0x0080,
	.ini_rcomp = 97,
	.ini_tempco_up = -1000,		/* -0.1 * 10000 */
	.ini_tempco_dwon = 74750,	/* 7.475 * 10000 */
	.ini_soccheck_a = 226,
	.ini_soccheck_b = 228,
	.ini_ocvtest = 57952,
	.ini_bits = 19,
	.model_data = {
		0xA9, 0xD0, 0xB7, 0x20, 0xB8, 0x90, 0xBA, 0xB0,
		0xBB, 0xD0, 0xBC, 0xE0, 0xBE, 0x00, 0xBF, 0x00,
		0xC0, 0x40, 0xC2, 0x30, 0xC4, 0xA0, 0xC6, 0x60,
		0xCA, 0x00, 0xCD, 0x70, 0xD2, 0x20, 0xD8, 0x60,
		0x02, 0x80, 0x26, 0x60, 0x1B, 0xB0, 0x27, 0xE0,
		0x3C, 0x80, 0x3B, 0xF0, 0x2B, 0xC0, 0x24, 0xD0,
		0x10, 0xE0, 0x16, 0xD0, 0x14, 0x30, 0x11, 0x80,
		0x10, 0x10, 0x0F, 0xB0, 0x0B, 0x00, 0x0B, 0x00,
	},
};

static struct max17048_platform_data gplatform_data_default = {
	.irq_gpio = 109,
	.alert_soc = HW_ALERT_LOW_SOC,
	.reset_mvlolt = 2800, /* 2.8v */
	.cutoff_batt_mv = HW_CUTOFF_BATT_MV,
	.cutoff_batt_soc = HW_CUTOFF_BATT_SOC,
	.bad_batt_mv = HW_PROTECT_BATT_BAD_MV,
	.pbat_data = &gbat_ini_default,
};

static enum power_supply_property max17048_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

#ifdef CONFIG_HUAWEI_DSM
extern struct dsm_client *bms_dclient;
extern struct qpnp_lbc_chip *g_lbc_chip;
#define ONE_MINUTE		60 //60 seconds
#define SOC_NORMAL_DELTA		5 //change 5 percent
#define MAX17048_TOO_FULL					105
extern int dump_registers_and_adc(struct dsm_client *dclient, struct qpnp_lbc_chip *chip, int type);

/* dump max17048 main 11 regs*/
void max17048_fgauge_dump_regs(struct dsm_client *dclient)
{
    int ret = 0;
    u8 reg = 0;
    u16 reg_val = 0;

    if (!global_chip)
    {
        pr_info("max17048 chip device not init,do nothing!\n");
        return;
    }
    dsm_client_record(dclient, "[MAX17048 fgauge] regs:\n");
    log_vdbg("[MAX17048 fgauge] regs:\n");
    for(reg = MAX17048_VCELL_REG; reg < MAX17048_VRESET_REG; reg=reg+2){
        ret = max17048_read_reg(global_chip->client, reg, &reg_val);
        if (ret >= 0)
        {
            dsm_client_record(dclient, "0x%x, 0x%x\n", reg, reg_val);
            log_vdbg("0x%x, 0x%x\n", reg, reg_val);
        }
    }
}
EXPORT_SYMBOL(max17048_fgauge_dump_regs);
#endif

static int __init early_parse_factory_flag(char* p)
{
	if(p && !strcmp(p,"factory"))
	{
		factory_flag = true;
	}
	return 0;
}
early_param("androidboot.huawei_swtype",early_parse_factory_flag);

/* Use another mothed to reduce the power consumption of FTM mode. */

static int max17048_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, reg, swab16(value));

	if (ret < 0)
		pr_err("reg %d err %d\n", reg, ret);

	return ret;
}

static int max17048_read_reg(struct i2c_client *client, u8 reg, u16* pvalue)
{
	s32 ret = 0;

	if (!pvalue)
	{
		return -EIO;
	}

	ret = i2c_smbus_read_word_data(client, reg);

#ifdef CONFIG_HUAWEI_DSM
	/* if max17048 i2c read failed, record this log, and notify to the dsm server*/
	if(ret < 0){
		if(!dsm_client_ocuppy(bms_dclient)){
			dsm_client_record(bms_dclient, "fuel gauge max17048 i2c read failed!\n");
			dsm_client_notify(bms_dclient, DSM_FUEL_GAUGE_I2C_ERROR);
		}
	}
#endif

	if (ret < 0)
	{
		pr_err("reg %d err %d\n", reg, ret);
	}
	else
	{
		*pvalue = swab16((u16)ret);
	}

	return ret;
}

#define max17048_get_ext_property(psy, prop, valptr) \
({\
	int __rc = -ENODEV;\
	if (psy)\
	{\
		__rc = psy->get_property(psy, prop, valptr);\
		if (unlikely(IS_ERR_VALUE(__rc)))\
		{\
			pr_err("get property:%d failed!!\n", prop);\
		}\
	}\
	else\
	{\
		pr_err("power_supply is null !!\n");\
	}\
	__rc;\
})

#define max17048_qcom_charger_get_property(_chip, prop, valptr) \
({\
	if (!(_chip)->psy_qcom_charger)\
	{\
		(_chip)->psy_qcom_charger = \
			power_supply_get_by_name(QCOM_POWR_SUPPLY_CAHRGER_NAME);\
	}\
	max17048_get_ext_property((_chip)->psy_qcom_charger, prop, valptr);\
})

static int max17048_maxim_charger_get_property(
				struct max17048_chip* chip,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	static struct power_supply* pmaxim_charger = NULL;
	if (!pmaxim_charger)
	{
		pmaxim_charger = power_supply_get_by_name(MAXIM_POWR_SUPPLY_CHARGER_NAME);\
	}
	return max17048_get_ext_property(pmaxim_charger, POWER_SUPPLY_PROP_STATUS, val);
}

static int max17048_get_battery_temp(struct max17048_chip* chip)
{
	int rc = 0;
	int temp = 250;
	union power_supply_propval val = {0};
	rc = max17048_qcom_charger_get_property(chip, POWER_SUPPLY_PROP_TEMP, &val);
	if (likely(!IS_ERR_VALUE(rc)))
	{
		temp = val.intval;
	}
	return temp;
}

static bool max17048_get_battery_present(struct max17048_chip* chip)
{
	int rc = 0;
	bool present = false;
	union power_supply_propval val = {0};
	rc = max17048_qcom_charger_get_property(chip, POWER_SUPPLY_PROP_PRESENT, &val);
	if (likely(!IS_ERR_VALUE(rc)))
	{
		present = !!val.intval;
	}
	return present;
}

static int max17048_get_charging_status(struct max17048_chip* chip)
{
	int rc = 0;
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	union power_supply_propval val = {0};
	rc = max17048_maxim_charger_get_property(chip, POWER_SUPPLY_PROP_STATUS, &val);
	if (likely(!IS_ERR_VALUE(rc)))
	{
		status = val.intval;
	}
	return status;
}

/* check the system powers up or not: true = power up, false = powering */
static bool max17048_is_power_up_status(struct max17048_chip* chip)
{
	struct timespec kernel_time;
	static bool power_up_flag = false;

	if(!power_up_flag)
	{
		/*  kernel time is less than VOLTAE_CHECK_BEGAIN_SECOND, system in power up status;
		    kernel time is greater than VOLTAE_CHECK_BEGAIN_SECOND, set flag true*/
		ktime_get_ts(&kernel_time);
		if(kernel_time.tv_sec < HW_SYS_POWRUP_TIME)
		{
			log_vdbg("sytem in power_up status ,kernel_time is %ld \n",
					kernel_time.tv_sec);
			power_up_flag = false;
		}
		else
		{
			log_vdbg("system have power up !!\n");
			power_up_flag = true;
		}
	}
	return power_up_flag;
}

static int max17048_prepare_load_model(struct max17048_chip* chip)
{
	int ret = 0;
	u8 OCV_1,OCV_2;
	u16 msb = 0;
	u16 check_times = 0;
	do {
		msleep(100);
		//Step1:unlock model access, enable access to OCV and table registers
		max17048_write_reg(chip->client, 
			MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_UNLOCK);

		//Step2:Read OCV, verify Model Access Unlocked  
		ret = max17048_read_reg(chip->client, MAX17048_OCV_REG, &msb);//read OCV
		OCV_1 = HIGH_WORD_VALUE(msb);//"big endian":low byte save to MSB
		OCV_2 = LOW_WORD_VALUE(msb);

		if(check_times++ >= MAX17048_LOADMODLE_MAXTIMES) {//avoid of while(1)
			pr_info("read ocv reg time out ...");
			return (-EIO);
		}
	}while ((ret < 0) || 
		((OCV_1==MAX17048_REG_INVALID_VALUE) && (OCV_2==MAX17048_REG_INVALID_VALUE)));//verify Model Access Unlocked
	return msb;
}

static void max17048_dump_model_regs(struct max17048_chip* chip)
{
	u16 value = 0;
	u8 k = 0;

	for (k=0; k < MAX17048_MODEL_DATA_SIZE; k+=2)
	{
		max17048_read_reg(chip->client, (MAX17048_MODEL_DATA_REG_ADRR+k), &value);
		pr_debug("model_data[0x%02x] = 0x%04x \n", (MAX17048_MODEL_DATA_REG_ADRR+k), value);
	}

	//read RCOMPSeg (for MAX17048/MAX17049 only)
	for (k=0; k < MAX17048_MODEL_RCOMSEG_SIZE; k+=2)
	{
		max17048_read_reg(chip->client, (MAX17048_MODEL_RCOMSEG_ADDR+k), &value);
		pr_debug("rcomp_seg[0x%02x] = 0x%04x \n", (MAX17048_MODEL_RCOMSEG_ADDR+k), value);
	}
}

void max17048_load_model(struct max17048_chip* chip) {	
   	/******************************************************************************
	Step 5. Write the Model
	Once the model is unlocked, the host software must write the 64 byte model
	to the device. The model is located between memory 0x40 and 0x7F.
	The model is available in the INI file provided with your performance
	report. See the end of this document for an explanation of the INI file.
	Note that the table registers are write-only and will always read
	0xFF. Step 9 will confirm the values were written correctly.
	*/
	u8 k=0;
	u16 value = 0;
	u8* pmodel_data = chip->pdata->pbat_data->model_data;
	u16 rcomseg = chip->pdata->pbat_data->ini_rcompseg;
	//Once the model is unlocked, the host software must write the 64 bytes model to the device
	for (k=0; k < MAX17048_MODEL_DATA_SIZE; k+=2)
	{
		value = TWO_U8_TO_U16(pmodel_data[k], pmodel_data[k+1]);
		//The model is located between memory 0x40 and 0x7F
		max17048_write_reg(chip->client, 
			(MAX17048_MODEL_DATA_REG_ADRR+k), value);
	}

	//Write RCOMPSeg (for MAX17048/MAX17049 only)
	for (k=0; k < MAX17048_MODEL_RCOMSEG_SIZE; k+=2)
	{
	    max17048_write_reg(chip->client,
			(MAX17048_MODEL_RCOMSEG_ADDR+k), rcomseg);
	}

	if (log_level >= 2)
	{
		max17048_dump_model_regs(chip);
	}
}

bool max17048_verify_model_is_correct(struct max17048_chip* chip) 
{
	u8 SOC_1, SOC_2;
	u16 msb;

	msleep(200);//Delay at least 150ms(max17048/1/3/4 only)

	//Step 7. Write OCV:write(reg[0x0E], INI_OCVTest_High_Byte, INI_OCVTest_Low_Byte)
	max17048_write_reg(chip->client,MAX17048_OCV_REG, 
		chip->pdata->pbat_data->ini_ocvtest);

	//Step 7.1 Disable Hibernate (MAX17048/49 only)
	max17048_write_reg(chip->client,MAX17048_HIBRT_REG,0x0);

	//Step 7.2. Lock Model Access (MAX17048/49/58/59 only)
	max17048_write_reg(chip->client, MAX17048_MODEL_ACCESS_REG, 
		MAX17048_MODEL_ACCESS_LOCK);

	//Step 8: Delay between 150ms and 600ms, delaying beyond 600ms could cause the verification to fail
	msleep(500);
 
	//Step 9. Read SOC register and compare to expected result
	if (max17048_read_reg(chip->client, MAX17048_SOC_REG, &msb) < 0)
	{
		pr_info("Read soc failed, model data was NOT loaded successfully!\n");
		return false; 
	}

	SOC_1 = HIGH_WORD_VALUE(msb);//"big endian":low byte save MSB
	SOC_2 = LOW_WORD_VALUE(msb);

	pr_debug("soc1=%d, soc2=%d, checka=%d, checkb=%d\n", SOC_1,SOC_2,
			chip->pdata->pbat_data->ini_soccheck_a,
			chip->pdata->pbat_data->ini_soccheck_b);
	if(SOC_1 >= chip->pdata->pbat_data->ini_soccheck_a &&
	SOC_1 <= chip->pdata->pbat_data->ini_soccheck_b) {
		pr_info("model data was loaded successfully!\n");
		return true;
	}
	else {
		pr_info("model data was NOT loaded successfully!\n");
		return false; 
	}
}

void max17048_cleanup_model_load(struct max17048_chip* chip, u16 original_ocv) 
{
	u16 recom_alrt = TWO_U8_TO_U16(
		(chip->pdata->pbat_data->ini_rcomp),
		(MAX17048_ALERT_SOC_HIGHEST - chip->pdata->alert_soc));

	//step9.1, Unlock Model Access (MAX17048/49/58/59 only): To write OCV, requires model access to be unlocked
	max17048_write_reg(chip->client,
		MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_UNLOCK);

	//step 10 Restore CONFIG and OCV: write(reg[0x0C], INI_RCOMP, Your_Desired_Alert_Configuration)
	max17048_write_reg(chip->client, MAX17048_RCOMP_REG,  recom_alrt);//RCOMP0=94 , battery empty Alert threshold = 4% -> 0x1C
	max17048_write_reg(chip->client, MAX17048_OCV_REG, original_ocv); 

	//step 10.1 Restore Hibernate (MAX17048/49 only)
	/* disable Hibernate */
	max17048_write_reg(chip->client, MAX17048_HIBRT_REG, 0x0);

	//step 11 Lock Model Access
	max17048_write_reg(chip->client,
		MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_LOCK);
	//step 12,//delay at least 150ms before reading SOC register
	mdelay(200); 
}

/*
RI (reset indicator) is set when the device powers
up. Any time this bit is set, the IC is not configured,
so the model should be loaded and the bit should
be cleared.
*/
bool max17048_check_por(struct max17048_chip* chip)
{
	int ret = 0;
	u16 status = 0;
	//firstly check POR
	ret = max17048_read_reg(chip->client, MAX17048_STATUS_REG, &status);
	if (ret < 0)
	{
		pr_err("check por failed, rc = %d \n", ret);
		return false;
	}
	return (!!(status & MAX17048_IC_STATUS_POR));  //if por is not set,do nothing
}

void max17048_clear_por(struct max17048_chip* chip)
{
	int ret = 0;
	u16 status = 0;
	ret = max17048_read_reg(chip->client, MAX17048_STATUS_REG, &status);
	if (ret >= 0)
	{
		status = status & (~MAX17048_IC_STATUS_POR);
		max17048_write_reg(chip->client,MAX17048_STATUS_REG,status);
	}
}

bool max17048_handle_model(struct max17048_chip* chip, int load_or_verify) 
{
	int ret = 0;
	bool model_load_ok = false;
	u16 check_times = 0;
	u16 msb_original = 0;
	
	//remember the OCV
	ret = max17048_prepare_load_model(chip);
	if (ret < 0){
		pr_err("read the original OCV failed!! \n");
		goto out;
	}
	msb_original = (u16)ret;

	check_times = 0;
	do {
		// Steps 1-4		
		max17048_prepare_load_model(chip);
		if (load_or_verify == MAX17048_LOAD_MODEL) {		
			// Step 5
			max17048_load_model(chip);
		}

		// Steps 6-9
		model_load_ok = max17048_verify_model_is_correct(chip);
		if (!model_load_ok) {
			load_or_verify = MAX17048_LOAD_MODEL;
		}

		if (check_times++ >= MAX17048_LOADMODLE_MAXTIMES) {
			pr_err("max17048 handle model :time out1...");
			goto out;
		}
	} while (!model_load_ok);

	// Steps 10-12
	max17048_cleanup_model_load(chip, msb_original);

	//clear up por
		//firstly check POR
	if(max17048_check_por(chip)){
		max17048_clear_por(chip);
	}
out:
#ifdef CONFIG_HUAWEI_DSM
	/* if max17048 handle model failed, record this log, and notify to the dsm server*/
	if(!model_load_ok){
		if(!dsm_client_ocuppy(bms_dclient)){
			dsm_client_record(bms_dclient, "max17048 handle model failed\n");
			dsm_client_notify(bms_dclient, DSM_MAXIM_HANDLE_MODEL_FAIL);
		}
	}
#endif
	return model_load_ok;
}

void max17048_update_rcomp(struct max17048_chip *chip) 
{
	u16 cfg=0;
	int temp = 250;

	//int NewRCOMP = INI_RCOMP;
	u8 new_rcomp = 0;
	int calc_rcomp = 0;
	u8 ini_rcomp = chip->pdata->pbat_data->ini_rcomp;
	int ini_tempco_up = chip->pdata->pbat_data->ini_tempco_up;
	int ini_tempco_down = chip->pdata->pbat_data->ini_tempco_dwon;
	
	/*get battery temperature*/
	temp = max17048_get_battery_temp(chip);

	if(temp > MAX17048_RCOMP_TEMP_CONST) {
		calc_rcomp = ini_rcomp + (temp -MAX17048_RCOMP_TEMP_CONST) *
			ini_tempco_up/(MAX17048_RCOMP_FACTORIAL * MAX17048_RCOMP_TEMP_FACTORIAL);
	} 
	else if(temp < MAX17048_RCOMP_TEMP_CONST) {
		calc_rcomp = ini_rcomp + (temp -MAX17048_RCOMP_TEMP_CONST) * 
			ini_tempco_down/(MAX17048_RCOMP_FACTORIAL * MAX17048_RCOMP_TEMP_FACTORIAL);
	} 

	if(calc_rcomp > MAX17048_REG_INVALID_VALUE){
		calc_rcomp = MAX17048_REG_INVALID_VALUE;
	}
	else if(calc_rcomp < 0){
		calc_rcomp = 0;
	}
	/*now update it to register*/
	if (max17048_read_reg(chip->client, MAX17048_RCOMP_REG, &cfg) >= 0)
	{
		new_rcomp = (u8)calc_rcomp;
		cfg = TWO_U8_TO_U16(new_rcomp, LOW_WORD_VALUE(cfg));
		max17048_write_reg(chip->client, MAX17048_RCOMP_REG, cfg);
	}
	msleep(150);
}

static int max17048_get_vcell_batt_uv(struct max17048_chip *chip)
{
	int ret = 0;
	u16 fg_vcell = 0;
	u32 vcell_uV = 0;

	ret = max17048_read_reg(chip->client, MAX17048_VCELL_REG, &fg_vcell);
	if (ret >= 0)
	{
		vcell_uV = MAX17048_VBATT_REGVAL_TO_UV(fg_vcell);
		chip->vcell = vcell_uV;
		log_vdbg("max17048:chip->vcell = %duV\n", vcell_uV);
		return vcell_uV;
	}
	pr_err("max17048:chip get vcell error rc = %d \n", ret);
	return ret;
}

static int max17048_avarage_vbat_mv(struct max17048_chip *chip, int avrg_count)
{
#define HW_AVRG_BATMV_COUNT_RIGHT(cnt)	((cnt) = min(128, max((cnt), 1)))

	int cnt = 0;
	int vbat_uv = 0;
	int sum = 0;

	/* sample_count scope is 1 -128 */
	HW_AVRG_BATMV_COUNT_RIGHT(avrg_count);

	for(cnt = 0; cnt < avrg_count ; cnt++)
	{
		vbat_uv = max17048_get_vcell_batt_uv(chip);
		if (vbat_uv < 0)
		{
			pr_debug("get batt mv failed, break count = %d \n", cnt);
			break;
		}
		sum += vbat_uv;
	}
	
	return ((0 == cnt) ? (-EIO) :  (sum / cnt / HW_VOLTAGE_FACTORIAL));
}

static u8 max17048_get_soc(struct max17048_chip *chip)
{
	int ret = 0;
	u16 fg_soc = 0, ret_soc = 0;
	int ini_bits = chip->pdata->pbat_data->ini_bits;

	__lock(chip);
	ret = max17048_read_reg(chip->client, MAX17048_SOC_REG, &fg_soc);
	if (unlikely(ret < 0))
	{
		ret_soc = chip->soc;
		pr_err("get fgague soc failed, return the pre-soc = %d, rc = %d\n", chip->soc, ret);
		goto end_out;
	}
	fg_soc = MAX17048_CALC_SOC(fg_soc, ini_bits);

#ifdef CONFIG_HUAWEI_DSM
	/* if origin soc read from soc reg is high than 105%, notify to server*/
	if(MAX17048_TOO_FULL < fg_soc){
		if(!dsm_client_ocuppy(bms_dclient)){
			dsm_client_record(bms_dclient, "fg_soc over 100 percent, fg_soc=%d\n", fg_soc);
			dsm_client_notify(bms_dclient, DSM_MAXIM_REAL_SOC_OVER_100);
		}
	}
#endif

	ret_soc = min((int)fg_soc, MAX17048_SOC_FULL);
	log_vdbg("max17048:updated soc:%d, fg_soc:%d, bits:%d\n", ret_soc, fg_soc, ini_bits);

end_out:
	__unlock(chip);
	return ret_soc;
}

static int max17048_get_charge_current_now(struct max17048_chip *chip)
{
#define MAXIM_VICHG_ADC_CHAN P_MUX2_1_3
	int rc;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, MAXIM_VICHG_ADC_CHAN, &results);
	if (rc) {
		pr_err("Unable to read charge current now !! rc=%d\n", rc);
		return -1;
	}
	VADC_VICHG_ZERO_CHECK(results.physical);

	return MAXIM_CURRENT_NOW_UV_TO_UA(results.physical);
}

static void max17048_soc_factory_check(struct max17048_chip* chip)
{
	/* In factory system mode, reporting the soc 1% when the soc fall to 0% */
	if (factory_flag && 
		(0 == chip->soc) && 
		get_max77819_charger_present())
	{
		chip->soc = 1;
		pr_info("In factory system mode, reporting the soc 1 when the soc fall to 0 \n");
	}
}

static int max17048_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int rc = 0;
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, fgbattery);

	__lock(chip);
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		max17048_get_vcell_batt_uv(chip);
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* remove max17048_get_soc(chip); */
		max17048_soc_factory_check(chip);
		val->intval = chip->soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->pdata->pbat_data->full_capacity;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = max17048_get_charge_current_now(chip);
		break;
	default:
		rc = -EINVAL;
	}
	__unlock(chip);
	return rc;
}

static u16 max17048_get_version(struct i2c_client *client)
{
	u16 chip_version = 0;

	max17048_read_reg(client, MAX17048_VER_REG, &chip_version);

	pr_info("max17048 Fuel-Gauge Ver 0x%04x\n", chip_version);
	return chip_version;
}

static void max17048_init(struct max17048_chip *chip)
{
	u8 ini_rcomp = 0;	
	u8 init_alert = 0;
	u16 init_cfg = 0;
	int mvolt = 0;
	u16  vreset = 0;

	if (!chip->pdata){
		pr_err("max17048_platform_data is not inited!!\n");
		return;
	}
	/*config rcomp and alert register*/
	ini_rcomp = chip->pdata->pbat_data->ini_rcomp;
	init_alert = chip->pdata->alert_soc;
	/* if bits type is 19, the alert set value must be double */
	if (MAX17048_SOC_BITS_TYPE_19 == chip->pdata->pbat_data->ini_bits)
	{
		init_alert = init_alert * 2;
	}
	if((init_alert > MAX17048_ALERT_SOC_HIGHEST)||
		(init_alert < MAX17048_ALERT_SOC_LOWEST))
	{
		init_alert = HW_ALERT_LOW_SOC;
	}

	init_cfg = TWO_U8_TO_U16(ini_rcomp, (MAX17048_ALERT_SOC_HIGHEST-init_alert));
	max17048_write_reg(chip->client, MAX17048_RCOMP_REG, init_cfg);	

	/*config vreset */
	mvolt = chip->pdata->reset_mvlolt;
	mvolt = min(mvolt, MAX17048_VRESET_HIGHEST_MV);
	mvolt = max(mvolt, MAX17048_VRESET_LOWEST_MV);
	
	vreset = MAX17048_VRESET_MV_TO_REGVAL(mvolt);
	vreset = vreset << 8;
	max17048_write_reg(chip->client, MAX17048_VRESET_REG, vreset);
}

#ifdef CONFIG_HUAWEI_DSM
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

/* monitor whether normal running soc jumps more than 5 percent in 1 minute*/
static void monitor_normal_soc_jump(struct max17048_chip *chip)
{
	static int normal_soc_cal_flag = 0;
	static unsigned long start_tm_sec = 0;
	unsigned long now_tm_sec = 0;
	int temp_soc;

	temp_soc = max17048_get_soc(chip);
	/* if the soc changed more than 5 percent in 1 minute*/
	/*dump maxim registers and adc values, and notify to dsm server */
	if(!normal_soc_cal_flag){
		get_current_time(&start_tm_sec);
		chip->saved_soc = temp_soc;
		normal_soc_cal_flag = 1;
	}

	get_current_time(&now_tm_sec);
	if(ONE_MINUTE >= (now_tm_sec - start_tm_sec)){
		if(abs(chip->saved_soc - temp_soc) > SOC_NORMAL_DELTA){
			pr_info("soc changed more than 5 percent during recent one minute "
				"chip->saved_soc=%d new_soc=%d\n",
				chip->saved_soc,
				temp_soc);
			dump_registers_and_adc(bms_dclient, g_lbc_chip, DSM_BMS_NORMAL_SOC_CHANGE_MUCH);
		}
	}else{
		normal_soc_cal_flag = 0;
		start_tm_sec = 0;
	}
}
#endif

static void max17048_relax_pm_by_soc(struct max17048_chip* chip)
{
	/* If battery alarm is enabled, but we charge the battery again at alert_soc,*/
	/* we should release the pm lock to allow the phone to sleep when soc -gt alert_soc */
	if (chip->bbatt_alrm && (chip->soc > chip->pdata->alert_soc))
	{
		chip->bbatt_alrm = false;
		pm_relax(&(chip->client->dev));
	}
}

static bool max17048_time_to_report(struct max17048_chip* chip)
{
	static int pre_type = -1;
	static int report_count = 0;
	int report_time_num = 0;

	/* smooth type change, to recount time */
	if (pre_type != chip->soc_smooth.smooth_type)
	{
		report_count = 0;
		pre_type = chip->soc_smooth.smooth_type;
	}

	if (SOC_SMOOTH_FALL == chip->soc_smooth.smooth_type)
	{
		report_time_num = 
			HW_SMOOTH_SOC_FALL_TIME / MAX17048_SOC_REPORT_WORK_TIME;
	}
	else if (SOC_SMOOTH_RISE == chip->soc_smooth.smooth_type)
	{
		report_time_num = 
			HW_SMOOTH_SOC_RISE_TIME / MAX17048_SOC_REPORT_WORK_TIME;
	}
	else if (SOC_SMOOTH_SHUT == chip->soc_smooth.smooth_type)
	{
		report_time_num = 0; /* report the soc at once to shut down the system */
	}
	/* if soc less than 15%, every 5s reports soc, or else every 30s reports soc */
	else if (chip->soc <= HW_REPORT_SOC_THRESHOLD)
	{
		report_time_num = 
			HW_REPORT_SOC_FAST_TIME / MAX17048_SOC_REPORT_WORK_TIME;
	}
	else
	{
		report_time_num = 
			HW_REPORT_SOC_SLOW_TIME / MAX17048_SOC_REPORT_WORK_TIME;
	}

	report_count++;
	if (report_count < report_time_num)
	{
		return false;
	}
	else
	{
		report_count = 0;
		return true;
	}
}

static void max17048_soc_smooth_fall(struct max17048_chip* chip)
{
	u8 fg_soc;

	if (SOC_SMOOTH_FALL == chip->soc_smooth.smooth_type)
	{
		if (chip->soc > chip->soc_smooth.end_soc)
		{
			chip->soc--;
			if (chip->soc == chip->soc_smooth.end_soc)
			{
				pr_info("soc smooth fall end to %d !!\n", chip->soc);
			}
		}
		fg_soc = max17048_get_soc(chip);
		if (fg_soc == chip->soc)
		{
			chip->soc_smooth.smooth_type = SOC_SMOOTH_NONE;
			pr_info("soc fall same to truly soc %d, no need smoothly!!\n", fg_soc);
		}
		else
		{
			if (fg_soc < chip->soc_smooth.end_soc)
			{
				pr_info("The soc fall rise from %d to %d!!\n", chip->soc, fg_soc);
				chip->soc_smooth.end_soc = fg_soc;
			}
			else
			{
				/* do nothing, wait the fg_soc fall itself */
			}
		}
	}
}

static void max17048_soc_smooth_rise(struct max17048_chip* chip)
{
	u8 fg_soc;
	if (SOC_SMOOTH_RISE == chip->soc_smooth.smooth_type)
	{
		if (chip->soc < chip->soc_smooth.end_soc)
		{
			chip->soc++;
			if (chip->soc >= chip->soc_smooth.end_soc)
			{
				pr_info("soc smooth rise end to %d !!\n", chip->soc);
			}
		}
		fg_soc = max17048_get_soc(chip);
		if (fg_soc == chip->soc)
		{
			chip->soc_smooth.smooth_type = SOC_SMOOTH_NONE;
			pr_info("soc rises same to truly soc %d, no need smoothly!!\n", fg_soc);
		}
		else
		{
			if (fg_soc > chip->soc_smooth.end_soc)
			{
				pr_info("The soc will rise from %d to %d!!\n", chip->soc, fg_soc);
				chip->soc_smooth.end_soc = fg_soc;
			}
			else if (fg_soc < chip->soc_smooth.end_soc)
			{
				if (chip->soc < chip->soc_smooth.end_soc)
				{
					pr_info("The soc will rise from %d to %d!!\n", chip->soc, fg_soc);
					chip->soc_smooth.end_soc = fg_soc;
				}
				else
				{
					/* do nothing, wait the fg_soc rise itself */
				}
			}
			else
			{
				/* do nothing */
			}
		}
	}
}

static struct low_soc_check_info* 
max17048_get_soc_check_info(struct max17048_chip* chip, bool cable_in)
{
	static struct low_soc_check_info low_info;
	if (cable_in)
	{
		low_info.danger_mv = HW_DANGER_USBIN_BATT_MV;
		low_info.low_mv = chip->pdata->bad_batt_mv;
		low_info.low_mv = max(low_info.low_mv, low_info.danger_mv);
		low_info.low_soc = HW_PROTECT_BATT_SOC;
		low_info.check_count = HW_PROTECT_BATT_BAD_CNT;
	}
	else
	{
		low_info.danger_mv = HW_DANGER_NOUSB_BATT_MV;
		low_info.low_mv = chip->pdata->cutoff_batt_mv;
		low_info.low_mv = max(low_info.low_mv, low_info.danger_mv);
		low_info.low_soc = chip->pdata->cutoff_batt_soc;
		low_info.check_count = HW_CUTOFF_BATT_CNT;
	}
	return &low_info;
}

static void max17048_low_soc_smooth_info(
			struct max17048_chip* chip, bool cable_in, int chg_status)
{
	struct low_soc_check_info* plow_info = NULL;
	static int low_count = 0, upr_count = 0, dgr_count = 0;
	static bool pre_chg_present = false;
	int bat_mv;
	u8  fg_soc;

	/* usb cable plug out or in, reset the count num */
	if (pre_chg_present != cable_in)
	{
		pre_chg_present = cable_in;
		low_count = 0;
		upr_count = 0;
		dgr_count = 0;
	}

	/* The system in power up state, no need to check low soc */
	if (cable_in && !max17048_is_power_up_status(chip))
	{
		pr_vdebug("system is powering up & cable is in, no check low soc !! \n");
		return;
	}

	/* get the low soc check info */
	plow_info = max17048_get_soc_check_info(chip, cable_in);

	/* get the battery voltage & check the low voltage and soc */
	bat_mv = max17048_avarage_vbat_mv(chip, HW_AVRG_BATT_MV_COUNT);

	/* get the battery voltage error */
	if (bat_mv <= 0)
	{
		pr_err("get battery voltage failed rc =%d, skip checking smooth info !!", bat_mv);
		return;
	}

	if (bat_mv < plow_info->danger_mv)
	{
		low_count++;
		dgr_count++;
		upr_count = 0;
	}
	else if (bat_mv <= plow_info->low_mv)
	{
		low_count++;
		upr_count = 0;
		dgr_count = 0;
	}
	else if (bat_mv > plow_info->low_mv)
	{
		upr_count++;
		low_count = 0;
		dgr_count = 0;
	}

	if (dgr_count >= plow_info->check_count)
	{
		dgr_count = plow_info->check_count;
		chip->soc_smooth.smooth_type = SOC_SMOOTH_SHUT;
		chip->soc_smooth.end_soc = plow_info->low_soc;
		pr_info("The battery voltge < %d, report soc to %d \n",
					plow_info->danger_mv, chip->soc_smooth.end_soc);
	}
	else if (low_count >= plow_info->check_count && chip->soc > plow_info->low_soc)
	{
		chip->soc_smooth.smooth_type = SOC_SMOOTH_FALL;
		if ((low_count == plow_info->check_count) ||
			(chip->soc_smooth.end_soc != plow_info->low_soc))
		{
			chip->soc_smooth.end_soc = plow_info->low_soc;
			pr_info("The battery voltge < %d, the soc will fall from %d to %d \n",
					plow_info->low_mv, chip->soc, chip->soc_smooth.end_soc);
		}
		low_count = plow_info->check_count;
	}
	else if (upr_count >= plow_info->check_count)
	{
		chip->soc_smooth.smooth_type = SOC_SMOOTH_NONE;		/* clear the smooth fall state */
		fg_soc = max17048_get_soc(chip);
		if ((chip->soc < plow_info->low_soc) ||
			((fg_soc - chip->soc) >= HW_MIN_SOC_GAP_SMOOTH_RISE))
		{
			fg_soc = max((u8)(plow_info->low_soc + 1), fg_soc);
			chip->soc_smooth.smooth_type = SOC_SMOOTH_RISE;
			if ((upr_count == plow_info->check_count) ||
				(chip->soc_smooth.end_soc != fg_soc))
			{
				chip->soc_smooth.end_soc = fg_soc;
				pr_info("The battery voltage > %d, the soc will rise from %d to %d \n",
						plow_info->low_mv, chip->soc, chip->soc_smooth.end_soc);
			}
		}
		else
		{
			/* do nothing */
		}
		upr_count = plow_info->check_count;
	}
	else
	{
		/* do nothing */
	}
}

static void max17048_get_report_soc(
			struct max17048_chip* chip, bool cable_in, int chg_status)
{
	u8 fg_soc = 0;
	if (SOC_SMOOTH_FALL == chip->soc_smooth.smooth_type)
	{
		max17048_soc_smooth_fall(chip);
		/* replace some lines */
	}
	else if (SOC_SMOOTH_RISE == chip->soc_smooth.smooth_type)
	{
		/* no charging, do not let soc rise */
		if (POWER_SUPPLY_STATUS_CHARGING != chg_status)
		{
			pr_vdebug("no charging, do not let soc rise!\n");
		}
		else
		{
			max17048_soc_smooth_rise(chip);
		}
	}
	else if (SOC_SMOOTH_SHUT == chip->soc_smooth.smooth_type)
	{
		chip->soc = chip->soc_smooth.end_soc;
	}
	else
	{
		/* get the fgague ic truly soc */
		fg_soc = max17048_get_soc(chip);
		if (POWER_SUPPLY_STATUS_CHARGING != chg_status)
		{
			pr_vdebug("no charging, report the min value of (%d, %d)!\n", chip->soc, fg_soc);
			chip->soc = min(chip->soc, fg_soc);
		}
		else
		{
			chip->soc = fg_soc;
		}
	}
}

static void max17048_report_soc(struct max17048_chip* chip)
{
	static u8 pre_soc = 0;
	bool cable_in = get_max77819_charger_present();
	int chg_status = max17048_get_charging_status(chip);
	
	/* check battery present */
	if (!max17048_get_battery_present(chip))
	{
		chip->soc = HW_BATT_FULL_SOC;
		chip->soc_smooth.smooth_type = SOC_SMOOTH_NONE;
		pr_vdebug("the battery removed, report soc 100 !! \n");
		goto report;
	}

	/* check the battery voltage to protect low soc */
	max17048_low_soc_smooth_info(chip, cable_in, chg_status);
	if (max17048_time_to_report(chip))
	{
		max17048_get_report_soc(chip, cable_in, chg_status);
		goto report;
	}
	else
	{
		goto smooth_skip;
	}

report:
	if (pre_soc != chip->soc)
	{
		pre_soc = chip->soc;
		pr_info("report new soc: %d \n", pre_soc);
		power_supply_changed(&(chip->fgbattery));
		max17048_relax_pm_by_soc(chip);
	}
smooth_skip:
	return;
}

static void max17048_dump_register(struct max17048_chip* chip)
{
	static u8	g_dump_reg_adrrs[] = {
		0x02,0x04,0x06,0x0a,0x0c,0x14,0x16,0x18,0x1a,
	};
	static u8	g_dump_lock_reg_addrs[] = {
		0x0e,
	};
	int i, num = 0;
	u16 value = 0;

	num = sizeof(g_dump_reg_adrrs) / sizeof(g_dump_reg_adrrs[0]);
	for (i = 0; i < num; i++)
	{
		max17048_read_reg(chip->client, g_dump_reg_adrrs[i], &value);
		pr_debug("reg[0x%02x] = 0x%04x \n", g_dump_reg_adrrs[i], value);
	}

	num = sizeof(g_dump_lock_reg_addrs) / sizeof(g_dump_lock_reg_addrs[0]);
	/* unlock */
	__unlock_register(chip)
	for (i = 0; i < num; i++)
	{
		max17048_read_reg(chip->client, g_dump_lock_reg_addrs[i], &value);
		pr_debug("reg[0x%02x] = 0x%04x \n", g_dump_lock_reg_addrs[i], value);
	}
	__lock_register(chip)
	/* lock */
}

static void max17048_debug_log(struct max17048_chip* chip)
{
	int temp = 0;

	if (log_level >= 2)
	{
		max17048_dump_register(chip);

		temp = max17048_get_battery_temp(chip);
		pr_debug("battery temp = %d \n", temp);
	}
}

static void max17048_load_model_by_por(struct max17048_chip* chip)
{
	if (max17048_check_por(chip))
	{
		__lock(chip);
		pr_info("the por mode, the chip have been reset, must to load model!!");
		max17048_handle_model(chip, MAX17048_VERIFY_FIX_MODEL);
		max17048_clear_por(chip);
		__unlock(chip);
	}
}

#define max17048_work_delay_ok(time)\
({\
    bool bdly_ok_##__LINE__ = true;\
    static int dly_##__LINE__ = 0;\
    int dlyNum_##__LINE__ = 0;\
\
    dlyNum_##__LINE__ = time / MAX17048_TIME_WORK_DELAY;\
    if (time % MAX17048_TIME_WORK_DELAY)\
    {\
        dlyNum_##__LINE__ += 1;\
    }\
\
    if (dly_##__LINE__ < dlyNum_##__LINE__)\
    {\
        bdly_ok_##__LINE__ = false;\
        dly_##__LINE__++;\
    }\
    else\
    {\
        bdly_ok_##__LINE__ = true;\
        dly_##__LINE__ = 0;\
    }\
\
    bdly_ok_##__LINE__;\
})

static void max17048_work(struct work_struct *work)
{
	struct max17048_chip *chip;
	chip = container_of(work, struct max17048_chip, work.work);

#ifdef CONFIG_HUAWEI_DSM
	if (max17048_work_delay_ok(MAX17048_DSM_WORK_TIME))
	{
		monitor_normal_soc_jump(chip);
	}
#endif

	if (max17048_work_delay_ok(MAX17048_RCOMP_WORK_TIME))
	{
		max17048_update_rcomp(chip);		//update rcomp periodically
	}

	if (max17048_work_delay_ok(MAX17048_SOC_REPORT_WORK_TIME))
	{
		max17048_report_soc(chip);
	}

	if (max17048_work_delay_ok(MAX17048_LOG_WORK_TIME))
	{
		max17048_debug_log(chip);
	}

	if (max17048_work_delay_ok(MAX17048_POR_WORK_TIME))
	{
		max17048_load_model_by_por(chip);
	}

	loop_schedule_delayed_work(&chip->work,
		msecs_to_jiffies(MAX17048_TIME_WORK_DELAY));
}

static void max17048_handle_work(struct work_struct *work)
{
	static bool chip_soc_init = false;
	struct max17048_chip *chip;
	chip = container_of(work, struct max17048_chip, hand_work.work);
	__lock(chip);
	max17048_handle_model(chip, MAX17048_VERIFY_FIX_MODEL);
	loop_schedule_delayed_work(&chip->hand_work, MAX17048_HANDLE_MODEL_DELAY);
	__unlock(chip);

	/* first to get the truly battery soc */
	if (!chip_soc_init)
	{
		chip->soc = max17048_get_soc(chip);
		chip_soc_init = true;
		log_info("first read soc = %d \n", chip->soc);
	}
}

static irqreturn_t max17048_low_bat_interrupt(int irq, void *thechip)
{
	struct max17048_chip* chip = (struct max17048_chip*)thechip;

	schedule_delayed_work(&chip->notifier_work, 0);

	return IRQ_HANDLED;
}

static void max17048_interrupt_notifier_work(struct work_struct *work)
{
	struct max17048_chip *chip;
	int ret = 0;
	u16 alert_val = 0;
	chip = container_of(work, struct max17048_chip, notifier_work.work);
	__lock(chip);

	/* check alert happend */
	ret = max17048_read_reg(chip->client, MAX17048_RCOMP_REG, &alert_val);
	if(unlikely(ret < 0) || unlikely(!(alert_val & MAX17048_ALERT_BITS)))
	{
		/* the alert bit is not 1, no alert happends */
		goto out_work;
	}

	/* clear the alert bit */
	alert_val = alert_val & (~MAX17048_ALERT_BITS);
	max17048_write_reg(chip->client, MAX17048_RCOMP_REG, alert_val);

	pr_info("battery is low: interrupt_notifier_work is invoked, soc = %d\n", chip->soc);
	if(chip->soc > chip->pdata->alert_soc)
	{
		pm_wakeup_event(&chip->client->dev,100);
	}
	else
	{
		pm_stay_awake(&chip->client->dev);
		chip->bbatt_alrm = true;
	}
	power_supply_changed(&chip->fgbattery);

out_work:
	/* clear all interrupts */
	max17048_write_reg(chip->client, MAX17048_STATUS_REG, 0);
	__unlock(chip);
	return;
}

static int max17048_soc_alarm_irq_init(struct max17048_chip* chip)
{
	int ret = 0;
	
	if (!gpio_is_valid(chip->pdata->irq_gpio)) {
		pr_err("irq gpio %d not provided\n", chip->pdata->irq_gpio);
		return -1;
	}

	/* configure max17048 irq gpio */
	ret = gpio_request(chip->pdata->irq_gpio,MAX17048_SOC_ALRM_IRQ_NAME);
	pr_debug("max17048 gpio_request\n");
	if (ret)
	{
		pr_err("unable to request gpio %d\n",	chip->pdata->irq_gpio);
		goto err_gpio;
	}
	else
	{
		gpio_direction_input(chip->pdata->irq_gpio);
		pr_debug("max17048 gpio_direction set to input\n");
    }
	chip->irq = chip->client->irq = gpio_to_irq(chip->pdata->irq_gpio);
	pr_debug("max17048 gpio = %d, aler irq = %d \n",chip->pdata->irq_gpio,chip->irq);

	/* request battery_low interruption */
	ret = request_irq(chip->irq, max17048_low_bat_interrupt, IRQF_TRIGGER_FALLING,
			MAX17048_SOC_ALRM_IRQ_NAME, chip);
	if (ret) {
		pr_err("could not request irq %d, status %d\n", chip->irq, ret);
		goto err_irq;
	}
	else
	{
		enable_irq_wake(chip->client->irq);
	}
	pr_debug("max17048 irq init ok,chip->irq = %d\n",chip->irq);
	return 0;

err_irq:
	free_irq(chip->irq, chip);
err_gpio:
	gpio_free(chip->pdata->irq_gpio);
	return -EIO;
}

static void max17048_soc_alarm_irq_free(struct max17048_chip* chip)
{
	free_irq(chip->irq, chip);
	gpio_free(chip->pdata->irq_gpio);
}

static int max17048_battery_id(struct max17048_chip *chip)
{
	int rc = 0;
	static int64_t vbatt_id = -1;
	struct qpnp_vadc_result results;

	if(-1 != vbatt_id)
	{
		return vbatt_id;
	}
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &results);
	if (rc) {
		pr_err("Unable to read battery_id rc=%d\n", rc);
		return -1;
	}
	vbatt_id = results.physical;
	return vbatt_id;
}

int maxim_get_battery_id_uv(void)
{
	if (global_chip)
	{
		return max17048_battery_id(global_chip);
	}
	return -1;
}
EXPORT_SYMBOL(maxim_get_battery_id_uv);

static int max17048_get_platform_data(struct max17048_chip* chip)
{
	int rc = 0;
	struct device_node *np = chip->client->dev.of_node;
	int vbatt_id = 0;

	/* get fgauge platform data info */
	//maxim,irq-soc-alarm-gpio;
	gplatform_data_default.irq_gpio = of_get_named_gpio(np, "maxim,irq-soc-alarm-gpio", 0);
	if (gplatform_data_default.irq_gpio <= 0){
		pr_err("batt-low-gpio is not available\n");
	}
	pr_debug("batt-low-gpio %d is assigned\n" ,gplatform_data_default.irq_gpio);

	OF_HW_READ_PROPERTY_VAL(np, "maxim,alert-soc", gplatform_data_default.alert_soc);
	OF_HW_READ_PROPERTY_VAL(np, "maxim,reset-mvlolt", gplatform_data_default.reset_mvlolt);
	OF_HW_READ_PROPERTY_VAL(np, "maxim,cutoff_batt_mv", gplatform_data_default.cutoff_batt_mv);
	OF_HW_READ_PROPERTY_VAL(np, "maxim,cutoff_batt_soc", gplatform_data_default.cutoff_batt_soc);
	OF_HW_READ_PROPERTY_VAL(np, "maxim,bad_batt_mv", gplatform_data_default.bad_batt_mv);
	
	/* get battery data info */
	vbatt_id = max17048_battery_id(chip);
	pr_debug("battery id uv %d \n", vbatt_id);
	rc = of_batterydata_read_fgauge_data_maxim(np, gplatform_data_default.pbat_data, vbatt_id);

	chip->pdata = &gplatform_data_default;

	return rc;
}

static int max17048_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);			
	struct max17048_chip *chip;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		pr_err("Failed: I2C bus function is not correct\n");		
		return -EIO;
	}
	ret = max17048_get_version(client);
	if (!MAX17048_RIGHT_VERSION(ret)) {
		pr_err("fail to get max17048 Fuel-Gauge Ver, exit!\n");
#ifdef CONFIG_HUAWEI_DSM
		/* if get max17048 version failed, record this log, and notify to the dsm server*/
		if(!dsm_client_ocuppy(bms_dclient)){
			dsm_client_record(bms_dclient,
				"get max17048 fuel gauge version failed: version = 0x%x\n", ret);
			dsm_client_notify(bms_dclient, DSM_MAXIM_GET_VER_FAIL);
		}
#endif
		return -EIO;
	}
	
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("Failed to allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->lock);

	chip->vadc_dev = qpnp_get_vadc(&(chip->client->dev), "maxim-bms");
	if (IS_ERR(chip->vadc_dev)) {
		ret = PTR_ERR(chip->vadc_dev);
		if (ret != -EPROBE_DEFER) {
			pr_err("vadc prop missing rc=%d\n", ret);
		}
		else{
			pr_err("no found dts adc node rc=%d\n", ret);
		}
		goto err_out1;
	}

	max17048_get_platform_data(chip);
	max17048_init(chip);

    /*low soc alert interupt support*/
	chip->bbatt_alrm = false;
	if (max17048_soc_alarm_irq_init(chip) < 0){
		ret = -EIO;
		goto err_out1; 
	}

	chip->fgbattery.name		= "max17048_fgauge";
	chip->fgbattery.type		= POWER_SUPPLY_TYPE_BMS;
	chip->fgbattery.get_property	= max17048_get_property;
	chip->fgbattery.properties	= max17048_battery_props;
	chip->fgbattery.num_properties	= ARRAY_SIZE(max17048_battery_props);

	ret = power_supply_register(&client->dev, &chip->fgbattery);
	if (ret) {
		pr_err("failed: power supply register\n");
		goto err_out2;
	}

#ifdef CONFIG_HUAWEI_DSM
	chip->saved_soc = -EINVAL;
#endif

	chip->soc = -EINVAL;
	chip->soc_smooth.end_soc = chip->soc;
	chip->soc_smooth.smooth_type = SOC_SMOOTH_NONE;

	INIT_DELAYED_WORK(&chip->notifier_work,max17048_interrupt_notifier_work);
	INIT_DELAYED_WORK(&chip->work, max17048_work);
	INIT_DELAYED_WORK(&chip->hand_work, max17048_handle_work);

	/* schedule handle work to load the model at once */
	schedule_delayed_work(&chip->hand_work,0);
	schedule_delayed_work(&chip->work, 0);

	global_chip = chip;
	return 0;

err_out2:
	max17048_soc_alarm_irq_free(chip);
err_out1:
	mutex_destroy(&chip->lock);
	kfree(chip);
	return ret;
}

static int max17048_remove(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->hand_work);
	cancel_delayed_work(&chip->work);
	cancel_delayed_work(&chip->notifier_work);
	
	power_supply_unregister(&chip->fgbattery);
	max17048_soc_alarm_irq_free(chip);
	
	mutex_destroy(&chip->lock);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17048_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	__lock(chip);

	cancel_delayed_work(&chip->hand_work);
	cancel_delayed_work(&chip->work);
	
	__unlock(chip);
	return 0;
}

static int max17048_resume(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	__lock(chip);

	schedule_delayed_work(&chip->hand_work,MAX17048_HANDLE_MODEL_DELAY);
	schedule_delayed_work(&chip->work, 0);

	__unlock(chip);
	return 0;
}

#else

#define max17048_suspend NULL
#define max17048_resume NULL

#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static struct of_device_id max17048_of_ids[] = {
	{ .compatible = "maxim,max17048" },
	{ },
};
MODULE_DEVICE_TABLE(of, max17048_of_ids);
#endif /* CONFIG_OF */

static const struct i2c_device_id max17048_id[] = {
	{ "max17048", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
	.driver	= {
		.name	= "max17048",
#ifdef CONFIG_OF
    		.of_match_table  = max17048_of_ids,
#endif /* CONFIG_OF */				
	},
	.probe		= max17048_probe,
	.remove		= max17048_remove,
	.suspend		= max17048_suspend,
	.resume		= max17048_resume,
	.id_table	= max17048_id,
};

module_i2c_driver(max17048_i2c_driver);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("max17048 Fuel Gauge");
MODULE_LICENSE("GPL");
