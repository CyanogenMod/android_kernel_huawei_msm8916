/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt)	"CHG: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/alarmtimer.h>
#include <linux/bitops.h>
#ifdef CONFIG_HUAWEI_KERNEL
#include <linux/of_batterydata.h>
#include<linux/wakelock.h>
#endif
#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#include <linux/time.h>
#endif
#ifdef CONFIG_HUAWEI_KERNEL
static struct qpnp_lbc_chip *global_chip;
#endif
#ifdef CONFIG_HUAWEI_KERNEL
#include <linux/power/bq24152_charger.h>
#endif
#ifdef CONFIG_HW_FEATURE_STORAGE_DIAGNOSE_LOG
#include <linux/store_log.h>
#endif
#include <linux/log_jank.h>
#define CREATE_MASK(NUM_BITS, POS) \
	((unsigned char) (((1 << (NUM_BITS)) - 1) << (POS)))
#define LBC_MASK(MSB_BIT, LSB_BIT) \
	CREATE_MASK(MSB_BIT - LSB_BIT + 1, LSB_BIT)

/* Interrupt offsets */
#define INT_RT_STS_REG				0x10
#define FAST_CHG_ON_IRQ                         BIT(5)
#define OVERTEMP_ON_IRQ				BIT(4)
#define BAT_TEMP_OK_IRQ                         BIT(1)
#define BATT_PRES_IRQ                           BIT(0)

/* USB CHARGER PATH peripheral register offsets */
#define USB_PTH_STS_REG				0x09
#define USB_IN_VALID_MASK			BIT(1)
#define USB_SUSP_REG				0x47
#define USB_SUSPEND_BIT				BIT(0)

/* CHARGER peripheral register offset */
#define CHG_OPTION_REG				0x08
#define CHG_OPTION_MASK				BIT(7)
#define CHG_STATUS_REG				0x09
#define CHG_VDD_LOOP_BIT			BIT(1)
#define CHG_VDD_MAX_REG				0x40
#define CHG_VDD_SAFE_REG			0x41
#define CHG_IBAT_MAX_REG			0x44
#define CHG_IBAT_SAFE_REG			0x45
#define CHG_VIN_MIN_REG				0x47
#define CHG_CTRL_REG				0x49
#define CHG_ENABLE				BIT(7)
#define CHG_FORCE_BATT_ON			BIT(0)
#define CHG_EN_MASK				(BIT(7) | BIT(0))
#define CHG_FAILED_REG				0x4A
#define CHG_FAILED_BIT				BIT(7)
#define CHG_VBAT_WEAK_REG			0x52
#define CHG_IBATTERM_EN_REG			0x5B
#define CHG_USB_ENUM_T_STOP_REG			0x4E
#define CHG_TCHG_MAX_EN_REG			0x60
#define CHG_TCHG_MAX_EN_BIT			BIT(7)
#define CHG_TCHG_MAX_MASK			LBC_MASK(6, 0)
#define CHG_TCHG_MAX_REG			0x61
#define CHG_WDOG_EN_REG				0x65
#define CHG_PERPH_RESET_CTRL3_REG		0xDA
#define CHG_COMP_OVR1				0xEE
#define CHG_VBAT_DET_OVR_MASK			LBC_MASK(1, 0)
#define OVERRIDE_0				0x2
#define OVERRIDE_NONE				0x0

/* BATTIF peripheral register offset */
#define BAT_IF_PRES_STATUS_REG			0x08
#define BATT_PRES_MASK				BIT(7)
#define BAT_IF_TEMP_STATUS_REG			0x09
#define BATT_TEMP_HOT_MASK			BIT(6)
#define BATT_TEMP_COLD_MASK			LBC_MASK(7, 6)
#define BATT_TEMP_OK_MASK			BIT(7)
#define BAT_IF_VREF_BAT_THM_CTRL_REG		0x4A
#define VREF_BATT_THERM_FORCE_ON		LBC_MASK(7, 6)
#define VREF_BAT_THM_ENABLED_FSM		BIT(7)
#define BAT_IF_BPD_CTRL_REG			0x48
#define BATT_BPD_CTRL_SEL_MASK			LBC_MASK(1, 0)
#define BATT_BPD_OFFMODE_EN			BIT(3)
#define BATT_THM_EN				BIT(1)
#define BATT_ID_EN				BIT(0)
#define BAT_IF_BTC_CTRL				0x49
#define BTC_COMP_EN_MASK			BIT(7)
#define BTC_COLD_MASK				BIT(1)
#define BTC_HOT_MASK				BIT(0)

/* MISC peripheral register offset */
#define MISC_REV2_REG				0x01
#define MISC_BOOT_DONE_REG			0x42
#define MISC_BOOT_DONE				BIT(7)
#define MISC_TRIM3_REG				0xF3
#define MISC_TRIM3_VDD_MASK			LBC_MASK(5, 4)
#define MISC_TRIM4_REG				0xF4
#define MISC_TRIM4_VDD_MASK			BIT(4)

#define PERP_SUBTYPE_REG			0x05
#define SEC_ACCESS                              0xD0

/* Linear peripheral subtype values */
#define LBC_CHGR_SUBTYPE			0x15
#define LBC_BAT_IF_SUBTYPE			0x16
#define LBC_USB_PTH_SUBTYPE			0x17
#define LBC_MISC_SUBTYPE			0x18

#define QPNP_CHG_I_MAX_MIN_90                   90

/* Feature flags */
#define VDD_TRIM_SUPPORTED			BIT(0)

#ifdef CONFIG_HUAWEI_KERNEL
/*Running test result*/
/*And charge abnormal info*/
#define CHARGE_STATUS_FAIL 	(0<<0) //Indicate running test charging status fail
#define CHARGE_STATUS_PASS 	(1<<0) //Indicate running test charging status pass
#define BATTERY_FULL 			(1<<1)
#define USB_NOT_PRESENT 		(1<<2)
#define REGULATOR_BOOST		(1<<3)
#define CHARGE_LIMIT			(1<<4)
#define BATTERY_HEALTH 		(1<<5)
#define CHARGER_OVP			(1<<6)
#define OCP_ABNORML			(1<<7)
#define BATTERY_VOL_ABNORML	(1<<8)
#define BATTERY_TEMP_ABNORML	(1<<9)
#define BATTERY_ABSENT			(1<<10)
#define BQ24152_FATAL_FAULT		(1<<11)

#define CHARGE_OCP_THR	-2500000 //charge current abnormal threshold
#define BATTERY_OCP_THR 	5000000 //discharge current abnormal threshold
#define BATTERY_VOL_THR_HI	4500000 //battery voltage abnormal high threshold
#define BATTERY_VOL_THR_LO	2500000 //battery voltage abnormal low threshold
#define BATTERY_TEMP_HI	780 //battery high temp threshold
#define BATTERY_TEMP_LO	-100 //battery low temp threshold
#define WARM_VOL_BUFFER	100 //cfg_warm_bat_mv need have a 100mV buffer
#define WARM_TEMP_THR		390 //battery warm temp threshold for running test
#define HOT_TEMP_THR		600 //battery hot temp threshold for running test
#define BATT_FULL			100 //battery full capactiy
#define USB_VALID_MASK		0xC0
#define USB_VALID_OVP_VALUE    0x40
#define ERROR_VBUS_OVP		1
#define ERROR_BATT_OVP		4
#define ERROR_THERMAL_SHUTDOWN		5
#define PASS_MASK			0x1E    //Running test pass mask

#define INT_RT_STS(base)			(base + 0x10)
#define LOG_BUF_LENGTH				128
#define NOT_CHARGE_COUNT		3
#define START_DISMATCH_COUNT      3
#define USBIN_VALID_IRQ			BIT(1)
#define CHG_VIN_MIN_LOOP_BIT			BIT(3)
#define WARM_TEMP_BUFFER		30
#define COLD_HOT_TEMP_BUFFER		30
#define BQ2415X_REG_STATUS		0x00
#define BQ2415X_REG_CURRENT		0x04
#endif

#ifdef CONFIG_HUAWEI_DSM
#define DSM_COUNT		3
#define CHECKING_TIME	15000
#define DELAY_TIME		5000
#define VOL_THR1		3600000
#define VOL_THR2		3700000
#define VOL_HIGH		4320000
#define VOL_TOO_LOW	3200000
#define VOL_REGULATION_MAX	4370000
#define VOL_REGULATION_MAX_MAXIM		4450000
#define SOC_THR1		2
#define SOC_ZERO		0
#define SOC_HIGH		90
#define SOC_HIGH_THR	95
#define OVER_CURRENT	5000000
#define HIGH_VOL	4400000
#define LOW_VOL		2500000
#define HOT_TEMP	600
#define LOW_TEMP	0
#define TEMP_UPPER_THR	400
#define TEMP_LOWER_THR	200
#define TEMP_BUFFER	20
#define CHARGE_CURRENT_MAX	-1500000
#define TEMP_DELTA		30
#define INIT_TEMP		-2730
#define HALF_MINUTE		30
#define MAX_COUNT		20
#define SOC_JUMP_USBIN		2
#define WARM_COOL_CURRENT_LIMIT		1000000
#define INIT_TIME					70
#define QUARTER_MINUTE				15
#define STATUS_NOT_MATCH_COUNT		1
#endif
#ifdef CONFIG_HUAWEI_KERNEL
#define LED_CHECK_PERIOD_MS		3000
#endif

#define QPNP_CHARGER_DEV_NAME	"qcom,qpnp-linear-charger"

/* usb_interrupts */

struct qpnp_lbc_irq {
	int		irq;
	unsigned long	disabled;
	bool            is_wake;
};

enum {
	USBIN_VALID = 0,
	USB_OVER_TEMP,
	USB_CHG_GONE,
	BATT_PRES,
	BATT_TEMPOK,
	CHG_DONE,
	CHG_FAILED,
	CHG_FAST_CHG,
	CHG_VBAT_DET_LO,
	MAX_IRQS,
};

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC	= BIT(3),
};

enum bpd_type {
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_ID] = "bpd_id",
	[BPD_TYPE_BAT_THM] = "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID] = "bpd_thm_id",
};

enum btc_type {
	HOT_THD_25_PCT = 25,
	HOT_THD_35_PCT = 35,
	COLD_THD_70_PCT = 70,
	COLD_THD_80_PCT = 80,
};

static u8 btc_value[] = {
	[HOT_THD_25_PCT] = 0x0,
	[HOT_THD_35_PCT] = BIT(0),
	[COLD_THD_70_PCT] = 0x0,
	[COLD_THD_80_PCT] = BIT(1),
};

static inline int get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
#ifdef CONFIG_HUAWEI_KERNEL
	POWER_SUPPLY_PROP_FACTORY_DIAG,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_HOT_IBAT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_LOG,
	POWER_SUPPLY_PROP_TECHNOLOGY,
#endif
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_COOL_TEMP,
	POWER_SUPPLY_PROP_WARM_TEMP,
#ifdef CONFIG_HUAWEI_KERNEL
	POWER_SUPPLY_PROP_RUNNING_TEST_STATUS,
#endif
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
#ifdef CONFIG_HUAWEI_KERNEL
	POWER_SUPPLY_PROP_RESUME_CHARGING,
#endif
};

static char *pm_batt_supplied_to[] = {
	"bms",
#ifdef CONFIG_HUAWEI_KERNEL
	"ti-charger"
#endif
};

struct vddtrim_map {
	int			trim_uv;
	int			trim_val;
};

/*
 * VDDTRIM is a 3 bit value which is split across two
 * register TRIM3(bit 5:4)	-> VDDTRIM bit(2:1)
 * register TRIM4(bit 4)	-> VDDTRIM bit(0)
 */
#define TRIM_CENTER			4
#define MAX_VDD_EA_TRIM_CFG		8
#define VDD_TRIM3_MASK			LBC_MASK(2, 1)
#define VDD_TRIM3_SHIFT			3
#define VDD_TRIM4_MASK			BIT(0)
#define VDD_TRIM4_SHIFT			4
#define AVG(VAL1, VAL2)			((VAL1 + VAL2) / 2)

/*
 * VDDTRIM table containing map of trim voltage and
 * corresponding trim value.
 */
struct vddtrim_map vddtrim_map[] = {
	{36700,		0x00},
	{28000,		0x01},
	{19800,		0x02},
	{10760,		0x03},
	{0,		0x04},
	{-8500,		0x05},
	{-16800,	0x06},
	{-25440,	0x07},
};

/*
 * struct qpnp_lbc_chip - device information
 * @dev:			device pointer to access the parent
 * @spmi:			spmi pointer to access spmi information
 * @chgr_base:			charger peripheral base address
 * @bat_if_base:		battery interface  peripheral base address
 * @usb_chgpth_base:		USB charge path peripheral base address
 * @misc_base:			misc peripheral base address
 * @bat_is_cool:		indicates that battery is cool
 * @bat_is_warm:		indicates that battery is warm
 * @chg_done:			indicates that charging is completed
 * @usb_present:		present status of USB
 * @batt_present:		present status of battery
 * @cfg_charging_disabled:	disable drawing current from USB.
 * @cfg_use_fake_battery:	flag to report default battery properties
 * @fastchg_on:			indicate charger in fast charge mode
 * @cfg_btc_disabled:		flag to disable btc (disables hot and cold
 *				irqs)
 * @cfg_max_voltage_mv:		the max volts the batt should be charged up to
 * @cfg_min_voltage_mv:		VIN_MIN configuration
 * @cfg_batt_weak_voltage_uv:	weak battery voltage threshold
 * @cfg_warm_bat_chg_ma:	warm battery maximum charge current in mA
 * @cfg_cool_bat_chg_ma:	cool battery maximum charge current in mA
 * @cfg_safe_voltage_mv:	safe voltage to which battery can charge
 * @cfg_warm_bat_mv:		warm temperature battery target voltage
 * @cfg_warm_bat_mv:		warm temperature battery target voltage
 * @cfg_cool_bat_mv:		cool temperature battery target voltage
 * @cfg_soc_resume_charging:	indicator that battery resumes charging
 * @cfg_float_charge:		enable float charging
 * @charger_disabled:		maintain USB path state.
 * @cfg_charger_detect_eoc:	charger can detect end of charging
 * @cfg_disable_vbatdet_based_recharge:	keep VBATDET comparator overriden to
 *				low and VBATDET irq disabled.
 * @cfg_safe_current:		battery safety current setting
 * @cfg_hot_batt_p:		hot battery threshold setting
 * @cfg_cold_batt_p:		eold battery threshold setting
 * @cfg_warm_bat_decidegc:	warm battery temperature in degree Celsius
 * @cfg_cool_bat_decidegc:	cool battery temperature in degree Celsius
 * @fake_battery_soc:		SOC value to be reported to userspace
 * @cfg_tchg_mins:		maximum allowed software initiated charge time
 * @chg_failed_count:		counter to maintained number of times charging
 *				failed
 * @cfg_disable_follow_on_reset	charger ignore PMIC reset signal
 * @cfg_bpd_detection:		battery present detection mechanism selection
 * @cfg_thermal_levels:		amount of thermal mitigation levels
 * @cfg_thermal_mitigation:	thermal mitigation level values
 * @therm_lvl_sel:		thermal mitigation level selection
 * @jeita_configure_lock:	lock to serialize jeita configuration request
 * @hw_access_lock:		lock to serialize access to charger registers
 * @ibat_change_lock:		lock to serialize ibat change requests from
 *				USB and thermal.
 * @irq_lock			lock to serialize enabling/disabling of irq
 * @supported_feature_flag	bitmask for all supported features
 * @vddtrim_alarm		alarm to schedule trim work at regular
 *				interval
 * @vddtrim_work		work to perform actual vddmax trimming
 * @init_trim_uv		initial trim voltage at bootup
 * @delta_vddmax_uv		current vddmax trim voltage
 * @chg_enable_lock:		lock to serialize charging enable/disable for
 *				SOC based resume charging
 * @usb_psy:			power supply to export information to
 *				userspace
 * @bms_psy:			power supply to export information to
 *				userspace
 * @batt_psy:			power supply to export information to
 *				userspace
 */
struct qpnp_lbc_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				misc_base;
	bool				bat_is_cool;
	bool				bat_is_warm;
	bool				chg_done;
	bool				usb_present;
	bool				batt_present;
	bool				cfg_charging_disabled;
	bool				cfg_btc_disabled;
	bool				cfg_use_fake_battery;
	bool				fastchg_on;
	bool				cfg_use_external_charger;
#ifdef CONFIG_HUAWEI_KERNEL
	bool				use_other_charger;
    bool                use_only_ti_charger;
	bool				resuming_charging;
	char				log_buf[LOG_BUF_LENGTH];
#endif
	unsigned int			cfg_warm_bat_chg_ma;
	unsigned int			cfg_cool_bat_chg_ma;
	unsigned int			cfg_safe_voltage_mv;
	unsigned int			cfg_max_voltage_mv;
	unsigned int			cfg_min_voltage_mv;
	unsigned int			cfg_charger_detect_eoc;
	unsigned int			cfg_disable_vbatdet_based_recharge;
	unsigned int			cfg_batt_weak_voltage_uv;
	unsigned int			cfg_warm_bat_mv;
	unsigned int			cfg_cool_bat_mv;
	unsigned int			cfg_hot_batt_p;
	unsigned int			cfg_cold_batt_p;
	unsigned int			cfg_thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;
	unsigned int			cfg_safe_current;
	unsigned int			cfg_tchg_mins;
	unsigned int			chg_failed_count;
	unsigned int			cfg_disable_follow_on_reset;
	unsigned int			supported_feature_flag;
	int				cfg_bpd_detection;
	int				cfg_warm_bat_decidegc;
	int				cfg_cool_bat_decidegc;
	int				fake_battery_soc;
#ifdef CONFIG_HUAWEI_KERNEL
	bool				cfg_soc_resume_charging;
#endif
	int				cfg_float_charge;
	int				charger_disabled;
	int				prev_max_ma;
#ifdef CONFIG_HUAWEI_KERNEL
	int				running_test_settled_status;
#endif
	int				usb_psy_ma;
	int				delta_vddmax_uv;
	int				init_trim_uv;
	struct alarm			vddtrim_alarm;
	struct work_struct		vddtrim_work;
	struct qpnp_lbc_irq		irqs[MAX_IRQS];
	struct mutex			jeita_configure_lock;
	struct mutex			chg_enable_lock;
#ifdef CONFIG_HUAWEI_DSM
	struct mutex			dsm_dump_lock;
	struct mutex			dsm_soc_lock;
#endif
	spinlock_t			ibat_change_lock;
	spinlock_t			hw_access_lock;
	spinlock_t			irq_lock;
#ifdef CONFIG_HUAWEI_KERNEL
	spinlock_t			chg_en_lock;
#endif
	struct power_supply		*usb_psy;
	struct power_supply		*bms_psy;
#ifdef CONFIG_HUAWEI_KERNEL
	struct power_supply		*ti_charger;
	struct power_supply		*maxim_charger;
#endif
	struct power_supply		batt_psy;
	struct qpnp_adc_tm_btm_param	adc_param;
	struct qpnp_vadc_chip		*vadc_dev;
	struct qpnp_adc_tm_chip		*adc_tm_dev;
#ifdef CONFIG_HUAWEI_DSM
	struct delayed_work		check_charging_batt_status_work;
	struct work_struct		dump_work;
	struct work_struct		usbin_valid_count_work;
	int				error_type;
#endif
#ifdef CONFIG_HUAWEI_KERNEL
	int				cfg_term_current;
	int				cfg_cold_bat_decidegc;
	int				cfg_hot_bat_decidegc;
#endif
#ifdef CONFIG_HUAWEI_KERNEL
	struct wake_lock		led_wake_lock;
	struct wake_lock		chg_wake_lock;
#endif
	  /* deleted 1 line */
};

#ifdef CONFIG_HUAWEI_KERNEL
enum hw_high_low_temp_configure_type {
	COLD_COLD_ZONE,
	COLD_COOL_ZONE,
	COOL_WARM_ZONE,
	WARM_HOT_ZONE,
	HOT_HOT_ZONE,
	UNKNOW_ZONE,
};
#define HOT_TEMP_DEFAULT		520
#define COLD_TEMP_DEFAULT		0
#define BATT_FULL_LEVEL			100
#define BATT_LEVEL_99			99
static int bad_temp_flag = false;
struct qpnp_lbc_chip *g_lbc_chip = NULL;
#endif

#ifdef CONFIG_HUAWEI_DSM
/*charger dsm client definition */
struct dsm_dev dsm_charger = {
	.name = "dsm_charger", // dsm client name
	.fops = NULL,
	.buff_size = 4096, // buffer size
};
struct dsm_client *charger_dclient = NULL;
extern u16 bms_base;
extern struct dsm_client *bms_dclient;
/*8916 pmic LBC_CHGR registers*/
static u8 LBC_CHGR[] = {
	0x08,
	0x09,
	0x0A,
	0x0B,
	0x10,
	0x11,
	0x12,
	0x13,
	0x14,
	0x15,
	0x16,
	0x18,
	0x19,
	0x1A,
	0x1B,
	0x40,
	0x41,
	0x43,
	0x44,
	0x45,
	0x47,
	0x49,
	0x4A,
	0x4C,
	0x4D,
	0x50,
	0x52,
	0x55,
	0x5B,
	0x5E,
	0x5F,
	0x60,
	0x61,
	0x62,
	0x63,
	0x64,
	0x65,
	0x66,
	0x69,
	0x6A
};
/*8916 pmic LBC_BAT_IF registers*/
static u8 LBC_BAT_IF[] = {
	0x08,
	0x09,
	0x0A,
	0x10,
	0x11,
	0x12,
	0x13,
	0x14,
	0x15,
	0x16,
	0x18,
	0x19,
	0x1A,
	0x1B,
	0x48,
	0x49,
	0x4A,
	0x4F,
	0xD0
};
/*8916 pmic LBC_USB registers*/
static u8 LBC_USB[] = {
	0x08,
	0x09,
	0x10,
	0x11,
	0x12,
	0x13,
	0x14,
	0x15,
	0x16,
	0x18,
	0x19,
	0x1A,
	0x1B,
	0x42,
	0x47,
	0x4E,
	0x4F,
	0xD0
};
/*8916 pmic LBC_MISC registers*/
static u8 LBC_MISC[] = {
	0x40,
	0x41,
	0x42,
	0x43,
	0x49,
	0xCD,
	0xCE,
	0xD0
};

/*8916 pmic VM_BMS registers*/
static u8 VM_BMS[] = {
	0x08,
	0x09,
	0x10,
	0x11,
	0x12,
	0x13,
	0x14,
	0x15,
	0x16,
	0x18,
	0x19,
	0x1A,
	0x1B,
	0x40,
	0x42,
	0x43,
	0x44,
	0x46,
	0x47,
	0x50,
	0x51,
	0x53,
	0x55,
	0x56,
	0x57,
	0x58,
	0x5A,
	0x5B,
	0x5C,
	0x5D,
	0x5E,
	0x5F,
	0x60,
	0x61,
	0x62,
	0x63,
	0x64,
	0x65,
	0x66,
	0x67,
	0x6A,
	0x6B,
	0xB0,
	0xB1,
	0xB2,
	0xB3,
	0xB4,
	0xB5,
	0xB6,
	0xB7,
	0xB8,
	0xB9,
	0xC0,
	0xC1,
	0xC2,
	0xC3,
	0xC4,
	0xC5,
	0xC6,
	0xC7,
	0xC8,
	0xC9,
	0xCA,
	0xCB,
	0xCC,
	0xCD,
	0xCE,
	0xCF,
	0xD0,
	0xD8,
	0xD9,
	0xDA,
	0xDB
};

int dump_registers_and_adc(struct dsm_client *dclient, struct qpnp_lbc_chip *chip, int type );
/* remove struct qpnp_lbc_chip *g_lbc_chip = NULL;*/
bool use_other_charger = false;
static int usbin_irq_invoke_flag = 0;
static unsigned long usbin_start_tm_sec = 0;

extern void bq2415x_dump_regs(struct dsm_client *dclient);
extern void bq27510_dump_regs(struct dsm_client *dclient);

extern void max77819_charger_dump_regs(struct dsm_client *dclient);
extern void max17048_fgauge_dump_regs(struct dsm_client *dclient);
extern bool is_sw_factory_mode(void);
/* remove is_bq24152_in_boost_mode & get_bq2415x_fault_status*/
/* move some lines */

#endif

#ifdef CONFIG_HUAWEI_KERNEL
static int get_prop_batt_temp(struct qpnp_lbc_chip *chip);
extern int get_bq2415x_reg_values(u8 reg);
extern bool get_max77819_charger_timeout(void);
extern int get_max77819_boost_mode(void);
extern void do_max77819_dcin_valid_irq(int present);
extern const struct max77819_temp_control_info* max77819_charger_get_temp_ctrl_info(void);
extern int is_bq24152_in_boost_mode(void);
extern int get_bq2415x_fault_status(void);
#endif
#ifdef CONFIG_HUAWEI_KERNEL
#define HOT_DESIGN_MAX_CURRENT 1440
int hot_design_current = HOT_DESIGN_MAX_CURRENT;
static int factory_diag_flag = 0;
static int factory_diag_last_current_ma = 0;
static int input_current_max_ma = 0;
static int input_current_max_flag = 0;
#endif
#ifdef CONFIG_HUAWEI_KERNEL
static int fake_capacity = -1;
module_param(fake_capacity, int, 0644);
static int charge_no_limit = 0;

enum hw_charge_configure_type{
	NORMAL_TYPE,
	DISABLE_SOFTWARE_HARDWARE_TEMP_CONTROL,
	DISABLE_SOFTWARE_TEMP_CONTROL,
	DISABLE_HARDWARE_TEMP_CONTROL,
	MAX_TYPE,
};

static int qpnp_lbc_bat_if_configure_btc(struct qpnp_lbc_chip *chip);
static int qpnp_lbc_is_batt_present(struct qpnp_lbc_chip *chip);

#ifdef CONFIG_HUAWEI_HLTHERM_CHARGING
#define BAT_IF_THEMP_LOCK_STATUS_REG         0xD0
#define BAT_IF_HARD_THEMP_CTL_REG            0xE5
static int qpnp_lbc_write(struct qpnp_lbc_chip *chip, u16 base,
                u8 *val, int count);

static void hw_high_low_temp_charging_flag(bool flag)
{
    int rc = 0;
    u8 temp = 0;

    temp = 0xA5;
    /*the register write 0xA5, open the lock*/
    rc = qpnp_lbc_write(global_chip,
            global_chip->bat_if_base + BAT_IF_THEMP_LOCK_STATUS_REG,
            &temp, 1);
    if (rc) {
        pr_err("close THEMP_LOCK_STATUS error rc=%d\n", rc);
    }

    if(flag)
    {
        temp = 0x28;
        /*the register write 0x28, Close temperature charging limit.Write 0x0,open the limit*/
        rc = qpnp_lbc_write(global_chip,
                    global_chip->bat_if_base + BAT_IF_HARD_THEMP_CTL_REG,
                    &temp, 1);
        if (rc) {
            pr_err("open HARD_THEMP_CTL error rc=%d\n", rc);
        }
        temp = 0x90;
        rc = qpnp_lbc_write(global_chip, global_chip->chgr_base + CHG_CTRL_REG, &temp, 1);
        if (rc) {
            pr_err("set global_chip->chgr_base + 0x49 error rc=%d\n", rc);
        }
        pr_debug("Open the high and low temperature charging\n");
    }
    else
    {
        temp = 0x0;
        rc = qpnp_lbc_write(global_chip,
                global_chip->bat_if_base + BAT_IF_HARD_THEMP_CTL_REG,
                &temp, 1);
        if (rc) {
            pr_err("close HARD_THEMP_CTL error rc=%d\n", rc);
        }
        pr_debug("Close the high and low temperature charging\n");
    }
}
#endif

#ifdef CONFIG_HUAWEI_HLTHERM_CHARGING
int get_high_low_temp_flag(void)
{
    return charge_no_limit;
}
#endif
/*==========================================
FUNCTION: hw_set_charge_temp_type

DESCRIPTION:	1. set battery charge temp limit type

INPUT:	hw_charge_configure_type
OUTPUT: NULL
RETURN: NULL

============================================*/
static void hw_set_charge_temp_type(enum hw_charge_configure_type type)
{
	int batt_present,rc;
	if(!global_chip)
	{
		return ;
	}
	
	batt_present = qpnp_lbc_is_batt_present(global_chip);
	switch(type){
	case NORMAL_TYPE:
#ifdef CONFIG_HUAWEI_HLTHERM_CHARGING
		hw_high_low_temp_charging_flag((bool)type);
#endif
		if(batt_present)
		{
			rc = qpnp_adc_tm_channel_measure(global_chip->adc_tm_dev,&global_chip->adc_param);
			if (rc) {
				pr_err("request ADC error rc=%d\n", rc);
			}
		}		
		global_chip->cfg_hot_batt_p = HOT_THD_35_PCT;
		global_chip->cfg_cold_batt_p = COLD_THD_70_PCT;
		rc = qpnp_lbc_bat_if_configure_btc(global_chip);
		if(rc)
		{
			pr_err("Failed to configure btc rc=%d\n", rc);
		}
		break;
	case DISABLE_SOFTWARE_HARDWARE_TEMP_CONTROL:
#ifdef CONFIG_HUAWEI_HLTHERM_CHARGING
		hw_high_low_temp_charging_flag((bool)type);
#endif
		qpnp_adc_tm_disable_chan_meas(global_chip->adc_tm_dev,&global_chip->adc_param);
		global_chip->cfg_hot_batt_p = HOT_THD_25_PCT;
		global_chip->cfg_cold_batt_p = COLD_THD_80_PCT;
		rc = qpnp_lbc_bat_if_configure_btc(global_chip);
		if(rc)
		{
			pr_err("Failed to configure btc rc=%d\n", rc);
		}
		else
		{
			pr_info("hardware battery threshold is changed to 25 to 80,and software temp control is canceled \n");
		}
		break;
	case DISABLE_SOFTWARE_TEMP_CONTROL:
		qpnp_adc_tm_disable_chan_meas(global_chip->adc_tm_dev,&global_chip->adc_param);		
		pr_info("software battery temperature control is canceled \n");
		break;
	case DISABLE_HARDWARE_TEMP_CONTROL:
		global_chip->cfg_hot_batt_p = HOT_THD_25_PCT;
		global_chip->cfg_cold_batt_p = COLD_THD_80_PCT;
		rc = qpnp_lbc_bat_if_configure_btc(global_chip);
		if(rc)
		{
			pr_err("Failed to configure btc rc=%d\n", rc);
		}
		else
		{
			pr_info("hardware battery temperature threshold is changed to 25 to 80 \n");
		}
		break;
	default:
		break;
	}

	return ;
}

static int set_charge_limit_temp_type(const char *val, struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	if(!global_chip)
	{
		return ret;
	}
	else
	{
		hw_set_charge_temp_type((enum hw_charge_configure_type)charge_no_limit);
		return ret;
	}
}

module_param_call(charge_no_limit,&set_charge_limit_temp_type,&param_get_int,&charge_no_limit,0664);

#endif

static void qpnp_lbc_enable_irq(struct qpnp_lbc_chip *chip,
					struct qpnp_lbc_irq *irq)
{
	unsigned long flags;

	spin_lock_irqsave(&chip->irq_lock, flags);
	if (__test_and_clear_bit(0, &irq->disabled)) {
		pr_debug("number = %d\n", irq->irq);
		enable_irq(irq->irq);
		if (irq->is_wake)
			enable_irq_wake(irq->irq);
	}
	spin_unlock_irqrestore(&chip->irq_lock, flags);
}

static void qpnp_lbc_disable_irq(struct qpnp_lbc_chip *chip,
					struct qpnp_lbc_irq *irq)
{
	unsigned long flags;

	spin_lock_irqsave(&chip->irq_lock, flags);
	if (!__test_and_set_bit(0, &irq->disabled)) {
		pr_debug("number = %d\n", irq->irq);
		disable_irq_nosync(irq->irq);
		if (irq->is_wake)
			disable_irq_wake(irq->irq);
	}
	spin_unlock_irqrestore(&chip->irq_lock, flags);
}

static int __qpnp_lbc_read(struct spmi_device *spmi, u16 base,
			u8 *val, int count)
{
	int rc = 0;

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);
#ifdef CONFIG_HUAWEI_DSM
	/* if spmi read fail, record this log, and notify to the dsm server*/
	if(rc){
		if(!dsm_client_ocuppy(charger_dclient)){
			dsm_client_record(charger_dclient, "[%s]spmi read failed, rc=%d\n",__func__, rc);
			dsm_client_notify(charger_dclient, DSM_SPMI_ABNORMAL_ERROR_NO);
		}
	}
#endif
	return rc;
}

static int __qpnp_lbc_write(struct spmi_device *spmi, u16 base,
			u8 *val, int count)
{
	int rc;

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val,
					count);
	if (rc)
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);

	return rc;
}

static int __qpnp_lbc_secure_write(struct spmi_device *spmi, u16 base,
				u16 offset, u8 *val, int count)
{
	int rc;
	u8 reg_val;

	reg_val = 0xA5;
	rc = __qpnp_lbc_write(spmi, base + SEC_ACCESS, &reg_val, 1);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);
		return rc;
	}

	rc = __qpnp_lbc_write(spmi, base + offset, val, 1);
	if (rc)
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);

	return rc;
}

static int qpnp_lbc_read(struct qpnp_lbc_chip *chip, u16 base,
			u8 *val, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);
		return -EINVAL;
	}

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	rc = __qpnp_lbc_read(spmi, base, val, count);
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);

	return rc;
}

static int qpnp_lbc_write(struct qpnp_lbc_chip *chip, u16 base,
			u8 *val, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);
		return -EINVAL;
	}

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	rc = __qpnp_lbc_write(spmi, base, val, count);
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);

	return rc;
}

static int qpnp_lbc_masked_write(struct qpnp_lbc_chip *chip, u16 base,
				u8 mask, u8 val)
{
	int rc;
	u8 reg_val;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	rc = __qpnp_lbc_read(spmi, base, &reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", base, rc);
		goto out;
	}
	pr_debug("addr = 0x%x read 0x%x\n", base, reg_val);

	reg_val &= ~mask;
	reg_val |= val & mask;

	pr_debug("writing to base=%x val=%x\n", base, reg_val);

	rc = __qpnp_lbc_write(spmi, base, &reg_val, 1);
	if (rc)
		pr_err("spmi write failed: addr=%03X, rc=%d\n", base, rc);

out:
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);
	return rc;
}

static int __qpnp_lbc_secure_masked_write(struct spmi_device *spmi, u16 base,
				u16 offset, u8 mask, u8 val)
{
	int rc;
	u8 reg_val, reg_val1;

	rc = __qpnp_lbc_read(spmi, base + offset, &reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", base, rc);
		return rc;
	}
	pr_debug("addr = 0x%x read 0x%x\n", base, reg_val);

	reg_val &= ~mask;
	reg_val |= val & mask;
	pr_debug("writing to base=%x val=%x\n", base, reg_val);

	reg_val1 = 0xA5;
	rc = __qpnp_lbc_write(spmi, base + SEC_ACCESS, &reg_val1, 1);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);
		return rc;
	}

	rc = __qpnp_lbc_write(spmi, base + offset, &reg_val, 1);
	if (rc) {
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base + offset, spmi->sid, rc);
		return rc;
	}

	return rc;
}

static int qpnp_lbc_get_trim_voltage(u8 trim_reg)
{
	int i;

	for (i = 0; i < MAX_VDD_EA_TRIM_CFG; i++)
		if (trim_reg == vddtrim_map[i].trim_val)
			return vddtrim_map[i].trim_uv;

	pr_err("Invalid trim reg reg_val=%x\n", trim_reg);
	return -EINVAL;
}

static u8 qpnp_lbc_get_trim_val(struct qpnp_lbc_chip *chip)
{
	int i, sign;
	int delta_uv;

	sign = (chip->delta_vddmax_uv >= 0) ? -1 : 1;

	switch (sign) {
	case -1:
		for (i = TRIM_CENTER; i >= 0; i--) {
			if (vddtrim_map[i].trim_uv > chip->delta_vddmax_uv) {
				delta_uv = AVG(vddtrim_map[i].trim_uv,
						vddtrim_map[i + 1].trim_uv);
				if (chip->delta_vddmax_uv >= delta_uv)
					return vddtrim_map[i].trim_val;
				else
					return vddtrim_map[i + 1].trim_val;
			}
		}
		break;
	case 1:
		for (i = TRIM_CENTER; i <= 7; i++) {
			if (vddtrim_map[i].trim_uv < chip->delta_vddmax_uv) {
				delta_uv = AVG(vddtrim_map[i].trim_uv,
						vddtrim_map[i - 1].trim_uv);
				if (chip->delta_vddmax_uv >= delta_uv)
					return vddtrim_map[i - 1].trim_val;
				else
					return vddtrim_map[i].trim_val;
			}
		}
		break;
	}

	return vddtrim_map[i].trim_val;
}

static int qpnp_lbc_is_usb_chg_plugged_in(struct qpnp_lbc_chip *chip)
{
#ifdef CONFIG_HUAWEI_KERNEL
	u8 usb_chgpth_rt_sts = 0;
#else
	u8 usbin_valid_rt_sts = 0;
#endif
	int rc;

#ifdef CONFIG_HUAWEI_KERNEL
	rc = qpnp_lbc_read(chip, INT_RT_STS(chip->usb_chgpth_base),
				&usb_chgpth_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->usb_chgpth_base), rc);
		return rc;
	}

	pr_debug("usb_chgpth_rt_sts 0x%x\n", usb_chgpth_rt_sts);

	return (usb_chgpth_rt_sts & USBIN_VALID_IRQ) ? 1 : 0;
#else
	rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + INT_RT_STS_REG,
				&usbin_valid_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->usb_chgpth_base + INT_RT_STS_REG, rc);
		return rc;
	}

	pr_debug("rt_sts 0x%x\n", usbin_valid_rt_sts);

	return (usbin_valid_rt_sts & USB_IN_VALID_MASK) ? 1 : 0;
#endif
}

static int qpnp_lbc_charger_enable(struct qpnp_lbc_chip *chip, int reason,
					int enable)
{
#ifdef CONFIG_HUAWEI_KERNEL
	int disabled;
	unsigned long flags;
#else
	int disabled = chip->charger_disabled;
#endif
	u8 reg_val;
	int rc = 0;

#ifdef CONFIG_HUAWEI_KERNEL
	if (chip->use_other_charger) {
		/* always disble pmic charger if use external charger */
		reg_val = CHG_FORCE_BATT_ON;
		rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
							CHG_EN_MASK, reg_val);
		if (rc) {
			pr_err("Failed to disable pmic charger rc=%d\n", rc);
			return rc;
		}
		return rc;
	}
	spin_lock_irqsave(&chip->chg_en_lock, flags);
	disabled = chip->charger_disabled;
#endif

	pr_debug("reason=%d requested_enable=%d disabled_status=%d\n",
					reason, enable, disabled);
	if (enable)
		disabled &= ~reason;
	else
		disabled |= reason;
	/* avoid goto skip when enable charger but chip->charger_diabled is 0 */
#ifdef CONFIG_HUAWEI_KERNEL
	if ((!!chip->charger_disabled == !!disabled) && chip->charger_disabled)
#else
	if (!!chip->charger_disabled == !!disabled)
#endif
		goto skip;

#ifdef CONFIG_HUAWEI_KERNEL
	if ((bad_temp_flag) && enable)
	{
		pr_info("bad_temp_flag %d, not enable charge\n", bad_temp_flag);
		goto skip;
	}
#endif

	reg_val = !!disabled ? CHG_FORCE_BATT_ON : CHG_ENABLE;
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
				CHG_EN_MASK, reg_val);
	if (rc) {
		pr_err("Failed to %s charger rc=%d\n",
				reg_val ? "enable" : "disable", rc);
#ifdef CONFIG_HUAWEI_KERNEL
		spin_unlock_irqrestore(&chip->chg_en_lock, flags);
#endif
		return rc;
	}
skip:
	chip->charger_disabled = disabled;
#ifdef CONFIG_HUAWEI_KERNEL
	spin_unlock_irqrestore(&chip->chg_en_lock, flags);
#endif
	return rc;
}

#ifdef CONFIG_HUAWEI_KERNEL
int is_usb_chg_exist(void)
{
	if (!global_chip) 
	{
		pr_err("called before init\n");
		return -EINVAL;
	}
	return qpnp_lbc_is_usb_chg_plugged_in(global_chip);
}
EXPORT_SYMBOL(is_usb_chg_exist);
#endif

static int qpnp_lbc_is_batt_present(struct qpnp_lbc_chip *chip)
{
	u8 batt_pres_rt_sts;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + INT_RT_STS_REG,
				&batt_pres_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->bat_if_base + INT_RT_STS_REG, rc);
		return rc;
	}

	return (batt_pres_rt_sts & BATT_PRES_IRQ) ? 1 : 0;
}

static int qpnp_lbc_bat_if_configure_btc(struct qpnp_lbc_chip *chip)
{
	u8 btc_cfg = 0, mask = 0, rc;

	/* Do nothing if battery peripheral not present */
	if (!chip->bat_if_base)
		return 0;

	if ((chip->cfg_hot_batt_p == HOT_THD_25_PCT)
			|| (chip->cfg_hot_batt_p == HOT_THD_35_PCT)) {
		btc_cfg |= btc_value[chip->cfg_hot_batt_p];
		mask |= BTC_HOT_MASK;
	}

	if ((chip->cfg_cold_batt_p == COLD_THD_70_PCT) ||
			(chip->cfg_cold_batt_p == COLD_THD_80_PCT)) {
		btc_cfg |= btc_value[chip->cfg_cold_batt_p];
		mask |= BTC_COLD_MASK;
	}

	if (!chip->cfg_btc_disabled) {
		mask |= BTC_COMP_EN_MASK;
		btc_cfg |= BTC_COMP_EN_MASK;
	}

	pr_debug("BTC configuration mask=%x\n", btc_cfg);

	rc = qpnp_lbc_masked_write(chip,
			chip->bat_if_base + BAT_IF_BTC_CTRL,
			mask, btc_cfg);
	if (rc)
		pr_err("Failed to configure BTC rc=%d\n", rc);

	return rc;
}

#define QPNP_LBC_VBATWEAK_MIN_UV        3000000
#define QPNP_LBC_VBATWEAK_MAX_UV        3581250
#define QPNP_LBC_VBATWEAK_STEP_UV       18750
static int qpnp_lbc_vbatweak_set(struct qpnp_lbc_chip *chip, int voltage)
{
	u8 reg_val;
	int rc;

	if (voltage < QPNP_LBC_VBATWEAK_MIN_UV ||
			voltage > QPNP_LBC_VBATWEAK_MAX_UV) {
		rc = -EINVAL;
	} else {
		reg_val = (voltage - QPNP_LBC_VBATWEAK_MIN_UV) /
					QPNP_LBC_VBATWEAK_STEP_UV;
		pr_debug("VBAT_WEAK=%d setting %02x\n",
				chip->cfg_batt_weak_voltage_uv, reg_val);
		rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_VBAT_WEAK_REG,
					&reg_val, 1);
		if (rc)
			pr_err("Failed to set VBAT_WEAK rc=%d\n", rc);
	}

	return rc;
}

#define QPNP_LBC_VBAT_MIN_MV		4000
#define QPNP_LBC_VBAT_MAX_MV		4775
#define QPNP_LBC_VBAT_STEP_MV		25
static int qpnp_lbc_vddsafe_set(struct qpnp_lbc_chip *chip, int voltage)
{
	u8 reg_val;
	int rc;

	if (voltage < QPNP_LBC_VBAT_MIN_MV
			|| voltage > QPNP_LBC_VBAT_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	reg_val = (voltage - QPNP_LBC_VBAT_MIN_MV) / QPNP_LBC_VBAT_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, reg_val);
	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_VDD_SAFE_REG,
				&reg_val, 1);
	if (rc)
		pr_err("Failed to set VDD_SAFE rc=%d\n", rc);

	return rc;
}

static int qpnp_lbc_vddmax_set(struct qpnp_lbc_chip *chip, int voltage)
{
	u8 reg_val;
	int rc, trim_val;
	unsigned long flags;

	if (voltage < QPNP_LBC_VBAT_MIN_MV
			|| voltage > QPNP_LBC_VBAT_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	reg_val = (voltage - QPNP_LBC_VBAT_MIN_MV) / QPNP_LBC_VBAT_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, reg_val);
	rc = __qpnp_lbc_write(chip->spmi, chip->chgr_base + CHG_VDD_MAX_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to set VDD_MAX rc=%d\n", rc);
		goto out;
	}

	/* Update trim value */
	if (chip->supported_feature_flag & VDD_TRIM_SUPPORTED) {
		trim_val = qpnp_lbc_get_trim_val(chip);
		reg_val = (trim_val & VDD_TRIM3_MASK) << VDD_TRIM3_SHIFT;
		rc = __qpnp_lbc_secure_masked_write(chip->spmi,
				chip->misc_base, MISC_TRIM3_REG,
				MISC_TRIM3_VDD_MASK, reg_val);
		if (rc) {
			pr_err("Failed to set MISC_TRIM3_REG rc=%d\n", rc);
			goto out;
		}

		reg_val = (trim_val & VDD_TRIM4_MASK) << VDD_TRIM4_SHIFT;
		rc = __qpnp_lbc_secure_masked_write(chip->spmi,
				chip->misc_base, MISC_TRIM4_REG,
				MISC_TRIM4_VDD_MASK, reg_val);
		if (rc) {
			pr_err("Failed to set MISC_TRIM4_REG rc=%d\n", rc);
			goto out;
		}

		chip->delta_vddmax_uv = qpnp_lbc_get_trim_voltage(trim_val);
		if (chip->delta_vddmax_uv == -EINVAL) {
			pr_err("Invalid trim voltage=%d\n",
					chip->delta_vddmax_uv);
			rc = -EINVAL;
			goto out;
		}

		pr_debug("VDD_MAX delta=%d trim value=%x\n",
				chip->delta_vddmax_uv, trim_val);
	}

out:
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);
	return rc;
}

static int qpnp_lbc_set_appropriate_vddmax(struct qpnp_lbc_chip *chip)
{
	int rc;

	if (chip->bat_is_cool)
		rc = qpnp_lbc_vddmax_set(chip, chip->cfg_cool_bat_mv);
	else if (chip->bat_is_warm)
		rc = qpnp_lbc_vddmax_set(chip, chip->cfg_warm_bat_mv);
	else
		rc = qpnp_lbc_vddmax_set(chip, chip->cfg_max_voltage_mv);
	if (rc)
		pr_err("Failed to set appropriate vddmax rc=%d\n", rc);

	return rc;
}

#define QPNP_LBC_MIN_DELTA_UV			13000
static void qpnp_lbc_adjust_vddmax(struct qpnp_lbc_chip *chip, int vbat_uv)
{
	int delta_uv, prev_delta_uv, rc;

	prev_delta_uv =  chip->delta_vddmax_uv;
	delta_uv = (int)(chip->cfg_max_voltage_mv * 1000) - vbat_uv;

	/*
	 * If delta_uv is positive, apply trim if delta_uv > 13mv
	 * If delta_uv is negative always apply trim.
	 */
	if (delta_uv > 0 && delta_uv < QPNP_LBC_MIN_DELTA_UV) {
		pr_debug("vbat is not low enough to increase vdd\n");
		return;
	}

	pr_debug("vbat=%d current delta_uv=%d prev delta_vddmax_uv=%d\n",
			vbat_uv, delta_uv, chip->delta_vddmax_uv);
	chip->delta_vddmax_uv = delta_uv + chip->delta_vddmax_uv;
	pr_debug("new delta_vddmax_uv  %d\n", chip->delta_vddmax_uv);
	rc = qpnp_lbc_set_appropriate_vddmax(chip);
	if (rc) {
		pr_err("Failed to set appropriate vddmax rc=%d\n", rc);
		chip->delta_vddmax_uv = prev_delta_uv;
	}
}

#define QPNP_LBC_VINMIN_MIN_MV		4200
#define QPNP_LBC_VINMIN_MAX_MV		5037
#define QPNP_LBC_VINMIN_STEP_MV		27
static int qpnp_lbc_vinmin_set(struct qpnp_lbc_chip *chip, int voltage)
{
	u8 reg_val;
	int rc;

	if ((voltage < QPNP_LBC_VINMIN_MIN_MV)
			|| (voltage > QPNP_LBC_VINMIN_MAX_MV)) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	reg_val = (voltage - QPNP_LBC_VINMIN_MIN_MV) / QPNP_LBC_VINMIN_STEP_MV;
	pr_debug("VIN_MIN=%d setting %02x\n", voltage, reg_val);
	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_VIN_MIN_REG,
				&reg_val, 1);
	if (rc)
		pr_err("Failed to set VIN_MIN rc=%d\n", rc);

	return rc;
}

#define QPNP_LBC_IBATSAFE_MIN_MA	90
#define QPNP_LBC_IBATSAFE_MAX_MA	1440
#define QPNP_LBC_I_STEP_MA		90
static int qpnp_lbc_ibatsafe_set(struct qpnp_lbc_chip *chip, int safe_current)
{
	u8 reg_val;
	int rc;

	if (safe_current < QPNP_LBC_IBATSAFE_MIN_MA
			|| safe_current > QPNP_LBC_IBATSAFE_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", safe_current);
		return -EINVAL;
	}

	reg_val = (safe_current - QPNP_LBC_IBATSAFE_MIN_MA)
			/ QPNP_LBC_I_STEP_MA;
	pr_debug("Ibate_safe=%d setting %02x\n", safe_current, reg_val);

	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_IBAT_SAFE_REG,
				&reg_val, 1);
	if (rc)
		pr_err("Failed to set IBAT_SAFE rc=%d\n", rc);

	return rc;
}

#define QPNP_LBC_IBATMAX_MIN	90
/* limit the max charging current to 1100 mA */
#ifdef CONFIG_HUAWEI_KERNEL
#define QPNP_LBC_IBATMAX_MAX	1100
#else
#define QPNP_LBC_IBATMAX_MAX	1440
#endif
/*
 * Set maximum current limit from charger
 * ibat =  System current + charging current
 */

/* Use another mothed to reduce the power consumption of FTM mode. */

static int qpnp_lbc_ibatmax_set(struct qpnp_lbc_chip *chip, int chg_current)
{
	u8 reg_val;
	int rc;

	if (chg_current > QPNP_LBC_IBATMAX_MAX)
		pr_debug("bad mA=%d clamping current\n", chg_current);

	chg_current = clamp(chg_current, QPNP_LBC_IBATMAX_MIN,
						QPNP_LBC_IBATMAX_MAX);
#ifdef CONFIG_HUAWEI_KERNEL
	chg_current = min(chg_current, QPNP_LBC_IBATMAX_MAX);
#endif
	reg_val = (chg_current - QPNP_LBC_IBATMAX_MIN) / QPNP_LBC_I_STEP_MA;
/* Use another mothed to reduce the power consumption of FTM mode. */
	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_IBAT_MAX_REG,
				&reg_val, 1);
	if (rc)
		pr_err("Failed to set IBAT_MAX rc=%d\n", rc);
	else
		chip->prev_max_ma = chg_current;

	return rc;
}

#define QPNP_LBC_TCHG_MIN	4
#define QPNP_LBC_TCHG_MAX	512
#define QPNP_LBC_TCHG_STEP	4
static int qpnp_lbc_tchg_max_set(struct qpnp_lbc_chip *chip, int minutes)
{
	u8 reg_val = 0;
	int rc;

	minutes = clamp(minutes, QPNP_LBC_TCHG_MIN, QPNP_LBC_TCHG_MAX);

	/* Disable timer */
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_TCHG_MAX_EN_REG,
						CHG_TCHG_MAX_EN_BIT, 0);
	if (rc) {
		pr_err("Failed to write tchg_max_en rc=%d\n", rc);
		return rc;
	}

	reg_val = (minutes / QPNP_LBC_TCHG_STEP) - 1;

	pr_debug("TCHG_MAX=%d mins setting %x\n", minutes, reg_val);
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_TCHG_MAX_REG,
						CHG_TCHG_MAX_MASK, reg_val);
	if (rc) {
		pr_err("Failed to write tchg_max_reg rc=%d\n", rc);
		return rc;
	}

	/* Enable timer */
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_TCHG_MAX_EN_REG,
				CHG_TCHG_MAX_EN_BIT, CHG_TCHG_MAX_EN_BIT);
	if (rc) {
		pr_err("Failed to write tchg_max_en rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int qpnp_lbc_vbatdet_override(struct qpnp_lbc_chip *chip, int ovr_val)
{
	int rc;
	u8 reg_val;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	spin_lock_irqsave(&chip->hw_access_lock, flags);

	rc = __qpnp_lbc_read(spmi, chip->chgr_base + CHG_COMP_OVR1,
				&reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
						chip->chgr_base, rc);
		goto out;
	}
	pr_debug("addr = 0x%x read 0x%x\n", chip->chgr_base, reg_val);

	reg_val &= ~CHG_VBAT_DET_OVR_MASK;
	reg_val |= ovr_val & CHG_VBAT_DET_OVR_MASK;

	pr_debug("writing to base=%x val=%x\n", chip->chgr_base, reg_val);

	rc = __qpnp_lbc_secure_write(spmi, chip->chgr_base, CHG_COMP_OVR1,
					&reg_val, 1);
	if (rc)
		pr_err("spmi write failed: addr=%03X, rc=%d\n",
						chip->chgr_base, rc);

out:
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);
	return rc;
}

static int get_prop_battery_voltage_now(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}

	return results.physical;
}

#ifdef CONFIG_HUAWEI_DSM
static int get_prop_battery_id(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &results);
	if (rc) {
		pr_err("Unable to read battery_id rc=%d\n", rc);
		return 0;
	}

	return results.physical;
}

static int get_prop_vbus_uv(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read vbus rc=%d\n", rc);
		return 0;
	}

	return results.physical;
}

static int get_prop_vchg_uv(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, VCHG_SNS, &results);
	if (rc) {
		pr_err("Unable to read vchg rc=%d\n", rc);
		return 0;
	}

	return results.physical;
}

static int get_prop_vcoin_uv(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, VCOIN, &results);
	if (rc) {
		pr_err("Unable to read vcoin rc=%d\n", rc);
		return 0;
	}

	return results.physical;
}

static int is_usb_ovp(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	u8 usbin_valid_rt_sts = 0;
	rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + USB_PTH_STS_REG,
				&usbin_valid_rt_sts, 1);
	if (rc) {
			pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->usb_chgpth_base + USB_PTH_STS_REG, rc);
				return rc;
	}

	return ((usbin_valid_rt_sts & USB_VALID_MASK)== USB_VALID_OVP_VALUE) ? 1 : 0;

}
#endif

static int get_prop_batt_present(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + BAT_IF_PRES_STATUS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read battery status read failed rc=%d\n",
				rc);
		return 0;
	}

	return (reg_val & BATT_PRES_MASK) ? 1 : 0;
}

#ifdef CONFIG_HUAWEI_KERNEL
#define BATT_TEMP_MAX    600
#endif

static int get_prop_batt_health(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;
#ifdef CONFIG_HUAWEI_KERNEL
	int temp = 0;

    rc = get_prop_batt_present(chip);
    if(!rc)
    {
        return POWER_SUPPLY_HEALTH_DEAD;
    }
#endif

	rc = qpnp_lbc_read(chip, chip->bat_if_base + BAT_IF_TEMP_STATUS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read battery health rc=%d\n", rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

#ifdef CONFIG_HUAWEI_KERNEL
    temp = get_prop_batt_temp(chip);
    if(temp >= BATT_TEMP_MAX)
    {
        return POWER_SUPPLY_HEALTH_OVERHEAT;
    }
    else
    {
        if (chip->bat_is_cool)
            return POWER_SUPPLY_HEALTH_COOL;
        if (chip->bat_is_warm)
            return POWER_SUPPLY_HEALTH_WARM;

        return POWER_SUPPLY_HEALTH_GOOD;
    }
#else
	if (BATT_TEMP_HOT_MASK & reg_val)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (!(BATT_TEMP_COLD_MASK & reg_val))
		return POWER_SUPPLY_HEALTH_COLD;
	if (chip->bat_is_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	if (chip->bat_is_warm)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_GOOD;
#endif
}

static int get_prop_charge_type(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	rc = qpnp_lbc_read(chip, chip->chgr_base + INT_RT_STS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt sts %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	if (reg_val & FAST_CHG_ON_IRQ)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int get_prop_batt_status(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	if (qpnp_lbc_is_usb_chg_plugged_in(chip) && chip->chg_done)
		return POWER_SUPPLY_STATUS_FULL;

	rc = qpnp_lbc_read(chip, chip->chgr_base + INT_RT_STS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt sts rc= %d\n", rc);
#ifdef CONFIG_HUAWEI_KERNEL
		return POWER_SUPPLY_STATUS_UNKNOWN;
#else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
#endif
	}

#ifdef CONFIG_HUAWEI_KERNEL
	if (bad_temp_flag == 1)
	{
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
#endif

	if (reg_val & FAST_CHG_ON_IRQ)
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

#ifdef CONFIG_HUAWEI_KERNEL
int hw_get_prop_batt_status(void)
{
	if(!global_chip)
	{
		pr_err("global_chip is not init ready \n");
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
	else
	{
		return get_prop_batt_status(global_chip);
	}
}
EXPORT_SYMBOL(hw_get_prop_batt_status);
#endif

static int get_prop_current_now(struct qpnp_lbc_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}

#define DEFAULT_CAPACITY	50
static int get_prop_capacity(struct qpnp_lbc_chip *chip)
{
	union power_supply_propval ret = {0,};
	int soc, battery_status, charger_in;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;
#ifdef CONFIG_HUAWEI_KERNEL
	if (chip->cfg_use_fake_battery || (!get_prop_batt_present(chip)&&qpnp_lbc_is_usb_chg_plugged_in(chip)))
#else
	if (chip->cfg_use_fake_battery || !get_prop_batt_present(chip))
#endif
		return DEFAULT_CAPACITY;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		mutex_lock(&chip->chg_enable_lock);
		if (chip->chg_done)
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CAPACITY, &ret);
		battery_status = get_prop_batt_status(chip);
		charger_in = qpnp_lbc_is_usb_chg_plugged_in(chip);
#ifdef CONFIG_HUAWEI_KERNEL
		/* reset chg_done flag if capacity not 100% or cfg_soc_resume_charging*/
		if ((ret.intval < 100 || chip->cfg_soc_resume_charging)
							&& chip->chg_done) {
#endif
			chip->chg_done = false;
			power_supply_changed(&chip->batt_psy);
		}
		if (battery_status != POWER_SUPPLY_STATUS_CHARGING
				&& charger_in
				&& !chip->cfg_charging_disabled
#ifdef CONFIG_HUAWEI_KERNEL
				&& !chip->resuming_charging
#endif
#ifdef CONFIG_HUAWEI_KERNEL
            && chip->cfg_soc_resume_charging) {
#endif
			pr_info("resuming charging at %d%% soc\n",
					ret.intval);
#ifdef CONFIG_HUAWEI_KERNEL
			chip->resuming_charging = true;
#endif
			if (!chip->cfg_disable_vbatdet_based_recharge)
				qpnp_lbc_vbatdet_override(chip, OVERRIDE_0);
			qpnp_lbc_charger_enable(chip, SOC, 1);
		}
		mutex_unlock(&chip->chg_enable_lock);

		soc = ret.intval;
		if (soc == 0) {
			if (!qpnp_lbc_is_usb_chg_plugged_in(chip))
				pr_warn_ratelimited("Batt 0, CHG absent\n");
		}
#ifdef CONFIG_HUAWEI_KERNEL
		if(fake_capacity != -1)
		{
			soc = fake_capacity;
		}
#endif
		return soc;
	} else {
		pr_debug("No BMS supply registered return %d\n",
							DEFAULT_CAPACITY);
	}

	/*
	 * Return default capacity to avoid userspace
	 * from shutting down unecessarily
	 */
	return DEFAULT_CAPACITY;
}

#ifdef CONFIG_HUAWEI_KERNEL
#define DEFAULT_TEMP		90
#else
#define DEFAULT_TEMP		250
#endif
static int get_prop_batt_temp(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->cfg_use_fake_battery || !get_prop_batt_present(chip))
		return DEFAULT_TEMP;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp %d, %lld\n", results.adc_code,
							results.physical);

	return (int)results.physical;
}

static void get_temp_ctrl_info_from_max77819(struct qpnp_lbc_chip *chip)
{
	static const struct max77819_temp_control_info* pmax_temp_ctrl_info = NULL;
	if (!pmax_temp_ctrl_info)
	{
		pmax_temp_ctrl_info = max77819_charger_get_temp_ctrl_info();
		if (pmax_temp_ctrl_info)
		{
			chip->cfg_cold_bat_decidegc = pmax_temp_ctrl_info->cold_bat_degree;
			chip->cfg_hot_bat_decidegc = pmax_temp_ctrl_info->hot_bat_degree;
			chip->cfg_cool_bat_decidegc = pmax_temp_ctrl_info->cool_bat_degree;
			chip->cfg_cool_bat_chg_ma = pmax_temp_ctrl_info->imaxua_cool_bat / 1000;
			chip->cfg_cool_bat_mv = pmax_temp_ctrl_info->vmaxuv_cool_bat / 1000;
			chip->cfg_warm_bat_decidegc = pmax_temp_ctrl_info->warm_bat_degree;
			chip->cfg_warm_bat_chg_ma = pmax_temp_ctrl_info->imaxua_warm_bat / 1000;
			chip->cfg_warm_bat_mv = pmax_temp_ctrl_info->vmaxuv_warm_bat / 1000;
		}
	}
}

#ifdef CONFIG_HUAWEI_KERNEL
/*===========================================
FUNCTION: get_running_test_result
DESCRIPTION: For running test apk to get the running test result and status
IPNUT:	qpnp_lbc_chip *chip
RETURN:	a int value, we use bit0 to bit10 to tell running test apk the test 
result and status, if bit0 is 0, the result is fail, bit5 to bit11 is the failed reason
if bit0 is 1, the result is pass.
=============================================*/
static int get_running_test_result(struct qpnp_lbc_chip *chip)
{
	int result = 0;
	int cur_status = 0;
	int is_temp_vol_current_ok = 1;
	int vol = 0, temp = 0, health = 0, current_ma =0, capacity;
	u8 usbin_valid_rt_sts = 0;
	int rc;
	int mode = 0;
	int error = 0;
	union power_supply_propval val = {0};

	if(!chip->use_other_charger){
		/*Get battery props, for 8916 except G760 */
		cur_status = get_prop_batt_status(chip);
		current_ma = get_prop_current_now(chip);
		vol = get_prop_battery_voltage_now(chip);
		temp = get_prop_batt_temp(chip);
		capacity = get_prop_capacity(chip);
		health = get_prop_batt_health(chip);
	}else if (chip->maxim_charger && chip->maxim_charger->get_property){
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_STATUS,&val);
		cur_status = val.intval;
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CURRENT_NOW,&val);
		current_ma = val.intval;
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_VOLTAGE_NOW,&val);
		vol = val.intval;
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_TEMP,&val);
		temp = val.intval;
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CAPACITY,&val);
		capacity = val.intval;
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_HEALTH,&val);
		health = val.intval;
		mode = get_max77819_boost_mode();
		get_temp_ctrl_info_from_max77819(chip);
	}else{
		/* Get battery props, for G760 */
		if(chip->ti_charger && chip->ti_charger->get_property){
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_STATUS,&val);
			cur_status = val.intval;
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_CURRENT_NOW,&val);
			current_ma = val.intval;
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_VOLTAGE_NOW,&val);
			vol = val.intval;
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_TEMP,&val);
			temp = val.intval;
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_CAPACITY,&val);
			capacity = val.intval;
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_HEALTH,&val);
			health = val.intval;
		}
		mode = is_bq24152_in_boost_mode();
	}
	pr_debug("get_running_test_result info: usb=%d batt_pres=%d batt_volt=%d batt_temp=%d"
				" cur_status=%d current_ma=%d setting status=%d\n",
				qpnp_lbc_is_usb_chg_plugged_in(chip),
				get_prop_batt_present(chip),
				vol,
				temp,
				cur_status,
				current_ma,
				chip->running_test_settled_status
				);

	if((CHARGE_OCP_THR > current_ma) || (BATTERY_OCP_THR < current_ma)){
		result |= OCP_ABNORML;
		is_temp_vol_current_ok = 0;
		pr_info("Find OCP! current_ma is %d\n", current_ma);
	}

	if((BATTERY_VOL_THR_HI < vol) || (BATTERY_VOL_THR_LO > vol)){
		result |= BATTERY_VOL_ABNORML;
		is_temp_vol_current_ok = 0;
		pr_info("Battery voltage is abnormal! voltage is %d\n", vol);
	}

	if((BATTERY_TEMP_HI < temp) || (BATTERY_TEMP_LO > temp)){
		result |= BATTERY_TEMP_ABNORML;
		is_temp_vol_current_ok = 0;
		pr_info("Battery temperature is abnormal! temp is %d\n", temp);
	}

	if(!is_temp_vol_current_ok){
		result |= CHARGE_STATUS_FAIL;
		pr_info("running test find abnormal battery status, the result is 0x%x\n", result);
		return result;
	}

	if(cur_status == chip->running_test_settled_status){
		result |= CHARGE_STATUS_PASS;
		return result;

	}else if((POWER_SUPPLY_STATUS_CHARGING == cur_status)
			&&(POWER_SUPPLY_STATUS_DISCHARGING == chip->running_test_settled_status)){
		result |= CHARGE_STATUS_FAIL;
		pr_info("get_running_test_result: usb=%d batt_pres=%d batt_volt=%d batt_temp=%d"
				" capacity=%d cur_status=%d current_ma=%d setting status=%d result=0x%x\n",
				qpnp_lbc_is_usb_chg_plugged_in(chip),
				get_prop_batt_present(chip),
				vol,
				temp,
				capacity,
				cur_status,
				current_ma,
				chip->running_test_settled_status,
				result
				);
		return result;

	}else if(POWER_SUPPLY_STATUS_CHARGING == chip->running_test_settled_status){
		if((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
			&& (BATT_FULL == capacity) && (get_prop_batt_present(chip))
				&& qpnp_lbc_is_usb_chg_plugged_in(chip)){
			cur_status = POWER_SUPPLY_STATUS_FULL;
		}

		if(POWER_SUPPLY_STATUS_FULL == cur_status){
			result |= BATTERY_FULL;
		}

		if(!qpnp_lbc_is_usb_chg_plugged_in(chip)){
			result |= USB_NOT_PRESENT;
		}

		if(chip->use_other_charger && (1 == mode)){
			result |= REGULATOR_BOOST;
		}

		if((vol >= ((chip->cfg_warm_bat_mv - WARM_VOL_BUFFER)*1000))
			&& (WARM_TEMP_THR <= temp)){
			result |= CHARGE_LIMIT;
		}

		if((POWER_SUPPLY_STATUS_NOT_CHARGING == cur_status)
			&& (HOT_TEMP_THR > temp)){
			result |= CHARGE_LIMIT;
			pr_info("settled_status = %d cur_status = %d temp = %d\n",
					chip->running_test_settled_status, cur_status, temp);
		}

		/* G760 return discharging when stop charging in hot or cold environment */
		/* we use battery temperature to determine whether it pass or not */
		if((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
			&& chip->use_other_charger){
			if(((temp >= (chip->cfg_hot_bat_decidegc - COLD_HOT_TEMP_BUFFER))&&(HOT_TEMP_THR > temp))
				||(temp <= (chip->cfg_cold_bat_decidegc + COLD_HOT_TEMP_BUFFER))){
				result |= CHARGE_LIMIT;
				pr_info("settled_status %d cur_status=%d temp=%d "
						"chip->cfg_hot_bat_decidegc=%d "
						"chip->cfg_cold_bat_decidegc=%d\n",
						chip->running_test_settled_status, cur_status, temp,
						chip->cfg_hot_bat_decidegc,
						chip->cfg_cold_bat_decidegc);
			}
		}

		if((POWER_SUPPLY_HEALTH_OVERHEAT == health)
			|| (POWER_SUPPLY_HEALTH_COLD ==health)){
			result |= BATTERY_HEALTH;
		}

		rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + USB_PTH_STS_REG,
				&usbin_valid_rt_sts, 1);
		if (rc) {
			pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->usb_chgpth_base + USB_PTH_STS_REG, rc);
		}else{
			if ((usbin_valid_rt_sts & USB_VALID_MASK)== USB_VALID_OVP_VALUE) {
				result |= CHARGER_OVP;
			}
		}

		if(!get_prop_batt_present(chip)){
			result |= BATTERY_ABSENT;
		}

		if((POWER_SUPPLY_STATUS_UNKNOWN == cur_status)
			&& chip->use_other_charger){
			error = get_bq2415x_fault_status();
			if((ERROR_VBUS_OVP == error) ||(ERROR_BATT_OVP == error)
				||(ERROR_THERMAL_SHUTDOWN == error)){
				result |= BQ24152_FATAL_FAULT;
				pr_info("find bq24152 fatal fault! error = %d\n", error);
			}
		}

		if(result & PASS_MASK){
			result |= CHARGE_STATUS_PASS;
		}else{
			result |= CHARGE_STATUS_FAIL;
			pr_info("get_running_test_result: usb=%d batt_pres=%d batt_volt=%d batt_temp=%d"
				" capacity=%d cur_status=%d current_ma=%d setting status=%d result=0x%x\n",
				qpnp_lbc_is_usb_chg_plugged_in(chip),
				get_prop_batt_present(chip),
				vol,
				temp,
				capacity,
				cur_status,
				current_ma,
				chip->running_test_settled_status,
				result
				);
		}

		return result;
	}else{
		pr_info("other else status!");
		/* if the setting status is discharging, meanwhile */
		/* if(cur_status != POWER_SUPPLY_STATUS_CHARGING*/
		/* && cur_status != POWER_SUPPLY_STATUS_DISCHARGING) */
		/* We return 1(PASS) directly, as when set discharging*/
		/* it do not need to care high temperature, battery full or unknow*/
		pr_info("usb=%d batt_pres=%d batt_volt=%d batt_temp=%d"
				" cur_status=%d current_ma=%d setting status=%d\n",
				qpnp_lbc_is_usb_chg_plugged_in(chip),
				get_prop_batt_present(chip),
				vol,
				temp,
				cur_status,
				current_ma,
				chip->running_test_settled_status
				);
		return 1;
	}
}
/*===========================================
FUNCTION: get_prop_charge_log
DESCRIPTION: to get some pmic charging regs for chargelog.sh
IPNUT:	qpnp_lbc_chip *chip
RETURN:	a int value
chip->log_buf is used to save the regs values
=============================================*/
static int get_prop_charge_log(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	u8 bat_sts = 0, chg_sts = 0, usb_sts = 0, chg_ctrl = 0, usb_susp = 0;

	rc = qpnp_lbc_read(chip, INT_RT_STS(chip->bat_if_base), &bat_sts, 1);
	if (rc)
		pr_err("failed to read batt_sts rc=%d\n", rc);

	rc = qpnp_lbc_read(chip, INT_RT_STS(chip->chgr_base), &chg_sts, 1);
	if (rc)
		pr_err("failed to read chgr_sts rc=%d\n", rc);

	rc = qpnp_lbc_read(chip, INT_RT_STS(chip->usb_chgpth_base), &usb_sts, 1);
	if (rc)
		pr_err("failed to read usb_sts rc=%d\n", rc);

	rc = qpnp_lbc_read(chip, chip->chgr_base + CHG_CTRL_REG, &chg_ctrl, 1);
	if (rc)
		pr_err("failed to read chg_ctrl sts %d\n", rc);

	rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + USB_SUSP_REG, &usb_susp, 1);
	if (rc)
		pr_err("failed to read usb_sts rc=%d\n", rc);

	usb_susp = usb_susp&0x1;

	snprintf(chip->log_buf, sizeof(chip->log_buf), "%d %u %u %u %u %u",
		chip->resuming_charging, bat_sts, chg_sts,
		usb_sts, chg_ctrl, usb_susp);

	return 0;
}

/*===========================================
FUNCTION: qpnp_lbc_is_in_vin_min_loop
DESCRIPTION: to get pmic8916 vin_min loop value
IPNUT:	qpnp_lbc_chip *chip
RETURN:	a int value, 1 means in vin_min loop; 0 means not in vin_min loop
=============================================*/
int qpnp_lbc_is_in_vin_min_loop(struct qpnp_lbc_chip *chip)
{
	u8 reg_val = 0;
	int rc;

	rc = qpnp_lbc_read(chip, chip->chgr_base + CHG_STATUS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read chg status rc=%d\n", rc);
		return rc;
	}

	pr_debug("CHG_STATUS_REG %x\n", reg_val);
	return (reg_val & CHG_VIN_MIN_LOOP_BIT) ? 1 : 0;
}
EXPORT_SYMBOL(qpnp_lbc_is_in_vin_min_loop);
#endif

static void qpnp_lbc_set_appropriate_current(struct qpnp_lbc_chip *chip)
{
	unsigned int chg_current = chip->usb_psy_ma;

	if (chip->bat_is_cool && chip->cfg_cool_bat_chg_ma)
		chg_current = min(chg_current, chip->cfg_cool_bat_chg_ma);
	if (chip->bat_is_warm && chip->cfg_warm_bat_chg_ma)
		chg_current = min(chg_current, chip->cfg_warm_bat_chg_ma);
	if (chip->therm_lvl_sel != 0 && chip->thermal_mitigation)
		chg_current = min(chg_current,
			chip->thermal_mitigation[chip->therm_lvl_sel]);
	pr_info("setting charger current %d mA\n", chg_current);
	qpnp_lbc_ibatmax_set(chip, chg_current);
}

static void qpnp_batt_external_power_changed(struct power_supply *psy)
{
	struct qpnp_lbc_chip *chip = container_of(psy, struct qpnp_lbc_chip,
								batt_psy);
	union power_supply_propval ret = {0,};
	int current_ma;
	unsigned long flags;
	int rc = 0;
	spin_lock_irqsave(&chip->ibat_change_lock, flags);
	if (!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("bms");

#ifdef CONFIG_HUAWEI_KERNEL
	if (!chip->ti_charger && chip->use_other_charger)
		chip->ti_charger = power_supply_get_by_name("ti-charger");
	if (!chip->maxim_charger && chip->use_other_charger)
		chip->maxim_charger = power_supply_get_by_name("max77819-charger");
	if (chip->ti_charger || chip->maxim_charger){
		rc = qpnp_lbc_charger_enable(chip, USER, 0);
		if (rc){
			pr_err("Failed to disable charging rc=%d\n",rc);
		}
	}
#endif
	if (qpnp_lbc_is_usb_chg_plugged_in(chip)) {
		chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
		current_ma = ret.intval / 1000;

#ifdef CONFIG_HUAWEI_KERNEL
		if (!factory_diag_flag && input_current_max_flag) {
			current_ma = max(current_ma, input_current_max_ma);
		}
		current_ma = min(current_ma, hot_design_current);
#endif

		if (current_ma == chip->prev_max_ma)
			goto skip_current_config;

		/* Disable charger in case of reset or suspend event */
		if (current_ma <= 2 && !chip->cfg_use_fake_battery
				&& get_prop_batt_present(chip)) {
			qpnp_lbc_charger_enable(chip, CURRENT, 0);
			chip->usb_psy_ma = QPNP_CHG_I_MAX_MIN_90;
			qpnp_lbc_set_appropriate_current(chip);
#ifdef CONFIG_HUAWEI_KERNEL
			if (chip->ti_charger)
				power_supply_set_current_limit(chip->ti_charger,0);
#endif
		} else {
			chip->usb_psy_ma = current_ma;
			qpnp_lbc_set_appropriate_current(chip);
			qpnp_lbc_charger_enable(chip, CURRENT, 1);
#ifdef CONFIG_HUAWEI_KERNEL
			if (chip->ti_charger)
				power_supply_set_current_limit(chip->ti_charger,current_ma);
#endif
		}
	}

skip_current_config:
	spin_unlock_irqrestore(&chip->ibat_change_lock, flags);
	pr_debug("power supply changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);
}

#ifdef CONFIG_HUAWEI_KERNEL
/* get maximum current limit which is set to IBAT_MAX register */
static int qpnp_lbc_ibatmax_get(struct qpnp_lbc_chip *chip)
{
	int rc, iusbmax_ma;
	u8 iusbmax = 0;

	rc = qpnp_lbc_read(chip, chip->chgr_base + CHG_IBAT_MAX_REG, &iusbmax, 1);
	if (rc) {
		pr_err("failed to read IUSB_MAX rc=%d\n", rc);
		return 0;
	}

	iusbmax_ma = QPNP_LBC_IBATMAX_MIN + iusbmax * QPNP_LBC_I_STEP_MA;
	pr_debug(" = 0x%02x, iusbmax_ma = %d\n", iusbmax, iusbmax_ma);

	return iusbmax_ma;
}
static int get_prop_full_design(struct qpnp_lbc_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &ret);
		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}

#endif

static int qpnp_lbc_system_temp_level_set(struct qpnp_lbc_chip *chip,
								int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;
	unsigned long flags;

	if (!chip->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->cfg_thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				chip->cfg_thermal_levels - 1);
		lvl_sel = chip->cfg_thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	spin_lock_irqsave(&chip->ibat_change_lock, flags);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->cfg_thermal_levels - 1)) {
		/* Disable charging if highest value selected by */
		rc = qpnp_lbc_charger_enable(chip, THERMAL, 0);
		if (rc < 0)
			dev_err(chip->dev,
				"Failed to set disable charging rc %d\n", rc);
		goto out;
	}

	qpnp_lbc_set_appropriate_current(chip);

	if (prev_therm_lvl == chip->cfg_thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Enable charging.
		 */
		rc = qpnp_lbc_charger_enable(chip, THERMAL, 1);
		if (rc < 0) {
			dev_err(chip->dev,
				"Failed to enable charging rc %d\n", rc);
		}
	}
out:
	spin_unlock_irqrestore(&chip->ibat_change_lock, flags);
	return rc;
}

#define MIN_COOL_TEMP		-300
#define MAX_WARM_TEMP		1000
#define HYSTERISIS_DECIDEGC	20

static int qpnp_lbc_configure_jeita(struct qpnp_lbc_chip *chip,
			enum power_supply_property psp, int temp_degc)
{
	int rc = 0;

	if ((temp_degc < MIN_COOL_TEMP) || (temp_degc > MAX_WARM_TEMP)) {
		pr_err("Bad temperature request %d\n", temp_degc);
		return -EINVAL;
	}

	mutex_lock(&chip->jeita_configure_lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_COOL_TEMP:
		if (temp_degc >=
			(chip->cfg_warm_bat_decidegc - HYSTERISIS_DECIDEGC)) {
			pr_err("Can't set cool %d higher than warm %d - hysterisis %d\n",
					temp_degc,
					chip->cfg_warm_bat_decidegc,
					HYSTERISIS_DECIDEGC);
			rc = -EINVAL;
			goto mutex_unlock;
		}
		if (chip->bat_is_cool)
			chip->adc_param.high_temp =
				temp_degc + HYSTERISIS_DECIDEGC;
		else if (!chip->bat_is_warm)
			chip->adc_param.low_temp = temp_degc;

		chip->cfg_cool_bat_decidegc = temp_degc;
		break;
	case POWER_SUPPLY_PROP_WARM_TEMP:
		if (temp_degc <=
		(chip->cfg_cool_bat_decidegc + HYSTERISIS_DECIDEGC)) {
			pr_err("Can't set warm %d higher than cool %d + hysterisis %d\n",
					temp_degc,
					chip->cfg_warm_bat_decidegc,
					HYSTERISIS_DECIDEGC);
			rc = -EINVAL;
			goto mutex_unlock;
		}
		if (chip->bat_is_warm)
			chip->adc_param.low_temp =
				temp_degc - HYSTERISIS_DECIDEGC;
		else if (!chip->bat_is_cool)
			chip->adc_param.high_temp = temp_degc;

		chip->cfg_warm_bat_decidegc = temp_degc;
		break;
	default:
		rc = -EINVAL;
		goto mutex_unlock;
	}

	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");

mutex_unlock:
	mutex_unlock(&chip->jeita_configure_lock);
	return rc;
}

static int qpnp_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
#ifdef CONFIG_HUAWEI_KERNEL
	case POWER_SUPPLY_PROP_FACTORY_DIAG:
       case POWER_SUPPLY_PROP_RESUME_CHARGING:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
#endif
	case POWER_SUPPLY_PROP_COOL_TEMP:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
	case POWER_SUPPLY_PROP_WARM_TEMP:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * End of charge happens only when BMS reports the battery status as full. For
 * charging to end the s/w must put the usb path in suspend. Note that there
 * is no battery fet and usb path suspend is the only control to prevent any
 * current going in to the battery (and the system)
 * Charging can begin only when VBATDET comparator outputs 0. This indicates
 * that the battery is a at a lower voltage than 4% of the vddmax value.
 * S/W can override this comparator to output a favourable value - this is
 * used while resuming charging when the battery hasnt fallen below 4% but
 * the SOC has fallen below the resume threshold.
 *
 * In short, when SOC resume happens:
 * a. overide the comparator to output 0
 * b. enable charging
 *
 * When vbatdet based resume happens:
 * a. enable charging
 *
 * When end of charge happens:
 * a. disable the overrides in the comparator
 *    (may be from a previous soc resume)
 * b. disable charging
 */
static int qpnp_batt_power_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct qpnp_lbc_chip *chip = container_of(psy, struct qpnp_lbc_chip,
								batt_psy);
	int rc = 0;

#ifdef CONFIG_HUAWEI_KERNEL
	union power_supply_propval val_factory_diag = {0,};
#endif

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (val->intval == POWER_SUPPLY_STATUS_FULL &&
				!chip->cfg_float_charge) {
			mutex_lock(&chip->chg_enable_lock);

			/* Disable charging */
			rc = qpnp_lbc_charger_enable(chip, SOC, 0);
			if (rc)
				pr_err("Failed to disable charging rc=%d\n",
						rc);
			else
				chip->chg_done = true;

			/*
			 * Enable VBAT_DET based charging:
			 * To enable charging when VBAT falls below VBAT_DET
			 * and device stays suspended after EOC.
			 */
			if (!chip->cfg_disable_vbatdet_based_recharge) {
				/* No override for VBAT_DET_LO comp */
				rc = qpnp_lbc_vbatdet_override(chip,
							OVERRIDE_NONE);
				if (rc)
					pr_err("Failed to override VBAT_DET rc=%d\n",
							rc);
				else
					qpnp_lbc_enable_irq(chip,
						&chip->irqs[CHG_VBAT_DET_LO]);
			}

			mutex_unlock(&chip->chg_enable_lock);
		}
		break;
	case POWER_SUPPLY_PROP_COOL_TEMP:
		rc = qpnp_lbc_configure_jeita(chip, psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_WARM_TEMP:
		rc = qpnp_lbc_configure_jeita(chip, psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		pr_debug("power supply changed batt_psy\n");
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		chip->cfg_charging_disabled = !(val->intval);
#ifdef CONFIG_HUAWEI_KERNEL
		pr_info("set charging_enabled value is %d\n",val->intval);
		if(chip->cfg_charging_disabled){
			chip->running_test_settled_status = POWER_SUPPLY_STATUS_DISCHARGING;
		}else{
			chip->running_test_settled_status = POWER_SUPPLY_STATUS_CHARGING;
		}
#endif
#ifdef CONFIG_HUAWEI_KERNEL
		if(!chip->use_other_charger)
		{
			rc = qpnp_lbc_charger_enable(chip, USER,
							!chip->cfg_charging_disabled);
			
			if (rc)
				pr_err("Failed to disable charging rc=%d\n", rc);
		}
		else if (chip->maxim_charger && chip->maxim_charger->get_property)
		{
			rc = chip->maxim_charger->set_property(chip->maxim_charger,
				POWER_SUPPLY_PROP_CHARGING_ENABLED,val);
			if (rc < 0)
			{
				pr_err("Maxim charge failed to disable charging rc=%d\n", rc);
			}
		}
		else if (chip->ti_charger && chip->ti_charger->get_property)
		{
			val_factory_diag.intval = !chip->cfg_charging_disabled;
			rc = chip->ti_charger->set_property(chip->ti_charger,
				POWER_SUPPLY_PROP_CHARGING_ENABLED,&val_factory_diag);
			
			if (rc < 0)
				pr_err("TI charge failed to disable charging rc=%d\n", rc);
		}
		else
		{
			pr_err("None other charger set propertye of charging_enabled \n");
		}
#endif
		break;
#ifdef CONFIG_HUAWEI_KERNEL
	case POWER_SUPPLY_PROP_FACTORY_DIAG:
		factory_diag_flag = !(val->intval);
		if(!chip->use_other_charger) 
		{
			if (factory_diag_flag) {
				factory_diag_last_current_ma = qpnp_lbc_ibatmax_get(chip);
				val_factory_diag.intval = QPNP_CHG_I_MAX_MIN_90 * 1000;
				chip->usb_psy->set_property(chip->usb_psy,
						POWER_SUPPLY_PROP_CURRENT_MAX, &val_factory_diag);
			} else {
				if (factory_diag_last_current_ma) {
					val_factory_diag.intval = factory_diag_last_current_ma * 1000;
					chip->usb_psy->set_property(chip->usb_psy,
						POWER_SUPPLY_PROP_CURRENT_MAX, &val_factory_diag);

				}
				factory_diag_last_current_ma = 0;
				}
		}
		else if (chip->maxim_charger && chip->maxim_charger->get_property)
		{
			rc = chip->maxim_charger->set_property(chip->maxim_charger,
				POWER_SUPPLY_PROP_FACTORY_DIAG,val);
			if (rc < 0)
			{
				pr_err("Maxim charge failed to disable charging rc=%d\n", rc);
			}
		}
		else if (chip->ti_charger && chip->ti_charger->get_property)
		{
			if(chip->ti_charger && chip->ti_charger->set_property){
				val_factory_diag.intval = QPNP_CHG_I_MAX_MIN_90;
				chip->ti_charger->set_property(chip->ti_charger,
					POWER_SUPPLY_PROP_CURRENT_MAX,&val_factory_diag);
			}
		}
		else
		{
			pr_err("None other charger set property of factory_daig \n");
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		if (val->intval) {
			input_current_max_ma = val->intval / 1000;
			input_current_max_flag = 1;
		} else {
			input_current_max_flag = 0;
		}
		break;
	case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
		if(!chip->maxim_charger)
		{
			if (val->intval) {
				hot_design_current = val->intval;
			} else {
				hot_design_current = HOT_DESIGN_MAX_CURRENT;
			}
		}
		else if (chip->maxim_charger && chip->maxim_charger->get_property)
		{
			chip->maxim_charger->set_property(chip->maxim_charger,POWER_SUPPLY_PROP_HOT_IBAT_LIMIT,val);
		}
		else
		{
		}
		break;
#endif
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		qpnp_lbc_vinmin_set(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		qpnp_lbc_system_temp_level_set(chip, val->intval);
		break;
#ifdef CONFIG_HUAWEI_KERNEL
	case POWER_SUPPLY_PROP_RESUME_CHARGING:
		chip->cfg_soc_resume_charging = val->intval;
		break;
#endif
	default:
		return -EINVAL;
	}

     if( POWER_SUPPLY_PROP_RESUME_CHARGING !=psp)
     {
#ifdef CONFIG_HUAWEI_KERNEL
	qpnp_batt_external_power_changed(&chip->batt_psy);
#endif
           power_supply_changed(&chip->batt_psy);
       }
	return rc;
}

static int qpnp_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct qpnp_lbc_chip *chip =
		container_of(psy, struct qpnp_lbc_chip, batt_psy);
#ifdef CONFIG_HUAWEI_KERNEL
	int batt_level = 0;
#endif
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (chip->use_other_charger &&
			(chip->maxim_charger && chip->maxim_charger->get_property))
		{
			/* report maxim77819 charging status */
			chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_STATUS,val);
		}
		else
		{
			val->intval = get_prop_batt_status(chip);
#ifdef CONFIG_HUAWEI_KERNEL
			batt_level = get_prop_capacity(chip);
			if(BATT_FULL_LEVEL == batt_level && qpnp_lbc_is_usb_chg_plugged_in(chip)){
				val->intval = POWER_SUPPLY_STATUS_FULL;
			}
#endif
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->cfg_max_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->cfg_min_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_battery_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
#ifdef CONFIG_HUAWEI_KERNEL
		if(!chip->use_other_charger ||
			(chip->maxim_charger && chip->maxim_charger->get_property) || chip->use_only_ti_charger)
		{
			val->intval = get_prop_batt_temp(chip);
		}
		else if (chip->ti_charger && chip->ti_charger->get_property)
		{
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_TEMP,val);
		}
		else
		{
			val->intval = DEFAULT_TEMP;
		}
#endif
		break;
	case POWER_SUPPLY_PROP_COOL_TEMP:
		val->intval = chip->cfg_cool_bat_decidegc;
		break;
	case POWER_SUPPLY_PROP_WARM_TEMP:
		val->intval = chip->cfg_warm_bat_decidegc;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CONFIG_HUAWEI_KERNEL
		if(!chip->use_other_charger)
		{
			val->intval = get_prop_capacity(chip);
		}
		else if (chip->ti_charger && chip->ti_charger->get_property)
		{
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_CAPACITY,val);
		}
		else if (chip->maxim_charger && chip->maxim_charger->get_property)
		{
			chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CAPACITY,val);
		}
		else
		{
			val->intval = DEFAULT_CAPACITY;
		}
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !(chip->cfg_charging_disabled);
#ifdef CONFIG_HUAWEI_KERNEL
		if (chip->maxim_charger && chip->maxim_charger->get_property)
		{
			chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CHARGING_ENABLED,val);
		}
#endif
		break;
#ifdef CONFIG_HUAWEI_KERNEL
	case POWER_SUPPLY_PROP_FACTORY_DIAG:
		val->intval = !(factory_diag_flag);
		if (chip->maxim_charger && chip->maxim_charger->get_property)
		{
			chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_FACTORY_DIAG,val);
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = qpnp_lbc_ibatmax_get(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
#ifdef CONFIG_HUAWEI_KERNEL
		if((!chip->use_other_charger)||(chip->use_only_ti_charger))
		{
			val->intval = get_prop_full_design(chip);
		}
		else if(chip->ti_charger && chip->ti_charger->get_property)
		{
		    chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,val);
		}
		else if (chip->maxim_charger && chip->maxim_charger->get_property)
		{
			chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,val);
		}
		else
		{
			val->intval = 0;
		}
#endif
		break;
	case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
		if(!chip->maxim_charger)
		{
			val->intval = hot_design_current;
		}
		else if (chip->maxim_charger && chip->maxim_charger->get_property)
		{
			chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_HOT_IBAT_LIMIT,val);
		}
		else
		{
			val->intval = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_LOG:
		get_prop_charge_log(chip);
		val->strval = chip->log_buf;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
#endif
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
#ifdef CONFIG_HUAWEI_KERNEL
	case POWER_SUPPLY_PROP_RUNNING_TEST_STATUS:
		val->intval = get_running_test_result(chip);
		break;
#endif
#ifdef CONFIG_HUAWEI_KERNEL
	case POWER_SUPPLY_PROP_RESUME_CHARGING:
             val->intval = chip->cfg_soc_resume_charging;
        break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_HUAWEI_KERNEL
#define HYSTERISIS_DECIDEGC 20
/*====================================================================================
FUNCTION: hw_tm_warm_notification_zone

DESCRIPTION:	when qpnp_tm_state is warm, call this function.to acquire the
				temperature zone type

INPUT:	temperature,struct qpnp_lbc_chip *chip
OUTPUT: NULL
RETURN: hw_high_low_temp_configure_type

======================================================================================*/
static int hw_tm_warm_notification_zone(int temp,struct qpnp_lbc_chip *chip)
{
	if(chip == NULL)
	{
		pr_err("chip is null \n");
		return UNKNOW_ZONE;
	}
	if(temp > chip->cfg_cold_bat_decidegc + HYSTERISIS_DECIDEGC
		&& temp <= chip->cfg_cool_bat_decidegc + HYSTERISIS_DECIDEGC)
	{
		return COLD_COOL_ZONE;
	}
	else if(temp > chip->cfg_cool_bat_decidegc + HYSTERISIS_DECIDEGC
		&& temp <= chip->cfg_warm_bat_decidegc)
	{
		return COOL_WARM_ZONE;
	}
	else if(temp > chip->cfg_warm_bat_decidegc
		&& temp <= chip->cfg_hot_bat_decidegc)
	{
		return WARM_HOT_ZONE;
	}
	else if(temp > chip->cfg_hot_bat_decidegc)
	{
		return HOT_HOT_ZONE;
	}
	else
	{
		pr_err("warm notification error,temp is %d \n",temp);
		return UNKNOW_ZONE;
	}
}
/*====================================================================================
FUNCTION: hw_tm_cool_notification_zone

DESCRIPTION:	when qpnp_tm_state is cool, call this function.to acquire the
				temperature zone type

INPUT:	temperature,struct qpnp_lbc_chip *chip
OUTPUT: NULL
RETURN: hw_high_low_temp_configure_type

======================================================================================*/
static int hw_tm_cool_notification_zone(int temp,struct qpnp_lbc_chip *chip)
{
	if(chip == NULL)
	{
		pr_err("chip is null \n");
		return UNKNOW_ZONE;
	}
	if(temp < chip->cfg_cold_bat_decidegc)
	{
		return COLD_COLD_ZONE;
	}
	else if(temp >= chip->cfg_cold_bat_decidegc
		&& temp < chip->cfg_cool_bat_decidegc)
	{
		return COLD_COOL_ZONE;
	}
	else if(temp >= chip->cfg_cool_bat_decidegc
		&& temp < chip->cfg_warm_bat_decidegc - HYSTERISIS_DECIDEGC)
	{
		return COOL_WARM_ZONE;
	}
	else if(temp >= chip->cfg_warm_bat_decidegc - HYSTERISIS_DECIDEGC
		&& temp < chip->cfg_hot_bat_decidegc - HYSTERISIS_DECIDEGC)
	{
		return WARM_HOT_ZONE;
	}
	else
	{
		pr_err("cold notification error,temp is %d\n",temp);
		return UNKNOW_ZONE;
	}
}
/*====================================================================================
FUNCTION: hw_tm_set_configure

DESCRIPTION:	according the temperature zone type to set voltage,current,adc_param
				which is set to alarm ,and decided to enable charging or not

INPUT:	enum hw_high_low_temp_configure_type zone,struct qpnp_lbc_chip *chip
OUTPUT: NULL
RETURN: NULL

======================================================================================*/
static void hw_tm_set_configure(enum hw_high_low_temp_configure_type zone,struct qpnp_lbc_chip *chip)
{
	bool bat_warm = 0, bat_cool = 0,bad_temp;
	int rc;
	if(chip == NULL)
	{
		pr_err("chip is null \n");
		return ;
	}
	pr_debug("temperature zone type %d\n",zone);
	switch(zone){
	case COLD_COLD_ZONE:
		chip->adc_param.low_temp = chip->cfg_cold_bat_decidegc;
		chip->adc_param.high_temp = chip->cfg_cold_bat_decidegc + HYSTERISIS_DECIDEGC;
		chip->adc_param.state_request = ADC_TM_WARM_THR_ENABLE;
		bat_cool = true;
		bat_warm = false;
		bad_temp = true;
		break;
	case COLD_COOL_ZONE:
		chip->adc_param.low_temp = chip->cfg_cold_bat_decidegc;
		chip->adc_param.high_temp = chip->cfg_cool_bat_decidegc + HYSTERISIS_DECIDEGC;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		bat_cool = true;
		bat_warm = false;
		bad_temp =false;
		break;
	case COOL_WARM_ZONE:
		chip->adc_param.low_temp = chip->cfg_cool_bat_decidegc;
		chip->adc_param.high_temp = chip->cfg_warm_bat_decidegc;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		bat_cool = false;
		bat_warm = false;
		bad_temp =false;
		break;
	case WARM_HOT_ZONE:
		chip->adc_param.low_temp = chip->cfg_warm_bat_decidegc - HYSTERISIS_DECIDEGC;
		chip->adc_param.high_temp = chip->cfg_hot_bat_decidegc;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		bat_cool = false;
		bat_warm = true;
		bad_temp = false;
		break;
	case HOT_HOT_ZONE:
		chip->adc_param.low_temp = chip->cfg_hot_bat_decidegc - HYSTERISIS_DECIDEGC;
		chip->adc_param.high_temp = chip->cfg_hot_bat_decidegc;
		chip->adc_param.state_request = ADC_TM_COOL_THR_ENABLE;
		bat_cool = false;
		bat_warm = true;
		bad_temp = true;
		break;
	default:
		chip->adc_param.low_temp = chip->cfg_cool_bat_decidegc;
		chip->adc_param.high_temp = chip->cfg_warm_bat_decidegc;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		bat_cool = false;
		bat_warm = false;
		bad_temp = false;
		break;
	}
	if(bad_temp_flag ^ bad_temp)
	{

		bad_temp_flag= bad_temp;
		pr_info("bad_temp_flag is %d,qpnp_chg_charge_en is %d\n",bad_temp_flag,!bad_temp_flag);
		rc = qpnp_lbc_charger_enable(chip, THERMAL, !bad_temp_flag);
		if (rc){
			pr_err("Failed to disable/enable charging rc=%d\n", rc);
		}
	}

	if (chip->bat_is_cool ^ bat_cool || chip->bat_is_warm ^ bat_warm)
	{
		chip->bat_is_cool = bat_cool;
		chip->bat_is_warm = bat_warm;

		/**
		 * set appropriate voltages and currents.
		 *
		 * Note that when the battery is hot or cold, the charger
		 * driver will not resume with SoC. Only vbatdet is used to
		 * determine resume of charging.
		 */
		qpnp_lbc_set_appropriate_vddmax(chip);
		qpnp_lbc_set_appropriate_current(chip);
//		qpnp_chg_set_appropriate_vbatdet(chip);
	}

	pr_debug("warm %d, cool %d, low = %d deciDegC, high = %d deciDegC ,hot = %d ,warm = %d , cool = %d , cold = %d \n",
			chip->bat_is_warm, chip->bat_is_cool,chip->adc_param.low_temp, chip->adc_param.high_temp,
			chip->cfg_hot_bat_decidegc,chip->cfg_warm_bat_decidegc,chip->cfg_cool_bat_decidegc,chip->cfg_cold_bat_decidegc);

	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}
static void qpnp_lbc_jeita_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct qpnp_lbc_chip *chip = ctx;
	int temp;
	enum hw_high_low_temp_configure_type temp_zone_type = UNKNOW_ZONE;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invalid notification %d\n", state);
		return;
	}

	temp = get_prop_batt_temp(chip);

	pr_debug("temp = %d state = %s\n", temp,
			state == ADC_TM_WARM_STATE ? "warm" : "cool");
	if(state == ADC_TM_WARM_STATE)
	{
		temp_zone_type = hw_tm_warm_notification_zone(temp, chip);
	}
	else
	{
		temp_zone_type = hw_tm_cool_notification_zone(temp, chip);
	}
	if(DISABLE_SOFTWARE_HARDWARE_TEMP_CONTROL == charge_no_limit
		|| DISABLE_SOFTWARE_TEMP_CONTROL == charge_no_limit)
	{
		pr_info("error,software temp control has been canceled\n");
		temp_zone_type = COOL_WARM_ZONE;
	}
	hw_tm_set_configure(temp_zone_type, chip);
}
#else
static void qpnp_lbc_jeita_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct qpnp_lbc_chip *chip = ctx;
	bool bat_warm = 0, bat_cool = 0;
	int temp;
	unsigned long flags;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invalid notification %d\n", state);
		return;
	}

	temp = get_prop_batt_temp(chip);

	pr_debug("temp = %d state = %s\n", temp,
			state == ADC_TM_WARM_STATE ? "warm" : "cool");

	if (state == ADC_TM_WARM_STATE) {
		if (temp >= chip->cfg_warm_bat_decidegc) {
			/* Normal to warm */
			bat_warm = true;
			bat_cool = false;
			chip->adc_param.low_temp =
					chip->cfg_warm_bat_decidegc
					- HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >=
			chip->cfg_cool_bat_decidegc + HYSTERISIS_DECIDEGC) {
			/* Cool to normal */
			bat_warm = false;
			bat_cool = false;

			chip->adc_param.low_temp =
					chip->cfg_cool_bat_decidegc;
			chip->adc_param.high_temp =
					chip->cfg_warm_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp <= chip->cfg_cool_bat_decidegc) {
			/* Normal to cool */
			bat_warm = false;
			bat_cool = true;
			chip->adc_param.high_temp =
					chip->cfg_cool_bat_decidegc
					+ HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_WARM_THR_ENABLE;
		} else if (temp <= (chip->cfg_warm_bat_decidegc -
					HYSTERISIS_DECIDEGC)){
			/* Warm to normal */
			bat_warm = false;
			bat_cool = false;

			chip->adc_param.low_temp =
					chip->cfg_cool_bat_decidegc;
			chip->adc_param.high_temp =
					chip->cfg_warm_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (chip->bat_is_cool ^ bat_cool || chip->bat_is_warm ^ bat_warm) {
		spin_lock_irqsave(&chip->ibat_change_lock, flags);
		chip->bat_is_cool = bat_cool;
		chip->bat_is_warm = bat_warm;
		qpnp_lbc_set_appropriate_vddmax(chip);
		qpnp_lbc_set_appropriate_current(chip);
		spin_unlock_irqrestore(&chip->ibat_change_lock, flags);
	}

	pr_debug("warm %d, cool %d, low = %d deciDegC, high = %d deciDegC\n",
			chip->bat_is_warm, chip->bat_is_cool,
			chip->adc_param.low_temp, chip->adc_param.high_temp);

	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}
#endif

#define IBAT_TERM_EN_MASK		BIT(3)
static int qpnp_lbc_chg_init(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	qpnp_lbc_vbatweak_set(chip, chip->cfg_batt_weak_voltage_uv);
	rc = qpnp_lbc_vinmin_set(chip, chip->cfg_min_voltage_mv);
	if (rc) {
		pr_err("Failed  to set  vin_min rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_vddsafe_set(chip, chip->cfg_safe_voltage_mv);
	if (rc) {
		pr_err("Failed to set vdd_safe rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_vddmax_set(chip, chip->cfg_max_voltage_mv);
	if (rc) {
		pr_err("Failed to set vdd_safe rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_ibatsafe_set(chip, chip->cfg_safe_current);
	if (rc) {
		pr_err("Failed to set ibat_safe rc=%d\n", rc);
		return rc;
	}

	if (of_property_read_bool(chip->spmi->dev.of_node, "qcom,tchg-mins")) {
		rc = qpnp_lbc_tchg_max_set(chip, chip->cfg_tchg_mins);
		if (rc) {
			pr_err("Failed to set tchg_mins rc=%d\n", rc);
			return rc;
		}
	}

	/*
	 * Override VBAT_DET comparator to enable charging
	 * irrespective of VBAT above VBAT_DET.
	 */
	rc = qpnp_lbc_vbatdet_override(chip, OVERRIDE_0);
	if (rc) {
		pr_err("Failed to override comp rc=%d\n", rc);
		return rc;
	}

	/*
	 * Disable iterm comparator of linear charger to disable charger
	 * detecting end of charge condition based on DT configuration
	 * and float charge configuration.
	 */
	if (!chip->cfg_charger_detect_eoc || chip->cfg_float_charge) {
		rc = qpnp_lbc_masked_write(chip,
				chip->chgr_base + CHG_IBATTERM_EN_REG,
				IBAT_TERM_EN_MASK, 0);
		if (rc) {
			pr_err("Failed to disable EOC comp rc=%d\n", rc);
			return rc;
		}
	}

	/* Disable charger watchdog */
	reg_val = 0;
	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_WDOG_EN_REG,
				&reg_val, 1);

	return rc;
}

static int qpnp_lbc_bat_if_init(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	/* Select battery presence detection */
	switch (chip->cfg_bpd_detection) {
	case BPD_TYPE_BAT_THM:
		reg_val = BATT_THM_EN;
		break;
	case BPD_TYPE_BAT_ID:
		reg_val = BATT_ID_EN;
		break;
	case BPD_TYPE_BAT_THM_BAT_ID:
		reg_val = BATT_THM_EN | BATT_ID_EN;
		break;
	default:
		reg_val = BATT_THM_EN;
		break;
	}

	rc = qpnp_lbc_masked_write(chip,
			chip->bat_if_base + BAT_IF_BPD_CTRL_REG,
			BATT_BPD_CTRL_SEL_MASK, reg_val);
	if (rc) {
		pr_err("Failed to choose BPD rc=%d\n", rc);
		return rc;
	}

	/* Force on VREF_BAT_THM */
	reg_val = VREF_BATT_THERM_FORCE_ON;
	rc = qpnp_lbc_write(chip,
			chip->bat_if_base + BAT_IF_VREF_BAT_THM_CTRL_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to force on VREF_BAT_THM rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int qpnp_lbc_usb_path_init(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	if (qpnp_lbc_is_usb_chg_plugged_in(chip)) {
		reg_val = 0;
		rc = qpnp_lbc_write(chip,
			chip->usb_chgpth_base + CHG_USB_ENUM_T_STOP_REG,
			&reg_val, 1);
		if (rc) {
			pr_err("Failed to write enum stop rc=%d\n", rc);
			return -ENXIO;
		}
	}

	if (chip->cfg_charging_disabled) {
		rc = qpnp_lbc_charger_enable(chip, USER, 0);
		if (rc)
			pr_err("Failed to disable charging rc=%d\n", rc);
	} else {
		/*
		 * Enable charging explictly,
		 * because not sure the default behavior.
		 */
#ifdef CONFIG_HUAWEI_KERNEL
		if (!chip->use_other_charger) {
			reg_val = CHG_ENABLE;
			rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
						CHG_EN_MASK, reg_val);
			if (rc)
				pr_err("Failed to enable charger rc=%d\n", rc);
		}
#else
		reg_val = CHG_ENABLE;
		rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
					CHG_EN_MASK, reg_val);
		if (rc)
			pr_err("Failed to enable charger rc=%d\n", rc);
#endif
	}

	return rc;
}

#define LBC_MISC_DIG_VERSION_1			0x01
static int qpnp_lbc_misc_init(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val, reg_val1, trim_center;

	/* Check if this LBC MISC version supports VDD trimming */
	rc = qpnp_lbc_read(chip, chip->misc_base + MISC_REV2_REG,
			&reg_val, 1);
	if (rc) {
		pr_err("Failed to read VDD_EA TRIM3 reg rc=%d\n", rc);
		return rc;
	}

	if (reg_val >= LBC_MISC_DIG_VERSION_1) {
		chip->supported_feature_flag |= VDD_TRIM_SUPPORTED;
		/* Read initial VDD trim value */
		rc = qpnp_lbc_read(chip, chip->misc_base + MISC_TRIM3_REG,
				&reg_val, 1);
		if (rc) {
			pr_err("Failed to read VDD_EA TRIM3 reg rc=%d\n", rc);
			return rc;
		}

		rc = qpnp_lbc_read(chip, chip->misc_base + MISC_TRIM4_REG,
				&reg_val1, 1);
		if (rc) {
			pr_err("Failed to read VDD_EA TRIM3 reg rc=%d\n", rc);
			return rc;
		}

		trim_center = ((reg_val & MISC_TRIM3_VDD_MASK)
					>> VDD_TRIM3_SHIFT)
					| ((reg_val1 & MISC_TRIM4_VDD_MASK)
					>> VDD_TRIM4_SHIFT);
		chip->init_trim_uv = qpnp_lbc_get_trim_voltage(trim_center);
		chip->delta_vddmax_uv = chip->init_trim_uv;
		pr_debug("Initial trim center %x trim_uv %d\n",
				trim_center, chip->init_trim_uv);
	}

	pr_debug("Setting BOOT_DONE\n");
	reg_val = MISC_BOOT_DONE;
	rc = qpnp_lbc_write(chip, chip->misc_base + MISC_BOOT_DONE_REG,
				&reg_val, 1);

	return rc;
}

#define OF_PROP_READ(chip, prop, qpnp_dt_property, retval, optional)	\
do {									\
	if (retval)							\
		break;							\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"qcom," qpnp_dt_property,	\
					&chip->prop);			\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		pr_err("Error reading " #qpnp_dt_property		\
				" property rc = %d\n", rc);		\
} while (0)

static int qpnp_charger_read_dt_props(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	const char *bpd;

	OF_PROP_READ(chip, cfg_max_voltage_mv, "vddmax-mv", rc, 0);
	OF_PROP_READ(chip, cfg_safe_voltage_mv, "vddsafe-mv", rc, 0);
	OF_PROP_READ(chip, cfg_min_voltage_mv, "vinmin-mv", rc, 0);
	OF_PROP_READ(chip, cfg_safe_current, "ibatsafe-ma", rc, 0);
	if (rc)
		pr_err("Error reading required property rc=%d\n", rc);

	OF_PROP_READ(chip, cfg_tchg_mins, "tchg-mins", rc, 1);
	OF_PROP_READ(chip, cfg_warm_bat_decidegc, "warm-bat-decidegc", rc, 1);
	OF_PROP_READ(chip, cfg_cool_bat_decidegc, "cool-bat-decidegc", rc, 1);
	OF_PROP_READ(chip, cfg_hot_batt_p, "batt-hot-percentage", rc, 1);
	OF_PROP_READ(chip, cfg_cold_batt_p, "batt-cold-percentage", rc, 1);
	OF_PROP_READ(chip, cfg_batt_weak_voltage_uv, "vbatweak-uv", rc, 1);
	OF_PROP_READ(chip, cfg_float_charge, "float-charge", rc, 1);
#ifdef CONFIG_HUAWEI_KERNEL
	OF_PROP_READ(chip, cfg_cold_bat_decidegc, "cold-bat-decidegc", rc, 1);
	OF_PROP_READ(chip, cfg_hot_bat_decidegc, "hot-bat-decidegc", rc, 1);
#endif

	if (rc) {
		pr_err("Error reading optional property rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_string(chip->spmi->dev.of_node,
						"qcom,bpd-detection", &bpd);
	if (rc) {

		chip->cfg_bpd_detection = BPD_TYPE_BAT_THM;
		rc = 0;
	} else {
		chip->cfg_bpd_detection = get_bpd(bpd);
		if (chip->cfg_bpd_detection < 0) {
			pr_err("Failed to determine bpd schema rc=%d\n", rc);
			return -EINVAL;
		}
	}

	/*
	 * Look up JEITA compliance parameters if cool and warm temp
	 * provided
	 */
	if (chip->cfg_cool_bat_decidegc || chip->cfg_warm_bat_decidegc) {
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("Failed to get adc-tm rc=%d\n", rc);
			return rc;
		}

		OF_PROP_READ(chip, cfg_warm_bat_chg_ma, "ibatmax-warm-ma",
				rc, 1);
		OF_PROP_READ(chip, cfg_cool_bat_chg_ma, "ibatmax-cool-ma",
				rc, 1);
		OF_PROP_READ(chip, cfg_warm_bat_mv, "warm-bat-mv", rc, 1);
		OF_PROP_READ(chip, cfg_cool_bat_mv, "cool-bat-mv", rc, 1);
		if (rc) {
			pr_err("Error reading battery temp prop rc=%d\n", rc);
			return rc;
		}
	}

#ifdef CONFIG_HUAWEI_KERNEL
	chip->use_other_charger = of_property_read_bool(
			chip->spmi->dev.of_node,"qcom,use-other-charger");
    chip->use_only_ti_charger = of_property_read_bool(
         chip->spmi->dev.of_node,"qcom,use-only-ti-charger");
#endif
#ifdef CONFIG_HUAWEI_DSM
	use_other_charger = chip->use_other_charger;
#endif
	/* Get the btc-disabled property */
	chip->cfg_btc_disabled = of_property_read_bool(
			chip->spmi->dev.of_node, "qcom,btc-disabled");

	/* Get the charging-disabled property */
	chip->cfg_charging_disabled =
		of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,charging-disabled");

	/* Get the fake-batt-values property */
	chip->cfg_use_fake_battery =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,use-default-batt-values");

	/* Get peripheral reset configuration property */
	chip->cfg_disable_follow_on_reset =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,disable-follow-on-reset");

	/* Get the float charging property */
	chip->cfg_float_charge =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,float-charge");

	/* Get the charger EOC detect property */
	chip->cfg_charger_detect_eoc =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,charger-detect-eoc");

	/* Get the vbatdet disable property */
	chip->cfg_disable_vbatdet_based_recharge =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,disable-vbatdet-based-recharge");
	/* Disable charging when faking battery values */
	if (chip->cfg_use_fake_battery)
		chip->cfg_charging_disabled = true;

	chip->cfg_use_external_charger = of_property_read_bool(
			chip->spmi->dev.of_node, "qcom,use-external-charger");

	if (of_find_property(chip->spmi->dev.of_node,
					"qcom,thermal-mitigation",
					&chip->cfg_thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->cfg_thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->cfg_thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(chip->spmi->dev.of_node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation,
				chip->cfg_thermal_levels);
		if (rc) {
			pr_err("Failed to read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	return rc;
}

static irqreturn_t qpnp_lbc_usbin_valid_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int usb_present;
	unsigned long flags;

	usb_present = qpnp_lbc_is_usb_chg_plugged_in(chip);
	pr_info("usbin-valid triggered: %d\n", usb_present);
#ifdef CONFIG_HUAWEI_KERNEL
	do_max77819_dcin_valid_irq(usb_present);
#endif
#ifdef CONFIG_HUAWEI_DSM
	schedule_work(&chip->usbin_valid_count_work);
#endif
	if (chip->usb_present ^ usb_present) {
		chip->usb_present = usb_present;
		if (!usb_present) {
#ifdef CONFIG_HUAWEI_KERNEL
        /* wake_lock for update the state of led,prevent phone enter sleep at once  */
        wake_lock_timeout(&chip->led_wake_lock,msecs_to_jiffies(LED_CHECK_PERIOD_MS));
#endif
			/* remove the dsm code of cancel_delayed_work */
			qpnp_lbc_charger_enable(chip, CURRENT, 0);
			spin_lock_irqsave(&chip->ibat_change_lock, flags);
			chip->usb_psy_ma = QPNP_CHG_I_MAX_MIN_90;
			qpnp_lbc_set_appropriate_current(chip);
			wake_unlock(&chip->chg_wake_lock);
			spin_unlock_irqrestore(&chip->ibat_change_lock,
								flags);
		} else {
			/*
			 * Override VBAT_DET comparator to start charging
			 * even if VBAT > VBAT_DET.
			 */
			if (!chip->cfg_disable_vbatdet_based_recharge)
				qpnp_lbc_vbatdet_override(chip, OVERRIDE_0);

			/*
			 * Enable SOC based charging to make sure
			 * charging gets enabled on USB insertion
			 * irrespective of battery SOC above resume_soc.
			 */
			qpnp_lbc_charger_enable(chip, SOC, 1);
			wake_lock(&chip->chg_wake_lock);
			/* remove the dsm code of schedule_delayed_work */
		}

		pr_debug("Updating usb_psy PRESENT property\n");
		power_supply_set_present(chip->usb_psy, chip->usb_present);
#ifdef CONFIG_HUAWEI_KERNEL
		if (chip->bat_if_base) {
			pr_debug("power supply changed batt_psy\n");
			power_supply_changed(&chip->batt_psy);
		}
#endif
	}
#ifdef CONFIG_HUAWEI_DSM
	/* if usb online status is abnormal*/
	/*dump pmic registers and adc values, and notify to dsm server */
	else{
		chip->error_type = DSM_CHARGER_ONLINE_ABNORMAL_ERROR_NO;
		schedule_work(&chip->dump_work);
	}
#endif
	return IRQ_HANDLED;
}

static int qpnp_lbc_is_batt_temp_ok(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + INT_RT_STS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->bat_if_base + INT_RT_STS_REG, rc);
		return rc;
	}

	return (reg_val & BAT_TEMP_OK_IRQ) ? 1 : 0;
}

static irqreturn_t qpnp_lbc_batt_temp_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int batt_temp_good;

	batt_temp_good = qpnp_lbc_is_batt_temp_ok(chip);
	pr_debug("batt-temp triggered: %d\n", batt_temp_good);

	pr_debug("power supply changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t qpnp_lbc_batt_pres_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int batt_present;

	batt_present = qpnp_lbc_is_batt_present(chip);
	pr_debug("batt-pres triggered: %d\n", batt_present);

	if (chip->batt_present ^ batt_present) {
		chip->batt_present = batt_present;
		pr_debug("power supply changed batt_psy\n");
		power_supply_changed(&chip->batt_psy);

		if ((chip->cfg_cool_bat_decidegc
					|| chip->cfg_warm_bat_decidegc)
					&& batt_present) {
			pr_debug("enabling vadc notifications\n");
			if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
						&chip->adc_param))
				pr_err("request ADC error\n");
		} else if ((chip->cfg_cool_bat_decidegc
					|| chip->cfg_warm_bat_decidegc)
					&& !batt_present) {
			qpnp_adc_tm_disable_chan_meas(chip->adc_tm_dev,
					&chip->adc_param);
			/* if battery present irq is triggered, and is absent*/
			/*dump pmic registers and adc values, and notify to dsm server */
#ifdef CONFIG_HUAWEI_DSM
			chip->error_type = DSM_BATT_PRES_ERROR_NO;
			schedule_work(&chip->dump_work);
#endif
			pr_debug("disabling vadc notifications\n");
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t qpnp_lbc_chg_failed_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int rc;
	u8 reg_val = CHG_FAILED_BIT;

	pr_info("chg_failed triggered count=%u\n", ++chip->chg_failed_count);
#ifdef CONFIG_HUAWEI_DSM
	chip->error_type = DSM_LINEAR_CHG_FAILED;
	schedule_work(&chip->dump_work);
#endif

	rc = qpnp_lbc_write(chip, chip->chgr_base + CHG_FAILED_REG,
				&reg_val, 1);
	if (rc)
		pr_err("Failed to write chg_fail clear bit rc=%d\n", rc);

	if (chip->bat_if_base) {
		pr_debug("power supply changed batt_psy\n");
		power_supply_changed(&chip->batt_psy);
	}

	return IRQ_HANDLED;
}

static int qpnp_lbc_is_fastchg_on(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->chgr_base + INT_RT_STS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt status rc=%d\n", rc);
		return rc;
	}
	pr_debug("charger status %x\n", reg_val);
	return (reg_val & FAST_CHG_ON_IRQ) ? 1 : 0;
}

#define TRIM_PERIOD_NS			(50LL * NSEC_PER_SEC)
static irqreturn_t qpnp_lbc_fastchg_irq_handler(int irq, void *_chip)
{
	ktime_t kt;
	struct qpnp_lbc_chip *chip = _chip;
	bool fastchg_on = false;

	fastchg_on = qpnp_lbc_is_fastchg_on(chip);

	pr_info("FAST_CHG IRQ triggered, fastchg_on: %d\n", fastchg_on);
#ifdef CONFIG_HUAWEI_KERNEL
	if (chip->resuming_charging) {
		chip->resuming_charging = false;
	}
#endif
	if (chip->fastchg_on ^ fastchg_on) {
		chip->fastchg_on = fastchg_on;
		if (fastchg_on) {
			mutex_lock(&chip->chg_enable_lock);
			chip->chg_done = false;
			mutex_unlock(&chip->chg_enable_lock);
			/*
			 * Start alarm timer to periodically calculate
			 * and update VDD_MAX trim value.
			 */
			if (chip->supported_feature_flag &
						VDD_TRIM_SUPPORTED) {
				kt = ns_to_ktime(TRIM_PERIOD_NS);
				alarm_start_relative(&chip->vddtrim_alarm,
							kt);
			}
		}

		if (chip->bat_if_base) {
			pr_debug("power supply changed batt_psy\n");
			power_supply_changed(&chip->batt_psy);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t qpnp_lbc_chg_done_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;

	pr_debug("charging done triggered\n");

	chip->chg_done = true;
	pr_debug("power supply changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);

	return IRQ_HANDLED;
}

static irqreturn_t qpnp_lbc_vbatdet_lo_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int rc;

	pr_info("vbatdet-lo triggered\n");

#ifdef CONFIG_HUAWEI_KERNEL
    if(chip->cfg_charging_disabled == true)
    {
        pr_info("cfg_charging_disabled = %d\n",chip->cfg_charging_disabled);
        return IRQ_HANDLED;
    }
#endif
	/*
	 * Disable vbatdet irq to prevent interrupt storm when VBAT is
	 * close to VBAT_DET.
	 */
	qpnp_lbc_disable_irq(chip, &chip->irqs[CHG_VBAT_DET_LO]);

	/*
	 * Override VBAT_DET comparator to 0 to fix comparator toggling
	 * near VBAT_DET threshold.
	 */
	qpnp_lbc_vbatdet_override(chip, OVERRIDE_0);

	/*
	 * Battery has fallen below the vbatdet threshold and it is
	 * time to resume charging.
	 */
	rc = qpnp_lbc_charger_enable(chip, SOC, 1);
	if (rc)
		pr_err("Failed to enable charging\n");

	return IRQ_HANDLED;
}

static int qpnp_lbc_is_overtemp(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + INT_RT_STS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt status rc=%d\n", rc);
		return rc;
	}

	pr_debug("OVERTEMP rt status %x\n", reg_val);
	return (reg_val & OVERTEMP_ON_IRQ) ? 1 : 0;
}

static irqreturn_t qpnp_lbc_usb_overtemp_irq_handler(int irq, void *_chip)
{
	struct qpnp_lbc_chip *chip = _chip;
	int overtemp = qpnp_lbc_is_overtemp(chip);

	pr_warn_ratelimited("charger %s temperature limit !!!\n",
					overtemp ? "exceeds" : "within");
#ifdef CONFIG_HUAWEI_DSM
	if(overtemp){
		chip->error_type = DSM_LINEAR_USB_OVERTEMP;
		schedule_work(&chip->dump_work);
	}
#endif
	return IRQ_HANDLED;
}

static int qpnp_disable_lbc_charger(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg;

	reg = CHG_FORCE_BATT_ON;
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
							CHG_EN_MASK, reg);
	/* disable BTC */
	rc |= qpnp_lbc_masked_write(chip, chip->bat_if_base + BAT_IF_BTC_CTRL,
							BTC_COMP_EN_MASK, 0);
	/* Enable BID and disable THM based BPD */
	reg = BATT_ID_EN | BATT_BPD_OFFMODE_EN;
	rc |= qpnp_lbc_write(chip, chip->bat_if_base + BAT_IF_BPD_CTRL_REG,
								&reg, 1);
	return rc;
}

#define SPMI_REQUEST_IRQ(chip, idx, rc, irq_name, threaded, flags, wake)\
do {									\
	if (rc)								\
		break;							\
	if (chip->irqs[idx].irq) {					\
		if (threaded)						\
			rc = devm_request_threaded_irq(chip->dev,	\
				chip->irqs[idx].irq, NULL,		\
				qpnp_lbc_##irq_name##_irq_handler,	\
				flags, #irq_name, chip);		\
		else							\
			rc = devm_request_irq(chip->dev,		\
				chip->irqs[idx].irq,			\
				qpnp_lbc_##irq_name##_irq_handler,	\
				flags, #irq_name, chip);		\
		if (rc < 0) {						\
			pr_err("Unable to request " #irq_name " %d\n",	\
								rc);	\
		} else {						\
			rc = 0;						\
			if (wake) {					\
				enable_irq_wake(chip->irqs[idx].irq);	\
				chip->irqs[idx].is_wake = true;		\
			}						\
		}							\
	}								\
} while (0)

#define SPMI_GET_IRQ_RESOURCE(chip, rc, resource, idx, name)		\
do {									\
	if (rc)								\
		break;							\
									\
	rc = spmi_get_irq_byname(chip->spmi, resource, #name);		\
	if (rc < 0) {							\
		pr_err("Unable to get irq resource " #name "%d\n", rc);	\
	} else {							\
		chip->irqs[idx].irq = rc;				\
		rc = 0;							\
	}								\
} while (0)

static int qpnp_lbc_request_irqs(struct qpnp_lbc_chip *chip)
{
	int rc = 0;

	SPMI_REQUEST_IRQ(chip, CHG_FAILED, rc, chg_failed, 0,
			IRQF_TRIGGER_RISING, 1);

	SPMI_REQUEST_IRQ(chip, CHG_FAST_CHG, rc, fastchg, 1,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_ONESHOT, 1);

	SPMI_REQUEST_IRQ(chip, CHG_DONE, rc, chg_done, 0,
			IRQF_TRIGGER_RISING, 0);

	SPMI_REQUEST_IRQ(chip, CHG_VBAT_DET_LO, rc, vbatdet_lo, 0,
			IRQF_TRIGGER_FALLING, 1);

	SPMI_REQUEST_IRQ(chip, BATT_PRES, rc, batt_pres, 1,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_ONESHOT, 1);

	SPMI_REQUEST_IRQ(chip, BATT_TEMPOK, rc, batt_temp, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 1);

	SPMI_REQUEST_IRQ(chip, USBIN_VALID, rc, usbin_valid, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 1);

	SPMI_REQUEST_IRQ(chip, USB_OVER_TEMP, rc, usb_overtemp, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 0);

	return 0;
}

static int qpnp_lbc_get_irqs(struct qpnp_lbc_chip *chip, u8 subtype,
					struct spmi_resource *spmi_resource)
{
	int rc = 0;

	switch (subtype) {
	case LBC_CHGR_SUBTYPE:
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_FAST_CHG, fast-chg-on);
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_FAILED, chg-failed);
		if (!chip->cfg_disable_vbatdet_based_recharge)
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_VBAT_DET_LO, vbat-det-lo);
		if (chip->cfg_charger_detect_eoc)
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_DONE, chg-done);
		break;
	case LBC_BAT_IF_SUBTYPE:
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						BATT_PRES, batt-pres);
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						BATT_TEMPOK, bat-temp-ok);
		break;
	case LBC_USB_PTH_SUBTYPE:
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						USBIN_VALID, usbin-valid);
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						USB_OVER_TEMP, usb-over-temp);
		break;
	};

	return 0;
}

/* Get/Set initial state of charger */
static void determine_initial_status(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	chip->usb_present = qpnp_lbc_is_usb_chg_plugged_in(chip);
	power_supply_set_present(chip->usb_psy, chip->usb_present);
	/*
	 * Set USB psy online to avoid userspace from shutting down if battery
	 * capacity is at zero and no chargers online.
	 */
	if (chip->usb_present)
		power_supply_set_online(chip->usb_psy, 1);

	/*
	 * Configure peripheral reset control
	 * This is a workaround only for SLT testing.
	 */
	if (chip->cfg_disable_follow_on_reset) {
		reg_val = 0x0;
		rc = __qpnp_lbc_secure_write(chip->spmi, chip->chgr_base,
				CHG_PERPH_RESET_CTRL3_REG, &reg_val, 1);
		if (rc)
			pr_err("Failed to configure PERPH_CTRL3 rc=%d\n", rc);
		else
			pr_warn("Charger is not following PMIC reset\n");
	}
}

#ifdef CONFIG_HUAWEI_KERNEL
static int qpnp_chg_load_battery_data(struct qpnp_lbc_chip *chip)
{
	struct bms_battery_data batt_data;
	struct device_node *node;
	struct qpnp_vadc_result result;
	int rc;

    /*for maxim_charger return*/
	if (chip->maxim_charger)
	{
		pr_info("Using the other charger ic, do not load battery data!\n");
		return 0;
	}
	node = of_find_node_by_name(chip->spmi->dev.of_node,
			"qcom,battery-data");
	if (node)
	{
		memset(&batt_data, 0, sizeof(struct bms_battery_data));
		rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
		if (rc)
		{
			pr_err("error reading batt id channel = %d, rc = %d\n",
						LR_MUX2_BAT_ID, rc);
			return rc;
		}

		batt_data.max_voltage_uv = -1;
		batt_data.iterm_ua = -1;
		rc = of_batterydata_read_data(node,
			&batt_data, result.physical);
		if (rc)
		{
			pr_err("failed to read battery data: %d\n", rc);
#ifdef CONFIG_HUAWEI_DSM
			dump_registers_and_adc(charger_dclient, chip, DSM_BATTERY_ID_UNKNOW);
#endif
			batt_data = palladium_1500_data;
		}

		if ((chip->cfg_cool_bat_decidegc || chip->cfg_warm_bat_decidegc) &&
			(batt_data.warm_bat_decidegc || batt_data.cool_bat_decidegc))
		{
			chip->cfg_warm_bat_decidegc = batt_data.warm_bat_decidegc;
			chip->cfg_warm_bat_chg_ma = batt_data.warm_bat_chg_ma;
			chip->cfg_warm_bat_mv = batt_data.warm_bat_mv;

			chip->cfg_cool_bat_decidegc = batt_data.cool_bat_decidegc;
			chip->cfg_cool_bat_chg_ma = batt_data.cool_bat_chg_ma;
			chip->cfg_cool_bat_mv = batt_data.cool_bat_mv;

			chip->cfg_hot_bat_decidegc = batt_data.hot_bat_decidegc;
			chip->cfg_cold_bat_decidegc = batt_data.cold_bat_decidegc;

			pr_info("use special temp-cv parameters\n");
		}
#ifdef CONFIG_HUAWEI_KERNEL		
		if(chip->use_other_charger)
		{
			jeita_batt_param.cold.i_max = 0;
			jeita_batt_param.cold.v_max = 0;
			jeita_batt_param.cold.t_high = chip->cfg_cold_bat_decidegc;
			jeita_batt_param.cold.t_low = INT_MIN;
			jeita_batt_param.cold.selected = 0;
			jeita_batt_param.cold.last_zone = NULL;

			jeita_batt_param.cool.i_max = chip->cfg_cool_bat_chg_ma;
			jeita_batt_param.cool.v_max = chip->cfg_cool_bat_mv;
			jeita_batt_param.cool.t_high = chip->cfg_cool_bat_decidegc;
			jeita_batt_param.cool.t_low = chip->cfg_cold_bat_decidegc;
			jeita_batt_param.cool.selected = 0;
			jeita_batt_param.cool.last_zone = NULL;

			jeita_batt_param.normal.i_max = 1500;
			jeita_batt_param.normal.v_max = chip->cfg_cool_bat_mv;
			jeita_batt_param.normal.t_high = chip->cfg_warm_bat_decidegc;
			jeita_batt_param.normal.t_low = chip->cfg_cool_bat_decidegc;
			jeita_batt_param.normal.selected = 0;
			jeita_batt_param.normal.last_zone = NULL;

			jeita_batt_param.warm.i_max = chip->cfg_warm_bat_chg_ma;
			jeita_batt_param.warm.v_max = chip->cfg_warm_bat_mv;
			jeita_batt_param.warm.t_high = chip->cfg_hot_bat_decidegc;
			jeita_batt_param.warm.t_low = chip->cfg_warm_bat_decidegc;
			jeita_batt_param.warm.selected = 0;
			jeita_batt_param.warm.last_zone = NULL;

			jeita_batt_param.hot.i_max = 0;
			jeita_batt_param.hot.v_max = 0;
			jeita_batt_param.hot.t_high = INT_MAX;
			jeita_batt_param.hot.t_low = chip->cfg_hot_bat_decidegc;
			jeita_batt_param.hot.selected = 0;
			jeita_batt_param.hot.last_zone = NULL;
		}
#endif

		pr_info("warm_bat_decidegc=%d "
				"warm_bat_chg_ma=%d "
				"warm_bat_mv=%d "
				"cool_bat_decidegc=%d "
				"cool_bat_chg_ma=%d "
				"cool_bat_mv=%d "
				"hot_bat_decidegc=%d "
				"cold_bat_decidegc=%d \n",
				chip->cfg_warm_bat_decidegc,
				chip->cfg_warm_bat_chg_ma,
				chip->cfg_warm_bat_mv,
				chip->cfg_cool_bat_decidegc,
				chip->cfg_cool_bat_chg_ma,
				chip->cfg_cool_bat_mv,
				chip->cfg_hot_bat_decidegc,
				chip->cfg_cold_bat_decidegc);

		if (batt_data.max_voltage_uv >= 0)
		{
			chip->cfg_max_voltage_mv = batt_data.max_voltage_uv / 1000;
		}
		if (batt_data.iterm_ua >= 0)
		{
			chip->cfg_term_current = batt_data.iterm_ua / 1000;
		}

	}

	return 0;
}
#endif

#ifdef CONFIG_HUAWEI_DSM
static void print_basic_info_before_dump(struct dsm_client *dclient, const int type)
{
	int error_type;

	error_type = type;
	switch(error_type)
	{
		case DSM_BMS_NORMAL_SOC_CHANGE_MUCH:
			dsm_client_record(dclient,
				"soc changed a lot in one minute "
				"the difference is more than 5 percent\n");
			pr_info("soc changed a lot in one minute "
				"the difference is more than 5 percent\n");
			break;

		case DSM_BMS_VOL_SOC_DISMATCH_1:
			dsm_client_record(dclient,
				"battery voltage is over 3.7V, but soc is "
				"below 2 percent, not match\n");
			pr_info("battery voltage is over 3.7V, but soc is "
				"below 2 percent, not match\n");
			break;

		case DSM_BMS_VOL_SOC_DISMATCH_2:
			dsm_client_record(dclient,
				"battery voltage is over 3.6V, but soc is "
				"0 percent, not match\n");
			pr_info("battery voltage is over 3.6V, but soc is "
				"0 percent, not match\n");
			break;

		case DSM_BMS_VOL_SOC_DISMATCH_3:
			dsm_client_record(dclient,
				"battery voltage is over 4.32V, but  "
				"soc is below 95 percent\n");
			pr_info("battery voltage is over 4.32V, but  "
				"soc is below 95 percent\n");
			break;

		case DSM_VM_BMS_VOL_SOC_DISMATCH_4:
			dsm_client_record(dclient,
				"battery voltage is too low when "
				"soc is 2 percent\n");
			pr_info("battery voltage is too low when "
				"soc is 2 percent\n");
			break;

		case DSM_MAXIM_HANDLE_MODEL_FAIL:
			dsm_client_record(dclient,
				"maxim handle model fail\n");
			pr_info("maxim handle model fail\n");
			break;

		case DSM_BMS_SOC_CHANGE_PLUG_INOUT:
			dsm_client_record(dclient,
				"soc changed more than 1 percent when plug in or out charger\n");
			pr_info("soc changed more than 1 percent when plug in or out charger\n");
			break;

		case DSM_BMS_POWON_SOC_CHANGE_MUCH:
			dsm_client_record(dclient,
				"poweron soc is different with last "
				"shutdown soc, the difference is more than 10 percent\n");
			pr_info("poweron soc is different with last "
				"shutdown soc, the difference is more than 10 percent\n");
			break;

		case DSM_BMS_NORMAL_SOC_ERROR_NO:
			dsm_client_record(dclient,
				"soc changed more than 5 percent during recent one minute\n");
			pr_info("soc changed more than 5 percent during recent one minute\n");
			break;

		case DSM_NOT_CHARGE_WHEN_ALLOWED:
			dsm_client_record(dclient,
				"cannot charging when allowed charging\n");
			pr_info("cannot charging when allowed charging\n");
			break;

		case DSM_CHG_OVP_ERROR_NO:
			dsm_client_record(dclient,
				"CHG_OVP happend!\n");
			pr_info("CHG_OVP happend!\n");
			break;

		case DSM_BATT_PRES_ERROR_NO:
			dsm_client_record(dclient,
				"battery is absent!\n");
			pr_info("battery is absent!\n");
			break;

		case DSM_WARM_CURRENT_LIMIT_FAIL:
			dsm_client_record(dclient,
				"set battery warm charge current failed\n");
			pr_info("set battery warm charge current failed\n");
			break;

		case DSM_COOL_CURRENT_LIMIT_FAIL:
			dsm_client_record(dclient,
				"set battery cool charge current failed\n");
			pr_info("set battery cool charge current failed\n");
			break;

		case DSM_FULL_WHEN_CHARGER_ABSENT:
			dsm_client_record(dclient,
				"battery status is full when charger is absent\n");
			pr_info("battery status is full when charger is absent\n");
			break;

		case DSM_CHARGER_ONLINE_ABNORMAL_ERROR_NO:
			dsm_client_record(dclient,
				"charger online status abnormal!\n");
			pr_info("charger online status abnormal!\n");
			break;

		case DSM_BATT_VOL_OVER_4400:
			dsm_client_record(dclient,
				"battery voltage is over 4.4V\n");
			pr_info("battery voltage is over 4.4V\n");
			break;

		case DSM_FAKE_FULL:
			dsm_client_record(dclient,
				"report charging full when actual soc is below 95 percent\n");
			pr_info("report charging full when actual soc is below 95 percent\n");
			break;

		case DSM_ABNORMAL_CHARGE_STATUS:
			dsm_client_record(dclient,
				"charging status is charging while charger is not online\n");
			pr_info("charging status is charging while charger is not online\n");
			break;

		case DSM_BATT_VOL_TOO_LOW:
			dsm_client_record(dclient,
				"battery voltage is too low(below 2.5V)\n");
			pr_info("battery voltage is too low(below 2.5V)\n");
			break;

		case DSM_STILL_CHARGE_WHEN_HOT:
			dsm_client_record(dclient,
				"still charge when battery is hot\n");
			pr_info("still charge when battery is hot\n");
			break;

		case DSM_STILL_CHARGE_WHEN_COLD:
			dsm_client_record(dclient,
				"still charge when battery is cold\n");
			pr_info("still charge when battery is cold\n");
			break;

		case DSM_STILL_CHARGE_WHEN_SET_DISCHARGE:
			dsm_client_record(dclient,
				"still charge when we set discharge\n");
			pr_info("still charge when we set discharge\n");
			break;

		case DSM_STILL_CHARGE_WHEN_VOL_OVER_4350:
			dsm_client_record(dclient,
				"still charge when we battery voltage reach or over 4.35V\n");
			pr_info("still charge when we battery voltage reach or over 4.35V\n");
			break;

		case DSM_HEATH_OVERHEAT:
			dsm_client_record(dclient,
				"battery health is overheat\n");
			pr_info("battery health is overheat\n");
			break;

		case DSM_BATTERY_ID_UNKNOW:
			dsm_client_record(dclient,
				"battery id is unknow\n");
			pr_info("battery id is unknow\n");
			break;

		case DSM_BATT_TEMP_JUMP:
			dsm_client_record(dclient,
				"battery temperature change more than 2 degree in short time\n");
			pr_info("battery temperature change more than 2 degree in short time\n");
			break;

		case DSM_BATT_TEMP_BELOW_0:
			dsm_client_record(dclient,
				"battery temperature is below 0 degree\n");
			pr_info("battery temperature is below 0 degree\n");
			break;

		case DSM_BATT_TEMP_OVER_60:
			dsm_client_record(dclient,
				"battery temperature is over 60 degree\n");
			pr_info("battery temperature is over 60 degree\n");
			break;

		case DSM_NOT_CHARGING_WHEN_HOT:
			dsm_client_record(dclient,
				"battery is hot, not charging\n");
			pr_info("battery is hot, not charging\n");
			break;

		case DSM_NOT_CHARGING_WHEN_COLD:
			dsm_client_record(dclient,
				"battery is cold, not charging\n");
			pr_info("battery is cold, not charging\n");
			break;

		case DSM_CHARGE_DISABLED_IN_USER_VERSION:
			dsm_client_record(dclient,
				"set charging_enabled as 0 in user version, it is not allowed\n");
			pr_info("set charging_enabled as 0 in user version, it is not allowed\n");
			break;

		case DSM_SET_FACTORY_DIAG_IN_USER_VERSION:
			dsm_client_record(dclient,
				"set factory_diag as 1 in user version, it is not allowed\n");
			pr_info("set factory_diag as 1 in user version, it is not allowed\n");
			break;

		case DSM_USBIN_IRQ_INVOKE_TOO_QUICK:
			dsm_client_record(dclient,
				"usbin vaild irq invoke too quickly, more than 10 times in 30s\n");
			pr_info("usbin vaild irq invoke too quickly, more than 10 times in 30s\n");
			break;

		case DSM_MAXIM_BAT_CONTACT_BREAK:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: bat contact break\n");
			pr_info("maxim charger batt status reg error: bat contact break\n");
			break;

		case DSM_MAXIM_BATTERY_REMOVED:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: bat removed\n");
			pr_info("maxim charger batt status reg error: bat removed\n");
			break;

		case DSM_MAXIM_BATT_UVP:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: VBAT_UVP\n");
			pr_info("maxim charger batt status reg error: VBAT_UVP\n");
			break;

		case DSM_MAXIM_DC_OVP:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: DC_OVP\n");
			pr_info("maxim charger batt status reg error: DC_OVP\n");
			break;

		case DSM_MAXIM_DC_UVP:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: DC_UVP\n");
			pr_info("maxim charger batt status reg error: DC_UVP\n");
			break;

		case DSM_MAXIM_AICL_NOK:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: AICL_NOK\n");
			pr_info("maxim charger batt status reg error: AICL_NOK\n");
			break;

		case DSM_MAXIM_VBAT_OVP:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: VBAT_OVP\n");
			pr_info("maxim charger batt status reg error: VBAT_OVP\n");
			break;

		case DSM_MAXIM_TEMP_SHUTDOWN:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: TEMP_SHUTDOWN\n");
			pr_info("maxim charger batt status reg error: TEMP_SHUTDOWN\n");
			break;

		case DSM_MAXIM_TIMER_FAULT:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: TIMER_FAULT\n");
			pr_info("maxim charger batt status reg error: TIMER_FAULT\n");
			break;

		case DSM_MAXIM_OTG_OVERCURRENT:
			dsm_client_record(dclient,
				"maxim charger OTG current limit is exceeded "
				"longer than debounce time\n");
			pr_info("maxim charger OTG current limit is exceeded "
				"longer than debounce time\n");
			break;

		case DSM_MAXIM_USB_SUSPEND:
			dsm_client_record(dclient,
				"maxim charger batt status reg error: USB_SUSPEND\n");
			pr_info("maxim charger batt status reg error: USB_SUSPEND\n");
			break;

		case DSM_MAXIM_ENABLE_OTG_FAIL:
			dsm_client_record(dclient,
				"maxim: enable OTG fail\n");
			pr_info("maxim: enable OTG fail\n");
			break;

		case DSM_MAXIM_CHARGE_WHEN_OTGEN:
			dsm_client_record(dclient,
				"maxim: still show charge status when OTG is enabled\n");
			pr_info("maxim: still show charge status when OTG is enabled\n");
			break;

		case DSM_MAXIM_HEATTH_COLD:
			dsm_client_record(dclient,
				"maxim: battery health is cold\n");
			pr_info("maxim: battery health is cold\n");
			break;

		case DSM_LINEAR_USB_OVERTEMP:
			dsm_client_record(dclient,
				"linear charger: USB overtemp!\n");
			pr_info("linear charger: USB overtemp!\n");
			break;

		case DSM_LINEAR_CHG_FAILED:
			dsm_client_record(dclient,
				"linear charger: charge failed!\n");
			pr_info("linear charger: charge failed!\n");
			break;

		case DSM_LINEAR_CHG_OVERCURRENT:
			dsm_client_record(dclient,
				"linear charger: battery charge current is exceeded 1.5A\n");
			pr_info("linear charger: battery charge current is exceeded 1.5A\n");
			break;

		case DSM_LINEAR_BAT_OVERCURRENT:
			dsm_client_record(dclient,
				"linear charger: battery discharge current is exceeded 5A\n");
			pr_info("linear charger: battery discharge current is exceeded 5A\n");
			break;

		case DSM_CHARGER_ADC_ABNORMAL_ERROR_NO:
			dsm_client_record(dclient,
				"Find abnormal ADC values!\n");
			pr_info("Find abnormal ADC values!\n");
			break;

		case DSM_CHARGER_BQ_BOOST_FAULT_ERROR_NO:
			dsm_client_record(dclient,
				"find boost mode fault! such as overload(OTG OCP), VBUS OVP, VBUS < UVLO,"
				"battery OVP, thermal shutdown and so on\n");
			pr_info("find boost mode fault! such as overload(OTG OCP), VBUS OVP, VBUS < UVLO,"
				"battery OVP, thermal shutdown and so on\n");
			break;

		case DSM_CHARGER_BQ_NORMAL_FAULT_ERROR_NO:
			dsm_client_record(dclient,
				"find charge mode fault! such as poor input source, VBUS OVP, battery is too"
				"low, timer fault, thermal shutdown and so on\n");
			pr_info("find charge mode fault! such as poor input source, VBUS OVP, battery is too"
				"low, timer fault, thermal shutdown and so on\n");
			break;

		default:
			break;
	}
}
int dump_registers_and_adc(struct dsm_client *dclient, struct qpnp_lbc_chip *chip, int type)
{
	int i = 0;
	int rc = 0;
	u8 reg_val = 0;
	int vbat_uv, current_ma, batt_temp,batt_id, vusb_uv, vchg_uv, vcoin_uv;
	union power_supply_propval val = {0};

	if( NULL == dclient )
	{
		pr_err("%s: there is no dclient!\n", __func__);
		return -1;
	}

	if( NULL == chip )
	{
		pr_err("%s: chip is NULL!\n", __func__);
		return -1;
	}

	mutex_lock(&chip->dsm_dump_lock);

	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(dclient))
	{
		/* buffer is busy */
		pr_err("%s: buffer is busy!\n", __func__);
		mutex_unlock(&chip->dsm_dump_lock);
		return -1;
	}

	print_basic_info_before_dump(dclient, type);

	/* linear charger and vm_bms*/
	if(!chip->use_other_charger){
		/*First dump LBC_CHGR registers*/
		dsm_client_record(dclient, "[LBC_CHGR] regs:\n");
		pr_debug("[LBC_CHGR] regs:\n");
		for(i = 0; i < ARRAY_SIZE(LBC_CHGR); i++){
			rc = qpnp_lbc_read(chip, chip->chgr_base + LBC_CHGR[i],
					&reg_val, 1);
			if (rc) {
				pr_err("Failed to read chgr_base registers %d\n", rc);
				mutex_unlock(&chip->dsm_dump_lock);
				return -1;
			}
			dsm_client_record(dclient,
					"0x%x, 0x%x\n",
					chip->chgr_base+LBC_CHGR[i], reg_val);
			pr_debug("0x%x, 0x%x\n",
					chip->chgr_base+LBC_CHGR[i], reg_val);
		}

		/*Then dump LBC_BAT_IF registers*/
		dsm_client_record(dclient, "[LBC_BAT_IF] regs:\n");
		pr_debug("[LBC_BAT_IF] regs:\n");
		for(i = 0; i < ARRAY_SIZE(LBC_BAT_IF); i++){
			rc = qpnp_lbc_read(chip, chip->bat_if_base + LBC_BAT_IF[i],
					&reg_val, 1);
			if (rc) {
				pr_err("Failed to read bat_if_base registers %d\n", rc);
				mutex_unlock(&chip->dsm_dump_lock);
				return -1;
			}
			dsm_client_record(dclient,
					"0x%x, 0x%x\n",
					chip->bat_if_base+LBC_BAT_IF[i], reg_val);
			pr_debug("0x%x, 0x%x\n",
					chip->bat_if_base+LBC_BAT_IF[i], reg_val);
		}

		/*Third dump LBC_USB registers*/
		dsm_client_record(dclient, "[LBC_USB] regs:\n");
		pr_debug("[LBC_USB] regs:\n");
		for(i = 0; i < ARRAY_SIZE(LBC_USB); i++){
			rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + LBC_USB[i],
					&reg_val, 1);
			if (rc) {
				pr_err("Failed to read usb_chgpth_base registers %d\n", rc);
				mutex_unlock(&chip->dsm_dump_lock);
				return -1;
			}
			dsm_client_record(dclient,
					"0x%x, 0x%x\n",
					chip->usb_chgpth_base+LBC_USB[i], reg_val);
			pr_debug("0x%x, 0x%x\n",
					chip->usb_chgpth_base+LBC_USB[i], reg_val);
		}

		/*Fourth dump LBC_MISC registers*/
		dsm_client_record(dclient, "[LBC_MISC] regs:\n");
		pr_debug("[LBC_MISC] regs:\n");
		for(i = 0; i < ARRAY_SIZE(LBC_MISC); i++){
			rc = qpnp_lbc_read(chip, chip->misc_base + LBC_MISC[i],
					&reg_val, 1);
			if (rc) {
				pr_err("Failed to read misc_base registers %d\n", rc);
				mutex_unlock(&chip->dsm_dump_lock);
				return -1;
			}
			dsm_client_record(dclient,
					"0x%x, 0x%x\n",
					chip->misc_base+LBC_MISC[i], reg_val);
			pr_debug("0x%x, 0x%x\n",
					chip->misc_base+LBC_MISC[i], reg_val);
		}

		/*If error no is bms soc type, then dump VM_BMS registers*/
		if ((bms_base != 0) && ((DSM_BMS_NORMAL_SOC_CHANGE_MUCH <= type)
			||(DSM_BMS_POWON_SOC_CHANGE_MUCH >= type))){
			dsm_client_record(dclient, "[VM_BMS] regs:\n");
			pr_debug("[VM_BMS] regs:\n");
			for(i = 0; i < ARRAY_SIZE(VM_BMS); i++){
				rc = qpnp_lbc_read(chip, bms_base + VM_BMS[i],
					&reg_val, 1);
				if (rc) {
					pr_err("Failed to read bms_base registers %d\n", rc);
					mutex_unlock(&chip->dsm_dump_lock);
					return -1;
				}
				dsm_client_record(dclient,
						"0x%x, 0x%x\n",
						bms_base+VM_BMS[i], reg_val);
				pr_debug("0x%x, 0x%x\n",
						bms_base+VM_BMS[i], reg_val);
			}
		}
		/*Last save battery vbat current and temp values*/
		vbat_uv = get_prop_battery_voltage_now(chip);
		current_ma = get_prop_current_now(chip);
        batt_temp = get_prop_batt_temp(chip);
	}else if(chip->ti_charger && chip->ti_charger->get_property){
        /*First dump BQ24152 registers*/
        bq2415x_dump_regs(dclient);
        if(!chip->use_only_ti_charger)
        {
            /*Then dump BQ27510 registers*/
            bq27510_dump_regs(dclient);
        }
        chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_VOLTAGE_NOW,&val);
        vbat_uv = val.intval;
        chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_CURRENT_NOW,&val);
        current_ma = val.intval;
        chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_TEMP,&val);
        batt_temp = val.intval;
    }else{
		/*First dump max77819 charger registers*/
		max77819_charger_dump_regs(dclient);
		/*Then dump max17048 fguage registers*/
		max17048_fgauge_dump_regs(dclient);
		/*Last save battery vbat current and temp values*/
		if(chip->maxim_charger && chip->maxim_charger->get_property){
			chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_VOLTAGE_NOW,&val);
			vbat_uv = val.intval;
			chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CURRENT_NOW,&val);
			current_ma = val.intval;
            batt_temp = get_prop_batt_temp(chip);
		}
	}
	batt_id = get_prop_battery_id(chip);
	vusb_uv = get_prop_vbus_uv(chip);
	vchg_uv = get_prop_vchg_uv(chip);
	vcoin_uv = get_prop_vcoin_uv(chip);

	dsm_client_record(dclient,
				"ADC values: vbat=%d current=%d batt_temp=%d "
				"batt_id=%d vusb=%d vchg=%d vcoin=%d\n",
				vbat_uv, current_ma, batt_temp, batt_id, vusb_uv, vchg_uv,
				vcoin_uv);

	pr_info("ADC values: vbat=%d current=%d batt_temp=%d "
				"batt_id=%d vusb=%d vchg=%d vcoin=%d\n",
				vbat_uv, current_ma, batt_temp, batt_id, vusb_uv, vchg_uv,
				vcoin_uv);

	dsm_client_notify(dclient, type);
	mutex_unlock(&chip->dsm_dump_lock);
	return 0;
}
EXPORT_SYMBOL(dump_registers_and_adc);

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

/* if usbin vaild irq invoke more than 20 times in 30s, we report it to dsm server*/
static void usbin_valid_count_work_func(struct work_struct *work)
{
	static int usbin_irq_invoke_count = 0;
	static int start_flag = 0;
	static unsigned long start_tm_sec = 0;
	unsigned long now_tm_sec = 0;
#ifdef CONFIG_LOG_JANK
	int usb_present;
#endif
	struct qpnp_lbc_chip	*chip =
		container_of(work, struct qpnp_lbc_chip, usbin_valid_count_work);

/* Move LOG_JANK code here to prevent sleep in usbin_valid interrupt*/
#ifdef CONFIG_LOG_JANK
	usb_present = qpnp_lbc_is_usb_chg_plugged_in(chip);
	if(usb_present)
	{
		pr_jank(JL_USBCHARGING_START,"%s#T:%5lu","JL_USBCHARGING_START",getrealtime());
	}
	else
	{
		pr_jank(JL_USBCHARGING_END,"%s#T:%5lu","JL_USBCHARGING_END",getrealtime());
	}
#endif

	if(!usbin_irq_invoke_flag){
		mutex_lock(&chip->dsm_soc_lock);
		get_current_time(&usbin_start_tm_sec);
		usbin_irq_invoke_flag = 1;
		mutex_unlock(&chip->dsm_soc_lock);
	}

	/* if the soc changed more than 5 percent in 1 minute*/
	/*dump pmic registers and adc values, and notify to dsm server */
	if(!start_flag){
		get_current_time(&start_tm_sec);
		start_flag = 1;
	}

	get_current_time(&now_tm_sec);
	if(HALF_MINUTE >= (now_tm_sec - start_tm_sec)){
		usbin_irq_invoke_count++;
		pr_debug("usbin_valid_count_work_func is invoked! usbin_irq_invoke_count is %d\n",usbin_irq_invoke_count);
		if(MAX_COUNT <= usbin_irq_invoke_count){
			usbin_irq_invoke_count = 0;
			dump_registers_and_adc(charger_dclient, chip, DSM_USBIN_IRQ_INVOKE_TOO_QUICK);
		}
	}else{
		start_flag = 0;
		start_tm_sec = 0;
		usbin_irq_invoke_count = 0;
	}
}

/* if soc jump more than 1 percent when usbin irq invoke(plug in or out)*/
static void monitor_soc_jump_when_usbinirq_invoke(struct qpnp_lbc_chip *chip)
{
	unsigned long now_tm_sec = 0;
	int cur_capacity = -1;
	static int prev_capacity;
	union power_supply_propval val = {0};
	struct timespec kernel_time;

	if(!usbin_irq_invoke_flag){
		if(!chip->use_other_charger){
			prev_capacity = get_prop_capacity(chip);
		}else if (chip->ti_charger && chip->ti_charger->get_property){
			     chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_CAPACITY,&val);
			     prev_capacity = val.intval;
		}else{
			if (chip->maxim_charger && chip->maxim_charger->get_property){
				chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CAPACITY,&val);
				prev_capacity = val.intval;
			}
		}
	}

	/* if soc jump more than 1 percent in 30 seconds after usbin irq invoke*/
	/*dump pmic registers and adc values, and notify to dsm server */
	ktime_get_ts(&kernel_time);
	get_current_time(&now_tm_sec);
	if((HALF_MINUTE >= (now_tm_sec - usbin_start_tm_sec))
		&& usbin_start_tm_sec){
		if(!chip->use_other_charger){
			cur_capacity = get_prop_capacity(chip);
		}else if (chip->ti_charger && chip->ti_charger->get_property){
			     chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_CAPACITY,&val);
			     cur_capacity = val.intval;
		}else{
			if (chip->maxim_charger && chip->maxim_charger->get_property){
				chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CAPACITY,&val);
				cur_capacity = val.intval;
			}
		}
		if(cur_capacity >= 0 && (INIT_TIME <= kernel_time.tv_sec) && abs(cur_capacity - prev_capacity) > SOC_JUMP_USBIN){
			pr_info("soc changed more than 1 percent in 30s after usbin irq invoke "
				"cur_capacity=%d prev_capacity=%d\n",
				cur_capacity,
				prev_capacity);
			dump_registers_and_adc(bms_dclient, g_lbc_chip, DSM_BMS_SOC_CHANGE_PLUG_INOUT);
		}
	}else{
		mutex_lock(&chip->dsm_soc_lock);
		usbin_irq_invoke_flag = 0;
		usbin_start_tm_sec = 0;
		mutex_unlock(&chip->dsm_soc_lock);
	}

}

/* this work is lanch by batt_pres abnormal, and online status error and others*/
static void qpnp_lbc_dump_work(struct work_struct *work)
{
	struct qpnp_lbc_chip	*chip =
		container_of(work, struct qpnp_lbc_chip, dump_work);

	pr_info("qpnp_lbc_dump_work is invoked! error type is %d\n",chip->error_type);
	dump_registers_and_adc(charger_dclient, chip, chip->error_type);
}

/* deleted delay_determine_initial_status, 4 lines */

/*when usb is plugged in, we lanch this work to monitor the charging status*/
static void check_charging_batt_status_work(struct work_struct *work)
{
	int vbat_uv, batt_temp, bat_present, cur_status, batt_level, usb_present, input_current=0;
	int chg_type, bat_current, resume_en, health;
	u8 bat_sts, chg_sts, usb_sts, chg_ctrl, suspend_en;
	static int not_charge_count = 0, cannot_charge_count = 0;
	static int start_dismatch_detect = 0;
	static int hot_charging_count = 0, cold_charging_count = 0;
	static int warm_exceed_limit_count = 0,  cool_exceed_limit_count = 0;
	static int previous_temp = INIT_TEMP;
	static unsigned long previous_tm_sec = 0;
	static int status_not_match_count = 0;
	unsigned long now_tm_sec = 0;
	int mode = 0;
	int rc = 0;
	int reg_value[5] = {0};
	u8 reg = 0;
	int current_max = 0, current_ma = 0;
	int nff_code = 0;
	int voltage_regulation = 0;
	bool charger_timeout = false;
	union power_supply_propval val = {0};
	struct qpnp_lbc_chip *chip =
		container_of(work, struct qpnp_lbc_chip, check_charging_batt_status_work.work);

	bat_present = get_prop_batt_present(chip);
	usb_present = qpnp_lbc_is_usb_chg_plugged_in(chip);
	batt_temp = get_prop_batt_temp(chip);

	/* get /sys/class/power_supply/usb/current_max value */
	chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	current_max = val.intval / 1000;

	if(!chip->use_other_charger){
		vbat_uv = get_prop_battery_voltage_now(chip);
		cur_status = get_prop_batt_status(chip);
		batt_level = get_prop_capacity(chip);
		health = get_prop_batt_health(chip);
		current_ma = get_prop_current_now(chip);
		input_current = qpnp_lbc_ibatmax_get(chip);
	}else if (chip->maxim_charger && chip->maxim_charger->get_property){
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_STATUS,&val);
		cur_status = val.intval;
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_VOLTAGE_NOW,&val);
		vbat_uv = val.intval;
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CAPACITY,&val);
		batt_level = val.intval;
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_HEALTH,&val);
		health = val.intval;
		mode = get_max77819_boost_mode();
		charger_timeout = get_max77819_charger_timeout();
		get_temp_ctrl_info_from_max77819(chip);
	}else{
		if(chip->ti_charger && chip->ti_charger->get_property){
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_VOLTAGE_NOW,&val);
			vbat_uv = val.intval;
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_TEMP,&val);
			batt_temp = val.intval;
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_STATUS,&val);
			cur_status = val.intval;
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_CAPACITY,&val);
			batt_level = val.intval;
            chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_HEALTH,&val);
            health = val.intval;
		}
		mode = is_bq24152_in_boost_mode();
	}
	/* if usb ovp happens*/
	/*dump pmic registers and adc values, and notify to dsm server */
	if(is_usb_ovp(chip)){
		dump_registers_and_adc(charger_dclient, chip, DSM_CHG_OVP_ERROR_NO);
	}

	/*if all the charging conditions are avaliable, but it still cannot charge, */
	/*we report the error and dump the registers values and ADC values  */
	if((((chip->cfg_hot_bat_decidegc - TEMP_BUFFER) > batt_temp)
		&& (vbat_uv <= (chip->cfg_warm_bat_mv - WARM_VOL_BUFFER))
		&& (batt_temp >= (chip->cfg_warm_bat_decidegc - TEMP_BUFFER)))
		|| (((chip->cfg_cold_bat_decidegc + TEMP_BUFFER) < batt_temp)
		&& (batt_temp < (chip->cfg_warm_bat_decidegc - TEMP_BUFFER)))){

		if((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
			&& (BATT_FULL_LEVEL != batt_level) && usb_present
			&& (current_max > 2) && (0 == mode)
			&& (!chip->cfg_charging_disabled) && (!factory_diag_flag)
			&& bat_present && (POWER_SUPPLY_STATUS_FULL != cur_status)
			&&(false == charger_timeout)){

			if(cannot_charge_count++ < NOT_CHARGE_COUNT){
				pr_info("cannot charge when allowed, count is %d\n", cannot_charge_count);
			}else{
				cannot_charge_count = 0;
				dump_registers_and_adc(charger_dclient, chip, DSM_NOT_CHARGE_WHEN_ALLOWED);
			}
		}else{
			cannot_charge_count = 0;
		}
	}

	if(!bat_present){ /* battery absent */
		dump_registers_and_adc(charger_dclient, chip, DSM_BATT_PRES_ERROR_NO);
	}else{
		if(!usb_present){ /* usb absent, discharging */
			if(START_DISMATCH_COUNT <= start_dismatch_detect++){
				start_dismatch_detect = START_DISMATCH_COUNT;
				if((VOL_THR1 <= vbat_uv) && (SOC_ZERO == batt_level)){
					dump_registers_and_adc(bms_dclient, chip, DSM_BMS_VOL_SOC_DISMATCH_1);
				}

				if((VOL_THR2 <= vbat_uv) && (SOC_ZERO != batt_level)
					&& (SOC_THR1 >= batt_level)){
					dump_registers_and_adc(bms_dclient, chip, DSM_BMS_VOL_SOC_DISMATCH_2);
				}

				if((VOL_HIGH <= vbat_uv) && (SOC_ZERO != batt_level)
					&& (SOC_HIGH >= batt_level)){
					dump_registers_and_adc(bms_dclient, chip, DSM_BMS_VOL_SOC_DISMATCH_3);
				}

				if((VOL_TOO_LOW >= vbat_uv) && (SOC_THR1 == batt_level)){
					dump_registers_and_adc(bms_dclient, chip, DSM_VM_BMS_VOL_SOC_DISMATCH_4);
				}
			}
			
			if(HIGH_VOL <= vbat_uv){
				dump_registers_and_adc(charger_dclient, chip, DSM_BATT_VOL_OVER_4400);
			}

			if(STATUS_NOT_MATCH_COUNT <= status_not_match_count++){
				status_not_match_count = 0;
				if(POWER_SUPPLY_STATUS_CHARGING == cur_status){
					dump_registers_and_adc(charger_dclient, chip, DSM_ABNORMAL_CHARGE_STATUS);
				}

				if(POWER_SUPPLY_STATUS_FULL == cur_status){
					dump_registers_and_adc(charger_dclient, chip, DSM_FULL_WHEN_CHARGER_ABSENT);
				}
			}
		}

		/* usb present and not otg mode*/
		if(usb_present && (0 == mode)){
			if((POWER_SUPPLY_STATUS_FULL == cur_status)
				&& (SOC_HIGH_THR >= batt_level)){
				dump_registers_and_adc(charger_dclient, chip, DSM_FAKE_FULL);
			}

			if( !is_sw_factory_mode() && factory_diag_flag){
				dump_registers_and_adc(charger_dclient, chip, DSM_SET_FACTORY_DIAG_IN_USER_VERSION);
			}

			/* chip->cfg_charging_disabled is true: charging_enabled is 0*/
			if(chip->cfg_charging_disabled){
				if(POWER_SUPPLY_STATUS_CHARGING == cur_status){
					dump_registers_and_adc(charger_dclient, chip, DSM_STILL_CHARGE_WHEN_SET_DISCHARGE);
				}
				/* as C199s use charging_enabled node to control flash light */
				/* so C199s ignore this monitor point, this point keeps for linear charger*/
				if(!is_sw_factory_mode() && (!chip->maxim_charger)){
					dump_registers_and_adc(charger_dclient, chip, DSM_CHARGE_DISABLED_IN_USER_VERSION);
				}
			/* chip->cfg_charging_disabled is false: charging_enabled is 1*/
			}else{
				/* maxim ic voltage regulation is high, so C199s use 4.45V threshold*/
				if(chip->maxim_charger){
					voltage_regulation = VOL_REGULATION_MAX_MAXIM;
				}else{
					voltage_regulation = VOL_REGULATION_MAX;
				}

				if((voltage_regulation <= vbat_uv)&&(POWER_SUPPLY_STATUS_CHARGING == cur_status)){
					dump_registers_and_adc(charger_dclient, chip, DSM_STILL_CHARGE_WHEN_VOL_OVER_4350);
				}

				if(((chip->cfg_hot_bat_decidegc + TEMP_BUFFER) < batt_temp)
					&&((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
					||(POWER_SUPPLY_STATUS_NOT_CHARGING == cur_status))){
					dump_registers_and_adc(charger_dclient, chip, DSM_NOT_CHARGING_WHEN_HOT);
				}

				if(((chip->cfg_cold_bat_decidegc - TEMP_BUFFER) > batt_temp)
					&&((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
					||(POWER_SUPPLY_STATUS_NOT_CHARGING == cur_status))){
					dump_registers_and_adc(charger_dclient, chip, DSM_NOT_CHARGING_WHEN_COLD);
				}

				if(((chip->cfg_hot_bat_decidegc + TEMP_BUFFER) < batt_temp)
					&&(POWER_SUPPLY_STATUS_CHARGING == cur_status)){
					if(hot_charging_count++ < DSM_COUNT){
						pr_info("still charge when battery is hot, count is %d\n", hot_charging_count);
					}else{
						hot_charging_count = 0;
						dump_registers_and_adc(charger_dclient, chip, DSM_STILL_CHARGE_WHEN_HOT);
					}
				}else{
					hot_charging_count = 0;
				}

				if(((chip->cfg_cold_bat_decidegc - TEMP_BUFFER) > batt_temp)
					&&(POWER_SUPPLY_STATUS_CHARGING == cur_status)){
					if(cold_charging_count++ < DSM_COUNT){
						pr_info("still charge when battery is cold, count is %d\n", cold_charging_count);
					}else{
						cold_charging_count = 0;
						dump_registers_and_adc(charger_dclient, chip, DSM_STILL_CHARGE_WHEN_COLD);
					}
				}else{
					cold_charging_count = 0;
				}

				if(((chip->cfg_warm_bat_decidegc + TEMP_BUFFER) < batt_temp)
					&& (!chip->use_other_charger)
					&&(WARM_COOL_CURRENT_LIMIT < input_current)){
					if(warm_exceed_limit_count++ < DSM_COUNT){
						pr_info("current is over warm current limit when warm, count is %d\n", warm_exceed_limit_count);
					}else{
						warm_exceed_limit_count = 0;
						dump_registers_and_adc(charger_dclient, chip, DSM_WARM_CURRENT_LIMIT_FAIL);
					}
				}else{
					warm_exceed_limit_count = 0;
				}

				if(((chip->cfg_cool_bat_decidegc - TEMP_BUFFER) > batt_temp)
					&& (!chip->use_other_charger)
					&&(WARM_COOL_CURRENT_LIMIT < input_current)){
					if(cool_exceed_limit_count++ < DSM_COUNT){
						pr_info("current is over cool current limit when cool, count is %d\n", cool_exceed_limit_count);
					}else{
						cool_exceed_limit_count = 0;
						dump_registers_and_adc(charger_dclient, chip, DSM_COOL_CURRENT_LIMIT_FAIL);
					}
				}else{
					cool_exceed_limit_count = 0;
				}
			}
		}

		/* monitor whether soc jump more than 1 percent when plug in out charger(in 30 seconds)*/
		monitor_soc_jump_when_usbinirq_invoke(chip);

		if(LOW_VOL >= vbat_uv){
			dump_registers_and_adc(charger_dclient, chip, DSM_BATT_VOL_TOO_LOW);
		}

		if(POWER_SUPPLY_HEALTH_OVERHEAT == health){
			dump_registers_and_adc(charger_dclient, chip, DSM_HEATH_OVERHEAT);
		}

		if(POWER_SUPPLY_HEALTH_COLD == health){
			dump_registers_and_adc(charger_dclient, chip, DSM_MAXIM_HEATTH_COLD);
		}
		get_current_time(&now_tm_sec);
		/* only care 20 to 40 temperature zone in centigrade, */
		/* if temp jumps in this zone in 15 seconds, notify to dsm server*/
		if((abs(previous_temp - batt_temp) > TEMP_DELTA)
			&& (TEMP_LOWER_THR <= batt_temp)
			&& (TEMP_UPPER_THR >= batt_temp)
			&& (TEMP_LOWER_THR <= previous_temp)
			&& (TEMP_UPPER_THR >= previous_temp)
			&& (INIT_TEMP != previous_temp)
			&& (QUARTER_MINUTE >= (now_tm_sec -previous_tm_sec))){
			dump_registers_and_adc(charger_dclient, chip, DSM_BATT_TEMP_JUMP);
		}
		previous_temp = batt_temp;
		previous_tm_sec = now_tm_sec;

		if(HOT_TEMP < batt_temp){
			dump_registers_and_adc(charger_dclient, chip, DSM_BATT_TEMP_OVER_60);
		}

		if(LOW_TEMP > batt_temp){
			dump_registers_and_adc(charger_dclient, chip, DSM_BATT_TEMP_BELOW_0);
		}

		if(!chip->use_other_charger){
			if(CHARGE_CURRENT_MAX > current_ma){
				dump_registers_and_adc(charger_dclient, chip, DSM_LINEAR_CHG_OVERCURRENT);
			}

			if(OVER_CURRENT < current_ma){
				dump_registers_and_adc(charger_dclient, chip, DSM_LINEAR_BAT_OVERCURRENT);
			}
		}

		if(!chip->cfg_charging_disabled && mode
			&& (POWER_SUPPLY_STATUS_CHARGING == cur_status)){
			dump_registers_and_adc(charger_dclient, chip, DSM_MAXIM_CHARGE_WHEN_OTGEN);
		}
	}

	/* Add NFF log here */
	usb_present = qpnp_lbc_is_usb_chg_plugged_in(chip);
	if(usb_present && (cur_status == POWER_SUPPLY_STATUS_DISCHARGING
		|| cur_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
		&& !chip->cfg_charging_disabled && (0 == mode)
		&& BATT_FULL_LEVEL != batt_level
		&& (current_max > 2)){
		pr_info("current status need to be checked %d %d %d\n",cur_status,usb_present,chip->cfg_charging_disabled);
		/*not charging for more then 120s*/
		if(not_charge_count++ < NOT_CHARGE_COUNT){
			goto skip_check_status;
		}
		else{
			not_charge_count = 0;
		}
	}
	else{
		pr_debug("right status\n");
		not_charge_count = 0;
		goto skip_check_status;
	}

	/* For 8916 linear charger */
	if(!chip->use_other_charger){
		/* get the battery info and charge sts */
		/* Remove redundancy code  */
		chg_type = get_prop_charge_type(chip);
		bat_current = get_prop_current_now(chip);

		rc = qpnp_lbc_read(chip, INT_RT_STS(chip->bat_if_base), &bat_sts, 1);
		if (rc)
			pr_err("failed to read batt_sts rc=%d\n", rc);
		rc = qpnp_lbc_read(chip, INT_RT_STS(chip->chgr_base), &chg_sts, 1);
		if (rc)
			pr_err("failed to read chg_sts rc=%d\n", rc);
		rc = qpnp_lbc_read(chip, INT_RT_STS(chip->usb_chgpth_base), &usb_sts, 1);
		if (rc)
			pr_err("failed to read usb_sts rc=%d\n", rc);
		rc = qpnp_lbc_read(chip, chip->chgr_base + CHG_CTRL_REG, &chg_ctrl, 1);
		if (rc)
			pr_err("failed to read chg_ctrl rc=%d\n", rc);
		rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + USB_SUSP_REG, &suspend_en, 1);
		if (rc)
			pr_err("failed to read suspend_en rc=%d\n", rc);

		resume_en = chip->resuming_charging;
		input_current = qpnp_lbc_ibatmax_get(chip);

		if(!bat_present){
			MSG_WRAPPER(CHARGE_ERROR_BASE|BATTERY_ERR,
				"%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, chg_type, bat_current,
				bat_sts, chg_sts, usb_sts, chg_ctrl, suspend_en, resume_en, input_current);
		}
		else if(batt_temp >= chip->cfg_hot_bat_decidegc || batt_temp <= chip->cfg_cold_bat_decidegc){
			MSG_WRAPPER(CHARGE_ERROR_BASE|TMEPERATURE_ERR|TMEPERATURE_OVERFLOW,
				"%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, chg_type, bat_current,
				bat_sts, chg_sts, usb_sts, chg_ctrl, suspend_en, resume_en, input_current);
		}
		else if(batt_temp < chip->cfg_hot_bat_decidegc && batt_temp >= chip->cfg_warm_bat_decidegc
			&& vbat_uv >= chip->cfg_warm_bat_mv){
			MSG_WRAPPER(CHARGE_ERROR_BASE|TMEPERATURE_ERR|TMEPERATURE_LIMIT,
				"%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, chg_type, bat_current,
				bat_sts, chg_sts, usb_sts, chg_ctrl, suspend_en, resume_en, input_current);
		}
		else{
			MSG_WRAPPER(CHARGE_ERROR_BASE,
				"%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, chg_type, bat_current,
				bat_sts, chg_sts, usb_sts, chg_ctrl, suspend_en, resume_en, input_current);
		}

		pr_info("%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
			cur_status, batt_level, bat_present, vbat_uv, batt_temp, chg_type, bat_current,
			bat_sts, chg_sts, usb_sts, chg_ctrl, suspend_en, resume_en, input_current);
	/* For maxim77819 charger */
	} else if (chip->maxim_charger && chip->maxim_charger->get_property) {
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CURRENT_NOW,&val);
		bat_current = val.intval;

		if(!bat_present){
			nff_code = CHARGE_ERROR_BASE|BATTERY_ERR;
		}
		else if(batt_temp >= chip->cfg_hot_bat_decidegc || batt_temp <= chip->cfg_cold_bat_decidegc){
			nff_code = CHARGE_ERROR_BASE|TMEPERATURE_ERR|TMEPERATURE_OVERFLOW;
		}
		else if(batt_temp < chip->cfg_hot_bat_decidegc && batt_temp >= chip->cfg_warm_bat_decidegc
			&& vbat_uv >= chip->cfg_warm_bat_mv){
			nff_code = CHARGE_ERROR_BASE|TMEPERATURE_ERR|TMEPERATURE_LIMIT;
		}
		else{
			nff_code = CHARGE_ERROR_BASE;
		}
		chip->maxim_charger->get_property(chip->maxim_charger,POWER_SUPPLY_PROP_CHARGE_LOG,&val);

		MSG_WRAPPER(nff_code,"%d %d %d %d %d %d %d %s\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, bat_current, mode, val.strval);

		pr_info("%d %d %d %d %d %d %d %s\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, bat_current, mode, val.strval);
	/* For G760 Ti charger */
	}else{
		if(chip->ti_charger && chip->ti_charger->get_property){
			chip->ti_charger->get_property(chip->ti_charger,POWER_SUPPLY_PROP_CURRENT_NOW,&val);
			bat_current = val.intval;
		}
		/* read bq2415x reg0 to reg4 values */
		for(reg = BQ2415X_REG_STATUS; reg <= BQ2415X_REG_CURRENT; reg++){
			reg_value[reg] = get_bq2415x_reg_values(reg);
		}
		if(!bat_present){
			MSG_WRAPPER(CHARGE_ERROR_BASE|BATTERY_ERR,
				"%d %d %d %d %d %d %x %x %x %x %x\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, bat_current,
				reg_value[0], reg_value[1], reg_value[2], reg_value[3], 
				reg_value[4]);
		}
		else if(batt_temp >= chip->cfg_hot_bat_decidegc || batt_temp <= chip->cfg_cold_bat_decidegc){
			MSG_WRAPPER(CHARGE_ERROR_BASE|TMEPERATURE_ERR|TMEPERATURE_OVERFLOW,
				"%d %d %d %d %d %d %x %x %x %x %x\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, bat_current,
				reg_value[0], reg_value[1], reg_value[2], reg_value[3], 
				reg_value[4]);
		}
		else if(batt_temp < chip->cfg_hot_bat_decidegc && batt_temp >= chip->cfg_warm_bat_decidegc
			&& vbat_uv >= chip->cfg_warm_bat_mv){
			MSG_WRAPPER(CHARGE_ERROR_BASE|TMEPERATURE_ERR|TMEPERATURE_LIMIT,
				"%d %d %d %d %d %d %x %x %x %x %x\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, bat_current,
				reg_value[0], reg_value[1], reg_value[2], reg_value[3], 
				reg_value[4]);
		}
		else{
			MSG_WRAPPER(CHARGE_ERROR_BASE,
				"%d %d %d %d %d %d %x %x %x %x %x\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, bat_current,
				reg_value[0], reg_value[1], reg_value[2], reg_value[3], 
				reg_value[4]);
		}

		pr_info("%d %d %d %d %d %d %x %x %x %x %x\n",
				cur_status, batt_level, bat_present, vbat_uv, batt_temp, bat_current,
				reg_value[0], reg_value[1], reg_value[2], reg_value[3], 
				reg_value[4]);
	}
skip_check_status:
	schedule_delayed_work(&chip->check_charging_batt_status_work,
			msecs_to_jiffies(CHECKING_TIME));
}
#endif
#define IBAT_TRIM			-300
static void qpnp_lbc_vddtrim_work_fn(struct work_struct *work)
{
	int rc, vbat_now_uv, ibat_now;
	u8 reg_val;
	ktime_t kt;
	struct qpnp_lbc_chip *chip = container_of(work, struct qpnp_lbc_chip,
						vddtrim_work);

	vbat_now_uv = get_prop_battery_voltage_now(chip);
	ibat_now = get_prop_current_now(chip) / 1000;
	pr_debug("vbat %d ibat %d capacity %d\n",
			vbat_now_uv, ibat_now, get_prop_capacity(chip));

	/*
	 * Stop trimming under following condition:
	 * USB removed
	 * Charging Stopped
	 */
	if (!qpnp_lbc_is_fastchg_on(chip) ||
			!qpnp_lbc_is_usb_chg_plugged_in(chip)) {
		pr_debug("stop trim charging stopped\n");
		goto exit;
	} else {
		rc = qpnp_lbc_read(chip, chip->chgr_base + CHG_STATUS_REG,
					&reg_val, 1);
		if (rc) {
			pr_err("Failed to read chg status rc=%d\n", rc);
			goto out;
		}

		/*
		 * Update VDD trim voltage only if following conditions are
		 * met:
		 * If charger is in VDD loop AND
		 * If ibat is between 0 ma and -300 ma
		 */
		if ((reg_val & CHG_VDD_LOOP_BIT) &&
				((ibat_now < 0) && (ibat_now > IBAT_TRIM)))
			qpnp_lbc_adjust_vddmax(chip, vbat_now_uv);
	}

out:
	kt = ns_to_ktime(TRIM_PERIOD_NS);
	alarm_start_relative(&chip->vddtrim_alarm, kt);
exit:
	pm_relax(chip->dev);
}

static enum alarmtimer_restart vddtrim_callback(struct alarm *alarm,
					ktime_t now)
{
	struct qpnp_lbc_chip *chip = container_of(alarm, struct qpnp_lbc_chip,
						vddtrim_alarm);

	pm_stay_awake(chip->dev);
	schedule_work(&chip->vddtrim_work);

	return ALARMTIMER_NORESTART;
}

static int qpnp_lbc_probe(struct spmi_device *spmi)
{
	u8 subtype;
	ktime_t kt;
	struct qpnp_lbc_chip *chip;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	struct power_supply *usb_psy;
	int rc = 0;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_lbc_chip),
				GFP_KERNEL);
	if (!chip) {
		pr_err("memory allocation failed.\n");
		return -ENOMEM;
	}

	chip->usb_psy = usb_psy;
	chip->dev = &spmi->dev;
	chip->spmi = spmi;
	chip->fake_battery_soc = -EINVAL;
#ifdef CONFIG_HUAWEI_KERNEL
	chip->cfg_soc_resume_charging = false;
#endif
	dev_set_drvdata(&spmi->dev, chip);
	device_init_wakeup(&spmi->dev, 1);
	mutex_init(&chip->jeita_configure_lock);
	mutex_init(&chip->chg_enable_lock);
	spin_lock_init(&chip->hw_access_lock);
	spin_lock_init(&chip->ibat_change_lock);
	spin_lock_init(&chip->irq_lock);
#ifdef CONFIG_HUAWEI_KERNEL
	spin_lock_init(&chip->chg_en_lock);
#endif
	INIT_WORK(&chip->vddtrim_work, qpnp_lbc_vddtrim_work_fn);
	alarm_init(&chip->vddtrim_alarm, ALARM_REALTIME, vddtrim_callback);

#ifdef CONFIG_HUAWEI_KERNEL
	chip->cfg_cold_bat_decidegc = COLD_TEMP_DEFAULT;
	chip->cfg_hot_bat_decidegc = HOT_TEMP_DEFAULT;
	chip->running_test_settled_status = POWER_SUPPLY_STATUS_CHARGING;
#endif
#ifdef CONFIG_HUAWEI_DSM
	chip->error_type = -EINVAL;
	mutex_init(&chip->dsm_dump_lock);
	mutex_init(&chip->dsm_soc_lock);
#endif
	/* Get all device-tree properties */
	rc = qpnp_charger_read_dt_props(chip);
	if (rc) {
		pr_err("Failed to read DT properties rc=%d\n", rc);
		return rc;
	}

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("spmi resource absent\n");
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
							IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
						spmi->dev.of_node->full_name);
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		rc = qpnp_lbc_read(chip, resource->start + PERP_SUBTYPE_REG,
					&subtype, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			goto fail_chg_enable;
		}

		switch (subtype) {
		case LBC_CHGR_SUBTYPE:
			chip->chgr_base = resource->start;

			/* Get Charger peripheral irq numbers */
			rc = qpnp_lbc_get_irqs(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to get CHGR irqs rc=%d\n", rc);
				goto fail_chg_enable;
			}
			break;
		case LBC_USB_PTH_SUBTYPE:
			chip->usb_chgpth_base = resource->start;
			rc = qpnp_lbc_get_irqs(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to get USB_PTH irqs rc=%d\n",
						rc);
				goto fail_chg_enable;
			}
			break;
		case LBC_BAT_IF_SUBTYPE:
			chip->bat_if_base = resource->start;
			chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
			if (IS_ERR(chip->vadc_dev)) {
				rc = PTR_ERR(chip->vadc_dev);
				if (rc != -EPROBE_DEFER)
					pr_err("vadc prop missing rc=%d\n",
							rc);
				goto fail_chg_enable;
			}
			/* Get Charger Batt-IF peripheral irq numbers */
			rc = qpnp_lbc_get_irqs(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to get BAT_IF irqs rc=%d\n", rc);
				goto fail_chg_enable;
			}
			break;
		case LBC_MISC_SUBTYPE:
			chip->misc_base = resource->start;
			break;
		default:
			pr_err("Invalid peripheral subtype=0x%x\n", subtype);
			rc = -EINVAL;
		}
	}

	if (chip->cfg_use_external_charger) {
		pr_warn("Disabling Linear Charger (e-external-charger = 1)\n");
		rc = qpnp_disable_lbc_charger(chip);
		if (rc)
			pr_err("Unable to disable charger rc=%d\n", rc);
		return -ENODEV;
	}

#ifdef CONFIG_HUAWEI_KERNEL
	rc = qpnp_chg_load_battery_data(chip);
	if (rc)
		goto fail_chg_enable;
#endif
	/* Initialize h/w */
	rc = qpnp_lbc_misc_init(chip);
	if (rc) {
		pr_err("unable to initialize LBC MISC rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_chg_init(chip);
	if (rc) {
		pr_err("unable to initialize LBC charger rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_bat_if_init(chip);
	if (rc) {
		pr_err("unable to initialize LBC BAT_IF rc=%d\n", rc);
		return rc;
	}
	rc = qpnp_lbc_usb_path_init(chip);
	if (rc) {
		pr_err("unable to initialize LBC USB path rc=%d\n", rc);
		return rc;
	}

#ifdef CONFIG_HUAWEI_KERNEL
	chip->ti_charger = NULL;
	chip->maxim_charger = NULL;
#endif

	if (chip->bat_if_base) {
		chip->batt_present = qpnp_lbc_is_batt_present(chip);
		chip->batt_psy.name = "battery";
#ifdef CONFIG_HUAWEI_KERNEL
		if(!chip->use_other_charger)
			chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
		else
			chip->batt_psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
#else
		chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
#endif
		chip->batt_psy.properties = msm_batt_power_props;
		chip->batt_psy.num_properties =
			ARRAY_SIZE(msm_batt_power_props);
		chip->batt_psy.get_property = qpnp_batt_power_get_property;
		chip->batt_psy.set_property = qpnp_batt_power_set_property;
		chip->batt_psy.property_is_writeable =
			qpnp_batt_property_is_writeable;
		chip->batt_psy.external_power_changed =
			qpnp_batt_external_power_changed;
		chip->batt_psy.supplied_to = pm_batt_supplied_to;
		chip->batt_psy.num_supplicants =
			ARRAY_SIZE(pm_batt_supplied_to);
		rc = power_supply_register(chip->dev, &chip->batt_psy);
		if (rc < 0) {
			pr_err("batt failed to register rc=%d\n", rc);
			goto fail_chg_enable;
		}
	}

	if ((chip->cfg_cool_bat_decidegc || chip->cfg_warm_bat_decidegc)
			&& chip->bat_if_base) {
		chip->adc_param.low_temp = chip->cfg_cool_bat_decidegc;
		chip->adc_param.high_temp = chip->cfg_warm_bat_decidegc;
		chip->adc_param.timer_interval = ADC_MEAS1_INTERVAL_1S;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->adc_param.btm_ctx = chip;
		chip->adc_param.threshold_notification =
			qpnp_lbc_jeita_adc_notification;
		chip->adc_param.channel = LR_MUX1_BATT_THERM;

		if (get_prop_batt_present(chip)) {
			rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->adc_param);
			if (rc) {
				pr_err("request ADC error rc=%d\n", rc);
				goto unregister_batt;
			}
		}
	}

	rc = qpnp_lbc_bat_if_configure_btc(chip);
	if (rc) {
		pr_err("Failed to configure btc rc=%d\n", rc);
		goto unregister_batt;
	}

	/* Get/Set charger's initial status */
	determine_initial_status(chip);

#ifdef CONFIG_HUAWEI_KERNEL
	wake_lock_init(&chip->led_wake_lock, WAKE_LOCK_SUSPEND, "pm8916_led");
	wake_lock_init(&chip->chg_wake_lock, WAKE_LOCK_SUSPEND, "pm8916_chg");
	if(qpnp_lbc_is_usb_chg_plugged_in(chip)){
		wake_lock(&chip->chg_wake_lock);
	}
	g_lbc_chip = chip;
#endif

#ifdef CONFIG_HUAWEI_DSM
	/*Add two works, first work func is used to dump log, second work func is */
	/*checking charging status every 30 seconds */
	INIT_WORK(&chip->usbin_valid_count_work, usbin_valid_count_work_func);
	INIT_WORK(&chip->dump_work, qpnp_lbc_dump_work);
	INIT_DELAYED_WORK(&chip->check_charging_batt_status_work,
				check_charging_batt_status_work);
	/* deleted 1 line */
	if (!charger_dclient) {
		charger_dclient = dsm_register_client(&dsm_charger);
	}
	/* delete some lines */

	schedule_delayed_work(&chip->check_charging_batt_status_work,
				msecs_to_jiffies(DELAY_TIME));
#endif
	rc = qpnp_lbc_request_irqs(chip);
	if (rc) {
		pr_err("unable to initialize LBC MISC rc=%d\n", rc);
		goto unregister_batt;
	}

	if (chip->cfg_charging_disabled && !get_prop_batt_present(chip))
		pr_info("Battery absent and charging disabled !!!\n");

	/* Configure initial alarm for VDD trim */
	if ((chip->supported_feature_flag & VDD_TRIM_SUPPORTED) &&
			qpnp_lbc_is_fastchg_on(chip)) {
		kt = ns_to_ktime(TRIM_PERIOD_NS);
		alarm_start_relative(&chip->vddtrim_alarm, kt);
	}

	pr_info("Probe chg_dis=%d bpd=%d usb=%d batt_pres=%d batt_volt=%d soc=%d\n",
			chip->cfg_charging_disabled,
			chip->cfg_bpd_detection,
			qpnp_lbc_is_usb_chg_plugged_in(chip),
			get_prop_batt_present(chip),
			get_prop_battery_voltage_now(chip),
			get_prop_capacity(chip));

#ifdef CONFIG_HUAWEI_KERNEL
	global_chip = chip;
#endif
	  /* deleted 1 line */
	return 0;

unregister_batt:
	if (chip->bat_if_base)
		power_supply_unregister(&chip->batt_psy);
fail_chg_enable:
	dev_set_drvdata(&spmi->dev, NULL);
#ifdef CONFIG_HUAWEI_DSM
	g_lbc_chip = NULL;
#endif
	return rc;
}

static int qpnp_lbc_remove(struct spmi_device *spmi)
{
	struct qpnp_lbc_chip *chip = dev_get_drvdata(&spmi->dev);
#ifdef CONFIG_HUAWEI_KERNEL
	wake_lock_destroy(&chip->led_wake_lock);
	wake_lock_destroy(&chip->chg_wake_lock);
#endif

#ifdef CONFIG_HUAWEI_DSM
	cancel_work_sync(&chip->usbin_valid_count_work);
	cancel_work_sync(&chip->dump_work);
	cancel_delayed_work_sync(&chip->check_charging_batt_status_work);
	g_lbc_chip = NULL;
	mutex_destroy(&chip->dsm_dump_lock);
	mutex_destroy(&chip->dsm_soc_lock);
#endif

	/* deleted 1 line */
	if (chip->supported_feature_flag & VDD_TRIM_SUPPORTED) {
		alarm_cancel(&chip->vddtrim_alarm);
		cancel_work_sync(&chip->vddtrim_work);
	}
	if (chip->bat_if_base)
		power_supply_unregister(&chip->batt_psy);
	mutex_destroy(&chip->jeita_configure_lock);
	mutex_destroy(&chip->chg_enable_lock);
	dev_set_drvdata(&spmi->dev, NULL);
	return 0;
}

#ifdef CONFIG_HUAWEI_DSM
static int lbc_suspend(struct device *dev)
{
	struct qpnp_lbc_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chip->check_charging_batt_status_work);

	return 0;
}
static int lbc_resume(struct device *dev)
{
	struct qpnp_lbc_chip *chip = dev_get_drvdata(dev);

	schedule_delayed_work(&chip->check_charging_batt_status_work,
				msecs_to_jiffies(0));

	return 0;
}

static const struct dev_pm_ops qpnp_lbc_pm_ops = {
	.suspend	= lbc_suspend,
	.resume	= lbc_resume,
};
#endif

static struct of_device_id qpnp_lbc_match_table[] = {
	{ .compatible = QPNP_CHARGER_DEV_NAME, },
	{}
};

static struct spmi_driver qpnp_lbc_driver = {
	.probe		= qpnp_lbc_probe,
	.remove		= qpnp_lbc_remove,
	.driver		= {
		.name		= QPNP_CHARGER_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= qpnp_lbc_match_table,
#ifdef CONFIG_HUAWEI_DSM
		.pm		= &qpnp_lbc_pm_ops,
#endif
	},
};

/*
 * qpnp_lbc_init() - register spmi driver for qpnp-chg
 */
static int __init qpnp_lbc_init(void)
{
	return spmi_driver_register(&qpnp_lbc_driver);
}
module_init(qpnp_lbc_init);

static void __exit qpnp_lbc_exit(void)
{
	spmi_driver_unregister(&qpnp_lbc_driver);
}
module_exit(qpnp_lbc_exit);

MODULE_DESCRIPTION("QPNP Linear charger driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" QPNP_CHARGER_DEV_NAME);
