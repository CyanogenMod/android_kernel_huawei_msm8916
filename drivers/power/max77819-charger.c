/*
 * Maxim MAX77819 Charger Driver
 *
 * Copyright (C) 2013 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//#define DEBUG
//#define VERBOSE_DEBUG
#define log_level  1
#define log_worker 1 // open time work of printing kernel log

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
//#include <linux/qpnp/qpnp-adc.h>
#include <linux/of_batterydata.h>

#include <linux/power_supply.h>
#include <linux/mfd/max77819.h>
#include <linux/mfd/max77819-charger.h>
#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#endif
#define DRIVER_DESC    "MAX77819 Charger Driver"
#define DRIVER_NAME    MAX77819_CHARGER_NAME
#define DRIVER_VERSION MAX77819_DRIVER_VERSION".3-rc"
#define DRIVER_AUTHOR  "Gyungoh Yoo <jack.yoo@maximintegrated.com>"

#define IRQ_WORK_DELAY              0
#define IRQ_WORK_INTERVAL           msecs_to_jiffies(5000)
#define LOG_WORK_INTERVAL           msecs_to_jiffies(5000)
#define POWR_SUPPLY_BATTERY_NAME   "battery"
#define POWR_SUPPLY_FGAUGE_NAME    "max17048_fgauge"
#define POWR_SUPPLY_USB_NAME       "usb"

#define TIME_UNIT_SECOND            1000
#define TIME_UNIT_MINUTE            (60 * TIME_UNIT_SECOND)
#define TIME_UNIT_HOUR              (60 * TIME_UNIT_MINUTE)

#define MAXIM77819_TIMER_TIMEOUT    (1 * TIME_UNIT_SECOND) /* 1s */
#define TEMP_PROTECTING_WORK_TIME   (5 * TIME_UNIT_SECOND)
#define TEMP_PROTECTING_FORCE_TIME  (5 * TIME_UNIT_MINUTE) /* every 5 mins to force temp protecting */

#define BATTERY_BAD_TEMPERATURE     (-400)
#define BATTERY_ALARM_TEMPERATURE   (600)
#define BATTERY_FULL_CAPACITY       100
#define RESUME_CHARGE_CAPACTIY      99
#define RESUME_CHARGE_WORK_TIME     (1 * TIME_UNIT_SECOND)
#define RESUME_CHG_VOLTAGE_BUFF     50000 /* 0.05v */

#define CHG_TOPOFF_CHECK_TIEM       (10 * TIME_UNIT_SECOND)  /* 10s */
#define CHG_TOPOFF_SOC_TIME         30  /* 30 minutes */
#define CHG_TOPOFF_VOLTAGE_TIME     1   /* 1 min */
#define CHG_TOPOFF_SOC_TIME_MAX     60  /* 60 minutes */
#define CHG_TOPOFF_LOOP_TIME        (2 * TIME_UNIT_SECOND)  /* 2 second */
#define CHG_TOPOFF_LOOP_VOLT_TIME   (1 * TIME_UNIT_MINUTE)  /* 1 minute */
#define CHG_TOPOFF_BATT_MARGIN_UV   5000  /* 5 mv */

#define CHG_CONT_TIME_MAX_DCP       (10 * TIME_UNIT_HOUR)  /* max time of charging continuously by usb_dcp */
#define CHG_CONT_TIME_MAX_USB       (12 * TIME_UNIT_HOUR)  /* max time of charging continuously by pc_usb */
#define CHG_CONT_TIME_WORK_TIME     (1  * TIME_UNIT_MINUTE)

#define CHG_BATTERY_TRICKLE_UV      3400000 /*uv*/

#define CHG_CURRENT_DCIN_MAX       2000000
#define CHG_CURRENT_DCIN_MIN       100000
#define CHG_CURRENT_DCIN_UNLIMIT   CHG_CURRENT_DCIN_MAX

#define CHG_CURRENT_FCC_MAX         2000000
#define CHG_CURRENT_FCC_MIN         0
#define CHG_CURRENT_FCC_UNLIMIT     CHG_CURRENT_FCC_MAX
#define CHG_CURRENT_TYPE_USB         500000 /* uA, means dcin type is usb */
#define CHG_CURRENT_TYPE_SUSPEND     100000 /* uA, means usb is suspend */
#define CHG_CURRENT_USB_CABLE_MAX    475000 /*uA, dcin type is usb, cable max current */
#define CHG_CURRENT_USB_IBATT_MAX    550000 /*uA, dcin type is usb, ibatt input max current */
#define CHG_CURRENT_USB_LIMIT_MIN    100000 /*uA, dcin susppend, cable max current */
#define CHG_CURRENT_AICL_STEP        50000  /*uA, aicl step to adjust current, must >= 50mA */
#define CHG_CURRENT_AICL_WORK_TIME   50     /*0.05s, must > 16ms hardware aicl time */
#define CHG_CURRENT_HOT_LIMIT_MAX    1500000 /* uA */
#define AICL_CURRENT_DCLIMIT_MIN      400000 /* uA, the min dclimit current, allow aicl to turn down */
#define AICL_CURRENT_CHGCC_MIN        500000 /* uA, the sys current, allow aicl to turn down */
#define AICL_MODE_FALL_WORK_TIME      (1 * TIME_UNIT_SECOND) /* 1s */
#define AICL_MODE_RISE_WORK_TIME      (60* TIME_UNIT_SECOND) /* 60s */
#define AICL_MODE_COUNT_NUM_MAX       3

#define CHG_IRQS_MONITOR_WORK_TIME    (1 * TIME_UNIT_SECOND)

#define VERIFICATION_UNLOCK         0

/* Register map */
#define CHGINT                      0x30
#define CHGINTM1                    0x31
#define CHGINT1_AICLOTG             BIT (7)
#define CHGINT1_TOPOFF              BIT (6)
#define CHGINT1_OVP                 BIT (5)
#define CHGINT1_DC_UVP              BIT (4)
#define CHGINT1_CHG                 BIT (3)
#define CHGINT1_BAT                 BIT (2)
#define CHGINT1_THM                 BIT (1)
#define CHG_STAT                    0x32
#define DC_BATT_DTLS                0x33
#define DC_BATT_DTLS_DC_AICL        BIT (7)
#define DC_BATT_DTLS_DC_I           BIT (6)
#define DC_BATT_DTLS_DC_OVP         BIT (5)
#define DC_BATT_DTLS_DC_UVP         BIT (4)
#define DC_BATT_DTLS_BAT_DTLS       BITS(3,2)
#define DC_BATT_DTLS_BATDET_DTLS    BITS(1,0)
#define CHG_DTLS                    0x34
#define CHG_DTLS_THM_DTLS           BITS(7,5)
#define CHG_DTLS_TOPOFF_DTLS        BIT (4)
#define CHG_DTLS_CHG_DTLS           BITS(3,0)
#define BAT2SYS_DTLS                0x35
#define BAT2SOC_CTL                 0x36
#define CHGCTL1                     0x37
#define CHGCTL1_SFO_DEBOUNCE_TMR    BITS(7,6)
#define CHGCTL1_SFO_DEBOUNCE_EN     BIT (5)
#define CHGCTL1_THM_DIS             BIT (4)
#define CHGCTL1_JEITA_EN            BIT (3)
#define CHGCTL1_BUCK_EN             BIT (2)
#define CHGCTL1_CHGPROT             BITS(1,0)
#define FCHGCRNT                    0x38
#define FCHGCRNT_FCHGTIME           BITS(7,5)
#define FCHGCRNT_CHGCC              BITS(4,0)
#define TOPOFF                      0x39
#define TOPOFF_TOPOFFTIME           BITS(7,5)
#define TOPOFF_IFST2P8              BIT (4)
#define TOPOFF_ITOPOFF              BITS(2,0)
#define BATREG                      0x3A
#define BATREG_REGTEMP              BITS(7,6)
#define BATREG_CHGRSTRT             BIT (5)
#define BATREG_MBATREG              BITS(4,1)
#define BATREG_VICHG_GAIN           BIT (0)
#define DCCRNT                      0x3B
#define DCCRNT_DCILMT               BITS(5,0)
#define AICLCNTL                    0x3C
#define AICLCNTL_AICL_RESET         BIT (5)
#define AICLCNTL_AICL               BITS(4,1)
#define AICLCNTL_DCMON_DIS          BIT (0)
#define RBOOST_CTL1                 0x3D
#define RBOOST_CTL1_RBOOSTEN        BIT(0)
#define CHGCTL2                     0x3E
#define CHGCTL2_DCILIM_EN           BIT (7)
#define CHGCTL2_PREQCUR             BITS(6,5)
#define CHGCTL2_CEN                 BIT (4)
#define CHGCTL2_QBATEN              BIT (3)
#define CHGCTL2_VSYSREG             BITS(2,0)
#define BATDET                      0x3F
#define USBCHGCTL                   0x40
#define USBCHGCTL_CHGTIMER          BIT (7)
#define MBATREGMAX                  0x41
#define CHGCCMAX                    0x42
#define RBOOST_CTL2                 0x43
#define RBOOST_CTL2_VBYPSET         BITS(6,0) 
#define CHGINT2                     0x44
#define CHGINTMSK2                  0x45
#define CHGINT2_DC_V                BIT (7)
#define CHGINT2_CHG_WDT             BIT (4)
#define CHGINT2_CHG_WDT_WRN         BIT (0)
#define CHG_WDTC                    0x46
#define CHG_WDT_CTL                 0x47
#define CHG_WDT_DTLS                0x48

#undef  log_fmt
#define log_fmt(format) \
        "Maxim:%s:%d: " format, __func__, __LINE__
#define loop_schedule_delayed_work(ptr_work, delaytime) \
        if (likely(!delayed_work_pending(ptr_work))){\
            schedule_delayed_work(ptr_work, delaytime);\
        }

/*recharge ,otg enable,used by outside module--jelphi*/
static struct max77819_charger *global_Charger;
/* remove extern void max77819_charger_rechg(bool) */
void notify_max77819_to_control_otg(bool enable);
int get_max77819_boost_mode(void);
//int g_charger_status = POWER_SUPPLY_STATUS_UNKNOWN;
/* remove g_capacity */
extern int maxim_get_battery_id_uv(void);
extern int is_usb_chg_exist(void);
static int max77819_charger_buck_mode(struct max77819_charger *me);
static int max77819_charger_resume_charging(struct max77819_charger* me);
static int max77819_charger_enable_control(struct max77819_charger *me,bool enable);
static void max77819_charger_set_current_by_dcin_type(struct max77819_charger* me);

enum max77819_chg_timer_state{
    CHG_TIMER_NONE,                      /* no timer works */
    CHG_TIMER_RESET,                     /* timer reset, to recount */
    CHG_TIMER_OVER_STOP,                 /* time over to stop charge */
    CHG_TIMER_COUNTING,                  /* timer in counting */
};

enum max77819_chg_aicl_state{
    CHG_AICL_NONE = 0,                   /* no need to do aicl */
    CHG_AICL_NEED_CHECK,                 /* aicl will need to check */
    CHG_AICL_WORKING_FALL,               /* aicl working & adjusting, fall current */
    CHG_AICL_WORKING_RISE,               /* aicl working & adjusting, rise current */
};

enum max77819_chg_monitor_irqs{
    CHG_IRQ_NONE = 0,
    CHG_IRQ_DC_UVP = 0x10,
};

struct max77819_charger {
    struct mutex                           lock;
    struct max77819_dev                   *chip;
    struct max77819_io                    *io;
    struct device                         *dev;
    struct kobject                        *kobj;
    struct attribute_group                *attr_grp;
    struct max77819_charger_platform_data *pdata;
    // remove irq for dcin interrupt
    struct delayed_work                    irq_work;
    struct delayed_work                    log_work;
    // remove struct delayed_work topoff_work;
    struct power_supply                    psy;
    struct power_supply                   *psy_this;
    struct power_supply                   *psy_ext;
    struct power_supply                   *psy_coop; /* cooperating charger */
    struct power_supply                   *psy_bms;
    struct power_supply                   *psy_qcom_charger;
    bool                                   dev_enabled;
    bool                                   dev_initialized;
    bool                                   dev_otg_mode;
    int                                    current_limit_volatile;
    int                                    current_limit_permanent;
    int                                    charge_current_volatile;
    int                                    charge_current_permanent;
    int                                    present;
    int                                    health;
    int                                    status;
    int                                    charge_type;
    int                                    soc;                        /* current battery capacity */
    bool                                   bfactory_daig_chg;
    int                                    resuming_charging;
    bool                                   bterm_chg_by_soc;
    bool                                   bterm_chg_by_volt;          /*whether in terminate charging by battery voltage */
    bool                                   bterm_chg_pause;           /* be pause charge to  get battery real voltage */
    bool                                   btemp_chg_protecting;      /* system is proctecting charge by temp */
    u8                                     irqs_reg;
    enum max77819_chg_timer_state          stop_chg_time_out;        /* timer state, to stop charge by time out */
    enum max77819_chg_aicl_state           aicl_work_state;
    struct max77819_temp_control_info*     ptemp_ctrl_info;
    struct delayed_work                    timer_work;
    struct delayed_work                    aicl_work;
};

#define __lock(_me)    mutex_lock(&(_me)->lock)
#define __unlock(_me)  mutex_unlock(&(_me)->lock)

static struct max77819_temp_control_info g_temp_ctrl_info = {
    .cold_bat_degree = 0,
    .cool_bat_degree = 100,
    .imaxua_cool_bat = 900000,
    .vmaxuv_cool_bat = 4350000,
    .warm_bat_degree = 450,
    .imaxua_warm_bat = 800000,
    .vmaxuv_warm_bat = 4100000,
    .hot_bat_degree = 600,
    .sys_limit_current = 1500000,  /* default value is the max charging current */
};

static struct max77819_charger_platform_data g_platform_data = {
    .psy_name = "ac",
    .ext_psy_name = POWR_SUPPLY_USB_NAME,

    .supplied_to = NULL,
    .num_supplicants = 0,

    .chg_dcin_current_max = 1100000,           /* in uA; 0.00A ~ 1.80A */
    .chg_fast_current_max = 1200000,           /* in uA; 0.00A ~ 1.80A */
    .charge_termination_voltage = 4350000,     /* in uV, 4.10V ~ 4.35V */
    .topoff_timer = CHG_TOPOFF_SOC_TIME,       /* in min, 0min ~ 60min, infinite */
    .topoff_current = 300000,                  /* in uA, 50mA ~ 400mA */
    .charge_restart_threshold = 150000,        /* in uV, 100mV ~ 150mV */

    /* Co-operating charger */
    .enable_coop = false,
    .coop_psy_name = "",

    /* Temperature regulation */
    .enable_thermistor = false,

    /* AICL control */
    .enable_aicl = true,
    .aicl_detection_voltage = 4400000,        /* in uV, 3.9V ~ 4.8V  */
    .aicl_reset_threshold = 100000,           /* in uV, 100mV or 200mV */
};

enum {
    BATDET_DTLS_CONTACT_BREAK       = 0b00,
    BATDET_DTLS_BATTERY_DETECTED_01 = 0b01,
    BATDET_DTLS_BATTERY_DETECTED_10 = 0b10,
    BATDET_DTLS_BATTERY_REMOVED     = 0b11,
};

static char *max77819_charger_batdet_details[] = {
    [BATDET_DTLS_CONTACT_BREAK]       = "contact break",
    [BATDET_DTLS_BATTERY_DETECTED_01] = "battery detected (01)",
    [BATDET_DTLS_BATTERY_DETECTED_10] = "battery detected (10)",
    [BATDET_DTLS_BATTERY_REMOVED]     = "battery removed",
};

enum {
    DC_UVP_INVALID = 0,
    DC_UVP_VALID   = 1,
};

static char *max77819_charger_dcuvp_details[] = {
    [DC_UVP_INVALID] = "VDC is invalid; VDC < VDC_UVLO",
    [DC_UVP_VALID]   = "VDC is valid; VDC > VDC_UVLO",
};

enum {
    DC_OVP_VALID   = 0,
    DC_OVP_INVALID = 1,
};

static char *max77819_charger_dcovp_details[] = {
    [DC_OVP_VALID]   = "VDC is valid; VDC < VDC_OVLO",
    [DC_OVP_INVALID] = "VDC is invalid; VDC > VDC_OVLO",
};

enum {
    DC_I_VALID   = 0,
    DC_I_INVALID = 1,
};

static char *max77819_charger_dci_details[] = {
    [DC_I_VALID]   = "IDC is valid; IDC < DCILMT",
    [DC_I_INVALID] = "IDC is invalid; IDC > DCILMT",
};

enum {
    DC_AICL_OK  = 0,
    DC_AICL_NOK = 1,
};

static char *max77819_charger_aicl_details[] = {
    [DC_AICL_OK]  = "VDC > AICL threshold",
    [DC_AICL_NOK] = "VDC < AICL threshold",
};

enum {
    BAT_DTLS_UVP     = 0b00,
    BAT_DTLS_TIMEOUT = 0b01,
    BAT_DTLS_OK      = 0b10,
    BAT_DTLS_OVP     = 0b11,
};

static char *max77819_charger_bat_details[] = {
    [BAT_DTLS_UVP]     = "battery voltage < 2.1V",
    [BAT_DTLS_TIMEOUT] = "timer fault",
    [BAT_DTLS_OK]      = "battery okay",
    [BAT_DTLS_OVP]     = "battery overvoltage",
};

enum {
    CHG_DTLS_DEAD_BATTERY     = 0b0000,
    CHG_DTLS_PRECHARGE        = 0b0001,
    CHG_DTLS_FASTCHARGE_CC    = 0b0010,
    CHG_DTLS_FASTCHARGE_CV    = 0b0011,
    CHG_DTLS_TOPOFF           = 0b0100,
    CHG_DTLS_DONE             = 0b0101,
    CHG_DTLS_TIMER_FAULT      = 0b0110,
    CHG_DTLS_TEMP_SUSPEND     = 0b0111,
    CHG_DTLS_OFF              = 0b1000,
    CHG_DTLS_THM_LOOP         = 0b1001,
    CHG_DTLS_TEMP_SHUTDOWN    = 0b1010,
    CHG_DTLS_BUCK             = 0b1011,
    CHG_DTLS_OTG_OVER_CURRENT = 0b1100,
    CHG_DTLS_USB_SUSPEND      = 0b1101,
};

static char *max77819_charger_chg_details[] = {
    [CHG_DTLS_DEAD_BATTERY] =
        "charger is in dead-battery region",
    [CHG_DTLS_PRECHARGE] =
        "charger is in precharge mode",
    [CHG_DTLS_FASTCHARGE_CC] =
        "charger is in fast-charge constant current mode",
    [CHG_DTLS_FASTCHARGE_CV] =
        "charger is in fast-charge constant voltage mode",
    [CHG_DTLS_TOPOFF] =
        "charger is in top-off mode",
    [CHG_DTLS_DONE] =
        "charger is in done mode",
    [CHG_DTLS_TIMER_FAULT] =
        "charger is in timer fault mode",
    [CHG_DTLS_TEMP_SUSPEND] =
        "charger is in temperature suspend mode",
    [CHG_DTLS_OFF] =
        "buck off, charger off",
    [CHG_DTLS_THM_LOOP] =
        "charger is operating with its thermal loop active",
    [CHG_DTLS_TEMP_SHUTDOWN] =
        "charger is off and junction temperature is > TSHDN",
    [CHG_DTLS_BUCK] =
        "buck on, charger off",
    [CHG_DTLS_OTG_OVER_CURRENT] =
        "charger OTG current limit is exceeded longer than debounce time",
    [CHG_DTLS_USB_SUSPEND] =
        "USB suspend",
};

enum {
    TOPOFF_NOT_REACHED = 0,
    TOPOFF_REACHED     = 1,
};

static char *max77819_charger_topoff_details[] = {
    [TOPOFF_NOT_REACHED] = "topoff is not reached",
    [TOPOFF_REACHED]     = "topoff is reached",
};

enum {
    THM_DTLS_LOW_TEMP_SUSPEND   = 0b001,
    THM_DTLS_LOW_TEMP_CHARGING  = 0b010,
    THM_DTLS_STD_TEMP_CHARGING  = 0b011,
    THM_DTLS_HIGH_TEMP_CHARGING = 0b100,
    THM_DTLS_HIGH_TEMP_SUSPEND  = 0b101,
};

static char *max77819_charger_thm_details[] = {
    [THM_DTLS_LOW_TEMP_SUSPEND]   = "cold; T < T1",
    [THM_DTLS_LOW_TEMP_CHARGING]  = "cool; T1 < T < T2",
    [THM_DTLS_STD_TEMP_CHARGING]  = "normal; T2 < T < T3",
    [THM_DTLS_HIGH_TEMP_CHARGING] = "warm; T3 < T < T4",
    [THM_DTLS_HIGH_TEMP_SUSPEND]  = "hot; T4 < T",
};

#define CHGINT1 CHGINT
#define max77819_charger_read_irq_status(_me, _irq_reg) \
        ({\
            u8 __irq_current = 0;\
            int __rc = max77819_read((_me)->io, _irq_reg, &__irq_current);\
            if (unlikely(IS_ERR_VALUE(__rc))) {\
                log_err(#_irq_reg" read error [%d]\n", __rc);\
                __irq_current = 0;\
            }\
            __irq_current;\
        })

enum {
    CFG_CHGPROT = 0,
    CFG_SFO_DEBOUNCE_TMR,
    CFG_SFO_DEBOUNCE_EN,
    CFG_THM_DIS,
    CFG_JEITA_EN,
    CFG_BUCK_EN,
    CFG_DCILIM_EN,
    CFG_PREQCUR,
    CFG_CEN,
    CFG_QBATEN,
    CFG_VSYSREG,
    CFG_DCILMT,
    CFG_FCHGTIME,
    CFG_CHGCC,
    CFG_AICL_RESET,
    CFG_AICL,
    CFG_DCMON_DIS,
    CFG_MBATREG,
    CFG_CHGRSTRT,
    CFG_TOPOFFTIME,
    CFG_ITOPOFF,
    CFG_RBOOSTEN,
    CFG_CHGTIMER,
};

static struct max77819_bitdesc max77819_charger_cfg_bitdesc[] = {
    #define CFG_BITDESC(_cfg_bit, _cfg_reg) \
            [CFG_##_cfg_bit] = MAX77819_BITDESC(_cfg_reg, _cfg_reg##_##_cfg_bit)

    CFG_BITDESC(CHGPROT         , CHGCTL1 ),
    CFG_BITDESC(SFO_DEBOUNCE_TMR, CHGCTL1 ),
    CFG_BITDESC(SFO_DEBOUNCE_EN , CHGCTL1 ),
    CFG_BITDESC(THM_DIS         , CHGCTL1 ),
    CFG_BITDESC(JEITA_EN        , CHGCTL1 ),
    CFG_BITDESC(BUCK_EN         , CHGCTL1 ),
    CFG_BITDESC(DCILIM_EN       , CHGCTL2 ),
    CFG_BITDESC(PREQCUR         , CHGCTL2 ),
    CFG_BITDESC(CEN             , CHGCTL2 ),
    CFG_BITDESC(QBATEN          , CHGCTL2 ),
    CFG_BITDESC(VSYSREG         , CHGCTL2 ),
    CFG_BITDESC(DCILMT          , DCCRNT  ),
    CFG_BITDESC(FCHGTIME        , FCHGCRNT),
    CFG_BITDESC(CHGCC           , FCHGCRNT),
    CFG_BITDESC(AICL_RESET      , AICLCNTL),
    CFG_BITDESC(AICL            , AICLCNTL),
    CFG_BITDESC(DCMON_DIS       , AICLCNTL),
    CFG_BITDESC(MBATREG         , BATREG  ),
    CFG_BITDESC(CHGRSTRT        , BATREG  ),
    CFG_BITDESC(TOPOFFTIME      , TOPOFF  ),
    CFG_BITDESC(ITOPOFF         , TOPOFF  ),
    CFG_BITDESC(RBOOSTEN        , RBOOST_CTL1),
    CFG_BITDESC(CHGTIMER        , USBCHGCTL),
};
#define __cfg_bitdesc(_cfg) (&max77819_charger_cfg_bitdesc[CFG_##_cfg])

#define PROTCMD_UNLOCK  3
#define PROTCMD_LOCK    0

/* Use another mothed to reduce the power consumption of FTM mode. */

#ifdef CONFIG_HUAWEI_DSM
#define MAX_COUNT	3
extern struct dsm_client *charger_dclient;
extern struct qpnp_lbc_chip *g_lbc_chip;
extern int dump_registers_and_adc(struct dsm_client *dclient, struct qpnp_lbc_chip *chip, int type);
#endif

static __always_inline int max77819_charger_unlock (struct max77819_charger *me)
{
    int rc;

    rc = max77819_write_bitdesc(me->io, __cfg_bitdesc(CHGPROT), PROTCMD_UNLOCK);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("failed to unlock [%d]\n", rc);
        goto out;
    }

#if VERIFICATION_UNLOCK
    do {
        u8 chgprot = 0;

        rc = max77819_read_bitdesc(me->io, __cfg_bitdesc(CHGPROT), &chgprot);
        if (unlikely(IS_ERR_VALUE(rc) || chgprot != PROTCMD_UNLOCK)) {
            log_err("access denied - CHGPROT %X [%d]\n", chgprot, rc);
            rc = -EACCES;
            goto out;
        }
    } while (0);
#endif /* VERIFICATION_UNLOCK */

out:
    return rc;
}

static __always_inline int max77819_charger_lock (struct max77819_charger *me)
{
    int rc;

    rc = max77819_write_bitdesc(me->io, __cfg_bitdesc(CHGPROT), PROTCMD_LOCK);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("failed to lock [%d]\n", rc);
    }

    return rc;
}

#define max77819_charger_read_config(_me, _cfg, _val_ptr) \
        ({\
            int __rc = max77819_read_bitdesc((_me)->io, __cfg_bitdesc(_cfg),\
                _val_ptr);\
            if (unlikely(IS_ERR_VALUE(__rc))) {\
                log_err("read config "#_cfg" error [%d]\n", __rc);\
            } else {\
                log_vdbg("read config "#_cfg": %Xh\n", *(_val_ptr));\
            }\
            __rc;\
        })
#define max77819_charger_write_config(_me, _cfg, _val) \
        ({\
            int __rc = max77819_charger_unlock(_me);\
            if (likely(!IS_ERR_VALUE(__rc))) {\
                __rc = max77819_write_bitdesc((_me)->io, __cfg_bitdesc(_cfg),\
                    _val);\
                if (unlikely(IS_ERR_VALUE(__rc))) {\
                    log_err("write config "#_cfg" error [%d]\n", __rc);\
                } else {\
                    log_vdbg("write config "#_cfg": %Xh\n", _val);\
                }\
                max77819_charger_lock(_me);\
            }\
            __rc;\
        })

#define max77819_charger_get_ext_property(pwr_spy, pwr_psp, ptr_val)\
({\
    int rc = -ENXIO;\
    if ((pwr_spy) && (pwr_spy)->get_property) {\
        rc = (pwr_spy)->get_property((pwr_spy), (pwr_psp), (ptr_val));\
    }\
    if (unlikely(IS_ERR_VALUE(rc))) {\
        (ptr_val)->intval = 0;\
    }\
    rc;\
})

#define max77819_charger_temp_protect_max_ua(me) \
({ \
    int iset_ua = 0; \
    switch ((me)->health) \
    { \
    case POWER_SUPPLY_HEALTH_COOL: \
        iset_ua = (me)->ptemp_ctrl_info->imaxua_cool_bat; \
        break; \
    case POWER_SUPPLY_HEALTH_WARM: \
        iset_ua = (me)->ptemp_ctrl_info->imaxua_warm_bat; \
        break; \
    default: \
        iset_ua = (me)->charge_current_permanent; \
        break; \
    } \
    iset_ua; \
})

#define max77819_charger_allow_to_aicl_fall(me)\
({\
    bool rev = true;\
    if (AICL_CURRENT_CHGCC_MIN > \
        ((me)->charge_current_volatile - CHG_CURRENT_AICL_STEP)) \
    {\
        log_vdbg("the chgcc is %d, no allow to aicl fall !\n", \
                (me)->charge_current_volatile); \
        rev = false; \
    }\
    rev;\
})

#define max77819_charger_allow_to_aicl_rise(me)\
({\
    bool rev = true;\
    if ((me->ptemp_ctrl_info->sys_limit_current < \
         ((me)->charge_current_volatile + CHG_CURRENT_AICL_STEP)) || \
        (max77819_charger_temp_protect_max_ua(me) < \
         ((me)->charge_current_volatile + CHG_CURRENT_AICL_STEP))) \
    {\
        log_vdbg("the chgcc is %d, no allow to aicl rise \n", \
                (me)->charge_current_volatile); \
        rev = false; \
    }\
    rev;\
})

static __inline bool max77819_charger_in_aicl(struct max77819_charger* me)
{
#define MAX77819_IS_IN_AICL_MODE(reg) (!!(reg & 0x80))
    int rc;
    u8 reg_aicl = 0;

    rc = max77819_read(me->io, CHG_STAT, &reg_aicl);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        log_err("get aicl status reg error, rc = %d !! \n", rc);
        return false;
    }

    return (MAX77819_IS_IN_AICL_MODE(reg_aicl));
}

static int max77819_charger_set_termination_voltage(struct max77819_charger *me, int uv)
{
#define MAXIM_TERM_VOLTAGE_THRESHOLD 3700000
#define MAXIM_TERM_VOLTAGE_STEP      50000
    int rc, val;
    val = (uv < MAXIM_TERM_VOLTAGE_THRESHOLD ? 0x00 :
           (int)DIV_ROUND_UP((uv - MAXIM_TERM_VOLTAGE_THRESHOLD), MAXIM_TERM_VOLTAGE_STEP))
          + 0x01;
    rc = max77819_charger_write_config(me, MBATREG, val);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_dbg("set battery termination voltage failed! rc = %d \n", rc);
    }
    return rc;
}

static __inline int max77819_charger_get_dcilmt (struct max77819_charger *me,
    int *uA)
{
    int rc;
    u8 dcilmt = 0;

    rc = max77819_charger_read_config(me, DCILMT, &dcilmt);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    if (unlikely(dcilmt >= 0x3F)) {
        *uA = CHG_CURRENT_DCIN_UNLIMIT;
        log_vdbg("<get_dcilmt> no limit\n");
        goto out;
    }

    *uA = dcilmt < 0x03 ? 100000 :
          dcilmt < 0x35 ? (int)(dcilmt - 0x03) * 25000 +  275000 :
                          (int)(dcilmt - 0x35) * 37500 + 1537500;
    log_vdbg("<get_dcilmt> %Xh -> %duA\n", dcilmt, *uA);

out:
    return rc;
}

static int max77819_charger_set_dcilmt (struct max77819_charger *me, int uA)
{
    u8 dcilmt;

    /* Use another mothed to reduce the power consumption of FTM mode. */

    if (unlikely(uA == CHG_CURRENT_DCIN_UNLIMIT)) {
        dcilmt = 0x3F;
        log_vdbg("<set_dcilmt> no limit\n");
        goto out;
    }

    dcilmt = uA <  275000 ? 0x00 :
             uA < 1537500 ? DIV_ROUND_UP(uA -  275000, 25000) + 0x03 :
             uA < 1875000 ? DIV_ROUND_UP(uA - 1537500, 37500) + 0x35 : 0x3F;
    log_vdbg("<set_dcilmt> %duA -> %Xh\n", uA, dcilmt);

out:
    return max77819_charger_write_config(me, DCILMT, dcilmt);
}

static __inline int max77819_charger_get_enable (struct max77819_charger *me,
    int *en)
{
    int rc;
    u8 cen = 0;

    rc = max77819_charger_read_config(me, CEN, &cen);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    *en = !!cen;
    log_vdbg("<get_enable> %s\n", *en ? "enabled" : "disabled");

out:
    return rc;
}

static int max77819_charger_set_enable (struct max77819_charger *me, int en)
{
    log_dbg("<set_enable> %s\n", en ? "enabling" : "disabling");
    return max77819_charger_write_config(me, CEN, !!en);
}

static __inline int max77819_charger_get_chgcc (struct max77819_charger *me,
    int *uA)
{
    int rc;
    u8 dcilmt = 0;

    rc = max77819_charger_read_config(me, CHGCC, &dcilmt);
    if (unlikely(IS_ERR_VALUE(rc))) {
#ifdef CONFIG_HUAWEI_DSM
        /* if maxim charger i2c read failed, record this log, and notify to the dsm server*/
        if(!dsm_client_ocuppy(charger_dclient)){
               dsm_client_record(charger_dclient, "maxim charger i2c read failed!\n");
               dsm_client_notify(charger_dclient, DSM_MAXIM_I2C_ERROR);
        }
#endif
        goto out;
    }

    *uA = dcilmt < 0x01 ? 0 :
          dcilmt < 0x1C ? (int)(dcilmt - 0x01) * 50000 +  250000 :
          dcilmt < 0x1F ? (int)(dcilmt - 0x1C) * 67000 + 1800000 : 2000000;
    log_vdbg("<get_chgcc> %Xh -> %duA\n", dcilmt, *uA);

out:
    return rc;
}

static int max77819_charger_set_chgcc (struct max77819_charger *me, int uA)
{
    u8 chgcc;

    chgcc = uA <  250000 ? 0x00 :
            uA < 1620000 ? DIV_ROUND_UP(uA -  250000, 50000) + 0x01 :
            uA < 1800000 ? DIV_ROUND_UP(uA - 1800000, 67000) + 0x1C : 0x1F;
    log_vdbg("<set_chgcc> %duA -> %Xh\n", uA, chgcc);

    return max77819_charger_write_config(me, CHGCC, chgcc);
}

static int max77819_charger_set_fchg_time(struct max77819_charger *me, int hour)
{
    u8 fchgt;

    fchgt = hour <= 3 ? 0x0 :
            hour < 10 ? (hour - 3) : 7;

    log_vdbg("<set_fchg_time>%s: %dh -> 0x%X \n", (fchgt ? "enable" : "disable"), hour, fchgt);
    return max77819_charger_write_config(me, FCHGTIME, fchgt);
}

static int max77819_charger_set_charge_current (struct max77819_charger *me,
    int limit_uA, int charge_uA)
{
    int rc;

    log_vdbg("setting current %duA but limited up to %duA\n", charge_uA, limit_uA);
    limit_uA = min(limit_uA, CHG_CURRENT_DCIN_MAX);
    limit_uA = max(limit_uA, CHG_CURRENT_DCIN_MIN);

    charge_uA = min(charge_uA, CHG_CURRENT_FCC_MAX);
    charge_uA = max(charge_uA, CHG_CURRENT_FCC_MIN);
    /* no need to set charge ua min */
    //charge_uA = min(charge_uA, limit_uA);

    rc = max77819_charger_set_dcilmt(me, limit_uA);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("set dclimt current error, rc = %d !! \n", rc);
    }
    rc = max77819_charger_set_chgcc(me, charge_uA);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("set chgcc current error, rc = %d !! \n", rc);
    }

    if (likely(charge_uA <= CHG_CURRENT_FCC_MIN))
    {
        log_warn("disabling charger, charging current <= %uA !!\n", CHG_CURRENT_FCC_MIN);
        rc = max77819_charger_set_enable(me, false);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("disable charge failed, rc = %d !! \n", rc);
        }
    }

    return rc;
}

static void max77819_charger_close_hardware_timer(struct max77819_charger* me)
{
    int rc;
    /* set 0 hour to close fast charge timer 0x38 */
    rc = max77819_charger_set_fchg_time(me, 0);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        log_err("close fast charge timer failed, rc = %d !!\n", rc);
    }

    /* set register bit value 1 to close charge control timer 0x40 */
    rc = max77819_charger_write_config(me, CHGTIMER, 1);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        log_err("close charge control timer failed, rc = %d !!\n", rc);
    }
}

static bool max77819_charger_present_input (struct max77819_charger *me)
{
    return (1 == is_usb_chg_exist());
}

static bool ma77819_charger_is_otg_mode(struct max77819_charger* me)
{
    return me->dev_otg_mode;
}

bool get_max77819_charger_present(void)
{
    if (!global_Charger)
    {
        pr_info("max77819 chip device not init,do nothing!\n");
        return false;
    }
    return max77819_charger_present_input(global_Charger);
}
EXPORT_SYMBOL(get_max77819_charger_present);
bool get_max77819_charger_timeout(void)
{
    if (!global_Charger)
    {
        log_info("max77819 chip device not init,do nothing!\n");
        return false;
    }
    return (CHG_TIMER_OVER_STOP == global_Charger->stop_chg_time_out);
}
EXPORT_SYMBOL(get_max77819_charger_timeout);
static int max77819_charger_exit_dev (struct max77819_charger *me)
{
    max77819_charger_set_charge_current(me, me->current_limit_permanent, 0);
    max77819_charger_set_enable(me, false);

    me->current_limit_volatile  = me->current_limit_permanent;
    me->charge_current_volatile = me->charge_current_permanent;

    me->bterm_chg_by_volt = false;
    me->bterm_chg_by_soc = false;
    me->bterm_chg_pause = false;
    me->stop_chg_time_out = CHG_TIMER_RESET;
    log_info("charger exit, set charge time flag to reset \n");
    me->aicl_work_state = CHG_AICL_NONE;

    /* remove me->dev_enabled */
    me->dev_initialized = false;
    return 0;
}

#define MAX77819_DUMP_REG_NUM        32
#define MAX77819_DUMP_REG_BASE_ADDR  0x30
#define MAX77819_REG37_ADDR          0x37
static void max77819_charger_dump_registers(struct max77819_charger *me)
{

    static u8 dump_reg[MAX77819_DUMP_REG_NUM] = {0};
    int i, rc;

    rc = max77819_bulk_read(me->io, MAX77819_DUMP_REG_BASE_ADDR,
            dump_reg, MAX77819_DUMP_REG_NUM);
    for (i = 0; i < MAX77819_DUMP_REG_NUM; i++)
    {
        log_dbg("BURST DUMP REG %02Xh VALUE %02Xh [%d]\n", i+0x30, dump_reg[i], rc);
    }
}

#ifdef CONFIG_HUAWEI_DSM
/* dump max77819 main 32 regs*/
void max77819_charger_dump_regs(struct dsm_client *dclient)
{
    u8 dump_reg[MAX77819_DUMP_REG_NUM] = {0};
    int i, rc;

    if (!global_Charger)
    {
        pr_info("max77819 chip device not init,do nothing!\n");
        return;
    }
    dsm_client_record(dclient, "[MAX77819 charger] regs:\n");
    log_vdbg("[MAX77819 charger] regs:\n");
    rc = max77819_bulk_read(global_Charger->io, MAX77819_DUMP_REG_BASE_ADDR,
            dump_reg, MAX77819_DUMP_REG_NUM);
    if(rc){
        pr_info("read max77819 charger regs failed: rc = %d", rc);
        return;
    }
    for (i = 0; i < MAX77819_DUMP_REG_NUM; i++)
    {
        dsm_client_record(dclient, "0x%x, 0x%x\n", i+0x30, dump_reg[i]);
        log_vdbg("%02Xh, %02Xh\n", i+0x30, dump_reg[i]);
    }
}
EXPORT_SYMBOL(max77819_charger_dump_regs);

/* monitor max77819 charger DC_BATT_DTLS and CHG_DTLS regs*/
static int monitor_max77819_charger_regs (struct max77819_charger *me)
{
    int rc = 0;
    u8 dc_batt_dtls, chg_dtls;
    u8 batdet, bat, dcuvp, dcovp, chg;
    /* total 10 errors we monitors, when one of them happens */
    /* more than 5 times, we will report it to dsm */
    static int error_count[11] = {0};

    rc = max77819_read(me->io, DC_BATT_DTLS, &dc_batt_dtls);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("DC_BATT_DTLS read error [%d]\n", rc);
        goto out;
    }

    rc = max77819_read(me->io, CHG_DTLS, &chg_dtls);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("CHG_DTLS read error [%d]\n", rc);
        goto out;
    }

    log_vdbg("DC_BATT_DTLS %Xh CHG_DTLS %Xh\n", dc_batt_dtls, chg_dtls);

    batdet = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_BATDET_DTLS);
    log_vdbg("BATDET %s\n", max77819_charger_batdet_details[batdet]);

    bat = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_BAT_DTLS);
    log_vdbg("BAT %s\n", max77819_charger_bat_details[bat]);

    dcuvp = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_DC_UVP);
    log_vdbg("DC_UVP %s\n", max77819_charger_dcuvp_details[dcuvp]);

    dcovp = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_DC_OVP);
    log_vdbg("DC_OVP %s\n", max77819_charger_dcovp_details[dcovp]);

    /* Remove AICL_NOK, as it is not stable for max77819 charger chip */

    chg = BITS_GET(chg_dtls, CHG_DTLS_CHG_DTLS);
    log_vdbg("CHG %s\n", max77819_charger_chg_details[chg]);

    if(BATDET_DTLS_CONTACT_BREAK == batdet){
        if(MAX_COUNT <= (error_count[0]++)){
            error_count[0] = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_BAT_CONTACT_BREAK);
        }else{
            log_vdbg("error_count[0] = %d\n", error_count[0]);
        }
    }else{
        error_count[0] = 0;
    }

    if(BATDET_DTLS_BATTERY_REMOVED == batdet){
        if(MAX_COUNT <= (error_count[1]++)){
            error_count[1] = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_BATTERY_REMOVED);
        }else{
            log_vdbg("error_count[1] = %d\n", error_count[1]);
        }
    }else{
        error_count[1] = 0;
    }

    if(BAT_DTLS_UVP == bat){
        if(MAX_COUNT <= (error_count[2]++)){
            error_count[2] = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_BATT_UVP);
        }else{
            log_vdbg("error_count[2] = %d\n", error_count[2]);
        }
    }else{
        error_count[2] = 0;
    }

    if(BAT_DTLS_TIMEOUT == bat){
        if(MAX_COUNT <= (error_count[3]++)){
            error_count[3] = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_TIMER_FAULT);
        }else{
            log_vdbg("error_count[3] = %d\n", error_count[3]);
        }
    }else{
        error_count[3] = 0;
    }

    if(BAT_DTLS_OVP == bat){
        if(MAX_COUNT <= (error_count[4]++)){
            error_count[4] = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_VBAT_OVP);
        }else{
            log_vdbg("error_count[4] = %d\n", error_count[4]);
        }
    }else{
        error_count[4] = 0;
    }

    if (me->dev_enabled && me->present && !ma77819_charger_is_otg_mode(me)){
        if(DC_UVP_INVALID == dcuvp){
            if(MAX_COUNT <= (error_count[5]++)){
                error_count[5] = 0;
                dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_DC_UVP);
            }else{
                log_vdbg("error_count[5] = %d\n", error_count[5]);
            }
        }else{
            error_count[5] = 0;
        }

        if(DC_OVP_INVALID == dcovp){
            if(MAX_COUNT <= (error_count[6]++)){
                error_count[6] = 0;
                dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_DC_OVP);
            }else{
                log_vdbg("error_count[6] = %d\n", error_count[6]);
            }
        }else{
            error_count[6] = 0;
        }
    }
    /* Remove AICL_NOK, as it is not stable for max77819 charger chip */
    if(CHG_DTLS_TEMP_SHUTDOWN == chg){
        if(MAX_COUNT <= (error_count[8]++)){
            error_count[8] = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_TEMP_SHUTDOWN);
        }else{
            log_vdbg("error_count[1] = %d\n", error_count[8]);
        }
    }else{
        error_count[8] = 0;
    }

    if(CHG_DTLS_OTG_OVER_CURRENT == chg){
        if(MAX_COUNT <= (error_count[9]++)){
            error_count[9] = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_OTG_OVERCURRENT);
        }else{
            log_vdbg("error_count[9] = %d\n", error_count[9]);
        }
    }else{
        error_count[9] = 0;
    }

    if(CHG_DTLS_USB_SUSPEND == chg){
        if(MAX_COUNT <= (error_count[10]++)){
            error_count[10] = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_USB_SUSPEND);
        }else{
            log_vdbg("error_count[10] = %d\n", error_count[10]);
        }
    }else{
        error_count[10] = 0;
    }

out:
    return rc;
}
#endif

static int max77819_charger_init_dev (struct max77819_charger *me)
{
    struct max77819_charger_platform_data *pdata = me->pdata;
    int rc, uV;
    u8 val;

    val  = 0;
//  val |= CHGINT1_AICLOTG;
//  val |= CHGINT1_TOPOFF;
//  val |= CHGINT1_OVP;
//  val |= CHGINT1_DC_UVP;
//  val |= CHGINT1_CHG;
//  val |= CHGINT1_BAT;
//  val |= CHGINT1_THM;

    rc = max77819_write(me->io, CHGINTM1, ~val);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("CHGINTM1 write error [%d]\n", rc);
        goto out;
    }

    val  = 0;
//  val |= CHGINT2_DC_V;
//  val |= CHGINT2_CHG_WDT;
//  val |= CHGINT2_CHG_WDT_WRN;

    rc = max77819_write(me->io, CHGINTMSK2, ~val);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("CHGINTMSK2 write error [%d]\n", rc);
        goto out;
    }

#if (log_level >= 1)
    log_dbg("before dev init, dump register begin !!\n");
    max77819_charger_dump_registers(me);
#endif

    // remove irq codes of dcin interrupt
    /* buck enable */
    rc = max77819_charger_write_config(me, BUCK_EN, (me->dev_enabled ? 1 : 0));
    if (unlikely(IS_ERR_VALUE(rc))){
        log_err("enable the buck failed!!\n");
        goto out;
    }

    /* remove some lines */

    /* DCILMT enable */
    rc = max77819_charger_write_config(me, DCILIM_EN, true);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    /* charge current */
    /* no need to set the current, use the ic default value */
    /*wait max77819_charger_external_power_changed to set right current */

    /* No use hardware timer, to use software timer */
    max77819_charger_close_hardware_timer(me);

    /* topoff timer */
    val = pdata->topoff_timer <=  0 ? 0x00 :
          pdata->topoff_timer <= 60 ?
            (int)DIV_ROUND_UP(pdata->topoff_timer, 10) : 0x07;
    rc = max77819_charger_write_config(me, TOPOFFTIME, val);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    /* topoff current */
    val = pdata->topoff_current <  50000 ? 0x00 :
          pdata->topoff_current < 400000 ?
            (int)DIV_ROUND_UP(pdata->topoff_current - 50000, 50000) : 0x07;
    rc = max77819_charger_write_config(me, ITOPOFF, val);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    /* charge restart threshold */
    val = (pdata->charge_restart_threshold > 150000);
    rc = max77819_charger_write_config(me, CHGRSTRT, val);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    /* charge termination voltage */
    rc = max77819_charger_set_termination_voltage(me, pdata->charge_termination_voltage);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    /* thermistor control */
    val = (pdata->enable_thermistor == false);
    rc = max77819_charger_write_config(me, THM_DIS, val);
    rc |= max77819_charger_write_config(me, JEITA_EN, val);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    /* AICL control */
    val = (pdata->enable_aicl == false);
    rc = max77819_charger_write_config(me, DCMON_DIS, val);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    if (likely(pdata->enable_aicl)) {
        // remove "int uV" to the function head
        /* AICL detection voltage selection */
        uV = pdata->aicl_detection_voltage;
        val = uV < 3900000 ? 0x00 :
              uV < 4800000 ? (int)DIV_ROUND_UP(uV - 3900000, 100000) : 0x09;
        log_dbg("AICL detection voltage %uV (%Xh)\n", uV, val);

        rc = max77819_charger_write_config(me, AICL, val);
        if (unlikely(IS_ERR_VALUE(rc))) {
            goto out;
        }

        /* AICL reset threshold */
        uV = (int)pdata->aicl_reset_threshold;
        val = (uV > 100000);
        log_dbg("AICL reset threshold %uV (%Xh)\n", uV, val);

        rc = max77819_charger_write_config(me, AICL_RESET, val);
        if (unlikely(IS_ERR_VALUE(rc))) {
            goto out;
        }
    }

#if (log_level >= 1)
    max77819_charger_dump_registers(me);
    log_dbg("after dev init, dump register end !!\n");
#endif
    me->dev_initialized = true;
    log_dbg("device initialized\n");

out:
    return rc;
}

#define max77819_charger_psy_setprop(_me, _psy, _psp, _val) \
        ({\
            struct power_supply *__psy = _me->_psy;\
            union power_supply_propval __propval = { .intval = _val };\
            int __rc = -ENXIO;\
            if (likely(__psy && __psy->set_property)) {\
                __rc = __psy->set_property(__psy, POWER_SUPPLY_PROP_##_psp,\
                    &__propval);\
            }\
            __rc;\
        })

static void max77819_charger_psy_init (struct max77819_charger *me)
{
    if (unlikely(!me->psy_this)) {
        me->psy_this = &me->psy;
    }

    if (unlikely(!me->psy_ext && me->pdata->ext_psy_name)) {
        me->psy_ext = power_supply_get_by_name(me->pdata->ext_psy_name);
	      /* deleted 4 lines */
    }

    if (unlikely(!me->psy_coop && me->pdata->coop_psy_name)) {
        me->psy_coop = power_supply_get_by_name(me->pdata->coop_psy_name);
        if (likely(me->psy_coop)) {
            log_dbg("psy %s found\n", me->pdata->coop_psy_name);
        }
    }
}

static void max77819_charger_psy_changed (struct max77819_charger *me)
{
    max77819_charger_psy_init(me);

    if (likely(me->psy_this)) {
        power_supply_changed(me->psy_this);
    }

    if (likely(me->psy_ext)) {
        power_supply_changed(me->psy_ext);
    }

    if (likely(me->psy_coop)) {
        power_supply_changed(me->psy_coop);
    }
}

#define max77819_charger_qcom_psy(me)\
({\
    if (!me->psy_qcom_charger)\
    {\
        me->psy_qcom_charger = power_supply_get_by_name(POWR_SUPPLY_BATTERY_NAME);\
        if (!me->psy_qcom_charger)\
        {\
            log_err("the qcom battery power_supply is not got!!\n");\
        }\
    }\
    me->psy_qcom_charger;\
})

static int max77819_charger_battery_temp(struct max77819_charger *me)
{
    int rc = 0;
    union power_supply_propval bat_temp_val = {0};
    if (!max77819_charger_qcom_psy(me)){
        return BATTERY_BAD_TEMPERATURE;
    }
    rc = me->psy_qcom_charger->get_property(me->psy_qcom_charger, POWER_SUPPLY_PROP_TEMP, &bat_temp_val);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        log_err("get battery temperature failed! rc = %d \n", rc);
        return BATTERY_BAD_TEMPERATURE;
    }
    log_vdbg("get battery temperature =%d \n", bat_temp_val.intval);
    return bat_temp_val.intval;
}

static int max77819_charger_battery_present(struct max77819_charger *me)
{
	int rc = 0;
    union power_supply_propval bat_val = {0};
    if (!max77819_charger_qcom_psy(me)){
        return 0;
    }
    rc = me->psy_qcom_charger->get_property(me->psy_qcom_charger, POWER_SUPPLY_PROP_PRESENT, &bat_val);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        log_err("get battery present failed! rc = %d \n", rc);
        return 0;
    }
    log_vdbg("battery present =%d \n", bat_val.intval);
    return bat_val.intval;
}

static int max77819_charger_battery_health(struct max77819_charger *me)
{
    int theHealth = POWER_SUPPLY_HEALTH_UNKNOWN;
    int bat_temp = 0;
    struct max77819_temp_control_info* ptemp = me->ptemp_ctrl_info;

    /* check battery present */
    if (!max77819_charger_battery_present(me))
    {
        return POWER_SUPPLY_HEALTH_DEAD;
    }

    /* get battery temperature */
    bat_temp = max77819_charger_battery_temp(me);
    if (BATTERY_BAD_TEMPERATURE == bat_temp)
	{
        return POWER_SUPPLY_HEALTH_UNKNOWN;
    }

    /* cold temperature */
    if (bat_temp < ptemp->cold_bat_degree)
    {
        theHealth = POWER_SUPPLY_HEALTH_COLD;
    }
    /* cool temperature */
    else if (bat_temp >= ptemp->cold_bat_degree &&
        bat_temp <= ptemp->cool_bat_degree)
    {
        theHealth = POWER_SUPPLY_HEALTH_COOL;
    }
    /* nomal temperature */
    else if (bat_temp > ptemp->cool_bat_degree &&
        bat_temp < ptemp->warm_bat_degree)
    {
        theHealth = POWER_SUPPLY_HEALTH_GOOD;
    }
    /* warm temperature */
    else if (bat_temp >= ptemp->warm_bat_degree &&
        bat_temp <= ptemp->hot_bat_degree)
    {
        theHealth = POWER_SUPPLY_HEALTH_WARM;
    }
    /* hot temperature */
    else
    {
        theHealth = POWER_SUPPLY_HEALTH_OVERHEAT;
    }
    return theHealth;
}

static int max77819_charger_battery_report_health(struct max77819_charger *me)
{
    /* if health is overheat & temp is less than 60, must report warm health */
    if ((POWER_SUPPLY_HEALTH_OVERHEAT == me->health) &&
       (max77819_charger_battery_temp(me) < BATTERY_ALARM_TEMPERATURE))
    {
        return POWER_SUPPLY_HEALTH_WARM;
    }
    else if (POWER_SUPPLY_HEALTH_COLD == me->health)
    {
        return POWER_SUPPLY_HEALTH_COOL;
    }
    return me->health;
}

static int max77819_charger_soc(struct max77819_charger *me)
{
    int rc = 0;
    union power_supply_propval bat_val = {0};
    if (!me->psy_bms)
    {
        me->psy_bms = power_supply_get_by_name(POWR_SUPPLY_FGAUGE_NAME);
        if (!me->psy_bms)
        {
            log_err("the maxim fguage power_supply is not got!!\n");
            return 0;
        }
    }
    rc = me->psy_bms->get_property(me->psy_bms, POWER_SUPPLY_PROP_CAPACITY, &bat_val);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        log_err("get battery soc failed! rc = %d \n", rc);
        return 0;
    }
    log_vdbg("battery soc =%d \n", bat_val.intval);
    return bat_val.intval;
}

static int max77819_charger_battery_uv(struct max77819_charger *me)
{
    int rc = 0;
    union power_supply_propval bat_temp_val = {0};
    if (!me->psy_bms){
        log_err("maxim fgague bms is not got!!\n");
        return -1;
    }
    rc = me->psy_bms->get_property(me->psy_bms, POWER_SUPPLY_PROP_VOLTAGE_NOW, &bat_temp_val);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        log_err("get battery voltage failed! rc = %d \n", rc);
        return -1;
    }
    return bat_temp_val.intval;
}

struct max77819_charger_status_map {
    int health, status, charge_type;
};

static struct max77819_charger_status_map max77819_charger_status_map[] = {
    #define STATUS_MAP(_chg_dtls, _health, _status, _charge_type) \
            [CHG_DTLS_##_chg_dtls] = {\
                .health = POWER_SUPPLY_HEALTH_##_health,\
                .status = POWER_SUPPLY_STATUS_##_status,\
                .charge_type = POWER_SUPPLY_CHARGE_TYPE_##_charge_type,\
            }
    //                           health               status        charge_type
    STATUS_MAP(DEAD_BATTERY,     DEAD,                DISCHARGING, NONE),
    STATUS_MAP(PRECHARGE,        UNKNOWN,             CHARGING,    TRICKLE),
    STATUS_MAP(FASTCHARGE_CC,    UNKNOWN,             CHARGING,    FAST),
    STATUS_MAP(FASTCHARGE_CV,    UNKNOWN,             CHARGING,    FAST),
    STATUS_MAP(TOPOFF,           UNKNOWN,             CHARGING,    FAST),
    STATUS_MAP(DONE,             UNKNOWN,             FULL,        NONE),
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
    STATUS_MAP(TIMER_FAULT,      SAFETY_TIMER_EXPIRE, DISCHARGING, NONE),
#else /* LINUX_VERSION_CODE ... */
    STATUS_MAP(TIMER_FAULT,      UNKNOWN,             DISCHARGING, NONE),
#endif /* LINUX_VERSION_CODE ... */
    STATUS_MAP(TEMP_SUSPEND,     UNKNOWN,             DISCHARGING, NONE),
    STATUS_MAP(OFF,              UNKNOWN,             DISCHARGING, NONE),
    STATUS_MAP(THM_LOOP,         UNKNOWN,             CHARGING,    NONE),
    STATUS_MAP(TEMP_SHUTDOWN,    OVERHEAT,            DISCHARGING, NONE),
    STATUS_MAP(BUCK,             UNKNOWN,             DISCHARGING, NONE),
    STATUS_MAP(OTG_OVER_CURRENT, UNKNOWN,             DISCHARGING, NONE),
    STATUS_MAP(USB_SUSPEND,      UNKNOWN,             DISCHARGING, NONE),
};

static int max77819_charger_reg_inaccurate(struct max77819_charger *me)
{
    int rc = 0 ;
    u8 dc_batt_dtls = 0, dcuvp = 0;

    /* get dcin present info from DC_BATT_DTLS */
    rc = max77819_read(me->io, DC_BATT_DTLS, &dc_batt_dtls);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("DC_BATT_DTLS read error [%d]\n", rc);
        return false;
    }
    dcuvp = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_DC_UVP);

    return ((dcuvp == DC_UVP_VALID) != max77819_charger_present_input(me));
}

static int max77819_charger_battery_status(struct max77819_charger *me)
{
    int rc;
    int status = POWER_SUPPLY_STATUS_DISCHARGING;
    u8 chg_dtls, chg;
    int buck_mode;
    int chg_en = 0;

    /* check battery presetnt */
    if (!max77819_charger_battery_present(me))
    {
        return POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    if (ma77819_charger_is_otg_mode(me))
    {
        return POWER_SUPPLY_STATUS_DISCHARGING;
    }
    /* check if is the register 0x33 value inaccurate */
    if (max77819_charger_reg_inaccurate(me))
    {
        log_vdbg("the register 0x33 value is inaccurate!! \n");

        /* get dcin present */
        me->present = max77819_charger_present_input(me);
        /* get buck boost mode */
        buck_mode = max77819_charger_buck_mode(me);
        /* get charge enable */
        max77819_charger_get_enable(me, &chg_en);

        if (me->present && buck_mode && chg_en)
        {
            status = POWER_SUPPLY_STATUS_CHARGING;
        }
        else
        {
            status = POWER_SUPPLY_STATUS_DISCHARGING;
        }
    }
    else
    {
        rc = max77819_read(me->io, CHG_DTLS, &chg_dtls);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("CHG_DTLS read error [%d]\n", rc);
            goto out;
        }

        chg = BITS_GET(chg_dtls, CHG_DTLS_CHG_DTLS);
        status = max77819_charger_status_map[chg].status;
    }

out:
    return status;
}

static int max77819_charger_charge_type(struct max77819_charger *me)
{
    int rc, type, status, bat_uv;
    u8 chg_dtls, chg;

    if (ma77819_charger_is_otg_mode(me))
    {
        return POWER_SUPPLY_CHARGE_TYPE_NONE;
    }

    if (max77819_charger_reg_inaccurate(me))
    {
        status = max77819_charger_battery_status(me);
        if (POWER_SUPPLY_STATUS_CHARGING != status)
        {
            type = POWER_SUPPLY_CHARGE_TYPE_NONE;
        }
        else
        {
            bat_uv = max77819_charger_battery_uv(me);
            if (bat_uv < CHG_BATTERY_TRICKLE_UV)
            {
                type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
            }
            else
            {
                type = POWER_SUPPLY_CHARGE_TYPE_FAST;
            }
        }
    }
    else
    {
        rc = max77819_read(me->io, CHG_DTLS, &chg_dtls);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("CHG_DTLS read error [%d]\n", rc);
            return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
        }

        chg = BITS_GET(chg_dtls, CHG_DTLS_CHG_DTLS);
        type = max77819_charger_status_map[chg].charge_type;
    }
    return type;
}

static int max77819_charger_update (struct max77819_charger *me)
{
    int rc = 0;
    u8 dc_batt_dtls, chg_dtls;
    u8 batdet, bat, dcuvp, dcovp, dci, aicl;
    u8 chg, topoff, thm;

    if (log_level >= 2)
    {
        rc = max77819_read(me->io, DC_BATT_DTLS, &dc_batt_dtls);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("DC_BATT_DTLS read error [%d]\n", rc);
            goto out;
        }

        rc = max77819_read(me->io, CHG_DTLS, &chg_dtls);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("CHG_DTLS read error [%d]\n", rc);
            goto out;
        }

        log_vdbg("DC_BATT_DTLS %Xh CHG_DTLS %Xh\n", dc_batt_dtls, chg_dtls);

        batdet = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_BATDET_DTLS);
        log_vdbg("*** BATDET %s\n", max77819_charger_batdet_details[batdet]);

        bat = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_BAT_DTLS);
        log_vdbg("*** BAT    %s\n", max77819_charger_bat_details[bat]);

        dcuvp = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_DC_UVP);
        log_vdbg("*** DC_UVP %s\n", max77819_charger_dcuvp_details[dcuvp]);

        dcovp = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_DC_OVP);
        log_vdbg("*** DC_OVP %s\n", max77819_charger_dcovp_details[dcovp]);

        dci = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_DC_I);
        log_vdbg("*** DC_I   %s\n", max77819_charger_dci_details[dci]);

        aicl = BITS_GET(dc_batt_dtls, DC_BATT_DTLS_DC_AICL);
        log_vdbg("*** AICL   %s\n", max77819_charger_aicl_details[aicl]);

        chg = BITS_GET(chg_dtls, CHG_DTLS_CHG_DTLS);
        log_vdbg("*** CHG    %s\n", max77819_charger_chg_details[chg]);

        topoff = BITS_GET(chg_dtls, CHG_DTLS_TOPOFF_DTLS);
        log_vdbg("*** TOPOFF %s\n", max77819_charger_topoff_details[topoff]);

        thm = BITS_GET(chg_dtls, CHG_DTLS_THM_DTLS);
        log_vdbg("*** THM    %s\n", max77819_charger_thm_details[thm]);
    }
out:

    me->present     = max77819_charger_present_input(me);
    me->status      = max77819_charger_battery_status(me);
    me->charge_type = max77819_charger_charge_type(me);
    me->health      = max77819_charger_battery_health(me);

    log_vdbg("PRESENT %d HEALTH %d STATUS %d CHARGE_TYPE %d\n",
        me->present, me->health, me->status, me->charge_type);
    return rc;
}

#define max77819_charger_resume_log_work(_me) \
        do {\
            if (likely(log_worker)) {\
                if (likely(!delayed_work_pending(&(_me)->log_work))) {\
                    schedule_delayed_work(&(_me)->log_work, LOG_WORK_INTERVAL);\
                }\
            }\
        } while (0)

#define max77819_charger_suspend_log_work(_me) \
        cancel_delayed_work_sync(&(_me)->log_work)

/* skip read the interrupt register 0x30 */
static char* max77819_charger_register_info(struct max77819_charger *me)
{
#define MAX77819_CHG_REG_ABSE_ADDR    0x31
#define MAX77819_CHG_REG_INFO_NUM     24
#define MAX77819_CHG_REG_INFO_BUF_LEN (2*MAX77819_CHG_REG_INFO_NUM + 10)
#define MAX77819_CHG_REG_VAL_BUF_LEN  3

    static char s_chg_reg_info[MAX77819_CHG_REG_INFO_BUF_LEN] = {0};
    char reg_str[MAX77819_CHG_REG_VAL_BUF_LEN] = {0};
    u8 reg_val[MAX77819_CHG_REG_INFO_NUM] = {0};
    int rc, i;

    memset(s_chg_reg_info, 0, MAX77819_CHG_REG_INFO_BUF_LEN);
    snprintf(s_chg_reg_info, (MAX77819_CHG_REG_INFO_BUF_LEN - 1),"%d%d%d%d_%02x",
            me->stop_chg_time_out, me->bterm_chg_by_volt,
            me->bterm_chg_by_soc, me->bterm_chg_pause,
            me->irqs_reg);
    me->irqs_reg = 0; /* clear the reg record */

    rc = max77819_bulk_read(me->io, MAX77819_CHG_REG_ABSE_ADDR,
            reg_val, MAX77819_CHG_REG_INFO_NUM);
    if (likely(!IS_ERR_VALUE(rc)))
    {
        for (i = 0; i < MAX77819_CHG_REG_INFO_NUM; i++)
        {
            snprintf(reg_str, MAX77819_CHG_REG_VAL_BUF_LEN, "%02x", reg_val[i]);
            strlcat(s_chg_reg_info, reg_str, MAX77819_CHG_REG_INFO_BUF_LEN);
        }
    }

    return s_chg_reg_info;
}

#ifdef CONFIG_HUAWEI_DSM
/* check wether warm and cool current limit has exceed the current we defined*/
#define WARM_COOL_CURRENT_LIMIT		1000000
#define TEMP_BUFFER		20
#define DSM_COUNT		3

static void dsm_check_charger_temp_protecting(struct max77819_charger *me)
{
    static int warm_exceed_limit_count = 0, cool_exceed_limit_count = 0;
    int batt_temp = 0, charge_curent=0;

    /* get battery temperature */
    batt_temp = max77819_charger_battery_temp(me);
    /* get charge current */
    max77819_charger_get_chgcc(me, &charge_curent);

    if(((me->ptemp_ctrl_info->warm_bat_degree + TEMP_BUFFER) < batt_temp)
        &&(WARM_COOL_CURRENT_LIMIT < charge_curent)){
        if(warm_exceed_limit_count++ < DSM_COUNT){
            pr_info("charge_current is over warm current limit when warm, count is %d\n", warm_exceed_limit_count);
        }else{
            warm_exceed_limit_count = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_WARM_CURRENT_LIMIT_FAIL);
        }
    }else{
        warm_exceed_limit_count = 0;
    }

    if(((me->ptemp_ctrl_info->cool_bat_degree - TEMP_BUFFER) > batt_temp)
        &&(WARM_COOL_CURRENT_LIMIT < charge_curent)){
        if(cool_exceed_limit_count++ < DSM_COUNT){
            pr_info("charge_current is over cool current limit when cool, count is %d\n", cool_exceed_limit_count);
        }else{
            cool_exceed_limit_count = 0;
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_COOL_CURRENT_LIMIT_FAIL);
        }
    }else{
        cool_exceed_limit_count = 0;
    }
}
#endif

static void max77819_charger_log_work (struct work_struct *work)
{
    struct max77819_charger *me =
        container_of(work, struct max77819_charger, log_work.work);
    int val_en = 0, val_dc = 0, val_chg = 0;
    char* p_chg_reg_info = NULL;

#ifdef CONFIG_HUAWEI_DSM
    /* get charge online/dcin status */
    me->present = max77819_charger_present_input(me);
    /* check dcin, otg mode && charge enable */
    if (me->dev_enabled && me->present && !ma77819_charger_is_otg_mode(me)){
        /* check whether warm and cool current limit has exceed or not effected*/
        dsm_check_charger_temp_protecting(me);
    }

    /*monitor max77819 charger/battery status from regs*/
    monitor_max77819_charger_regs(me);
#endif
    /* delete one line*/
    if(log_level >= 2)
    {
        max77819_charger_get_enable(me, &val_en);
        max77819_charger_get_dcilmt(me, &val_dc);
        max77819_charger_get_chgcc(me, &val_chg);
        log_info("chg_en=%s, dc_lmt=(%duA, %duA), chgcc=(%duA,%duA)\n",
                (val_en ? "on" : "off"),
                me->current_limit_volatile, val_dc,
                me->charge_current_volatile, val_chg);

        p_chg_reg_info = max77819_charger_register_info(me);
        val_en = max77819_charger_battery_uv(me);
        log_info("(CHG_DTLS[0x34], FCHGCRNT[0x37], FCHGCRNT[0x38],\
        TOPOFF[0x39], BATREG[0x3A], BATREG[0x3E]) = (%s), vbat = %duV\n",
                p_chg_reg_info, val_en);
    }
    max77819_charger_resume_log_work(me);
    /* delete one line*/
}

static void max77819_charger_irq_work (struct work_struct *work)
{
    struct max77819_charger *me =
        container_of(work, struct max77819_charger, irq_work.work);

    if (ma77819_charger_is_otg_mode(global_Charger))
    {
        log_info("max77819 chip device is in otg mode, do nothing!\n");
        return;
    }
    __lock(me);

    log_dbg("DC input %s irq, dev init state %d\n",
            me->present ? "inserted" : "removed", me->dev_initialized);
    if (me->present ^ me->dev_initialized) {
        me->bfactory_daig_chg = true;
        me->resuming_charging = true;
        me->btemp_chg_protecting = false;

        max77819_charger_psy_init(me);

        if (likely(me->present)) {
            max77819_charger_init_dev(me);
            max77819_charger_set_enable(me, true);
            /* read the interrupt reg to clear the dcin irq CHG_IRQ_DC_UVP */
            max77819_charger_read_irq_status(me, CHGINT);

            /* Only dcin cable present and it is not otg, this work should be scheduled */
            if (likely(!ma77819_charger_is_otg_mode(me)))
            {
                schedule_delayed_work(&me->timer_work, msecs_to_jiffies(MAXIM77819_TIMER_TIMEOUT));
            }
        } else {
            max77819_charger_exit_dev(me);
        }

        max77819_charger_psy_setprop(me, psy_coop, CHARGING_ENABLED, me->present);
        max77819_charger_psy_setprop(me, psy_ext, PRESENT, me->present);
    }

    /* notify psy changed */
    max77819_charger_psy_changed(me);

    __unlock(me);
    return;
}

/* remove  max77819_charger_topoff_work (struct work_struct *work) */

static int max77819_charger_set_current_by_health(struct max77819_charger* me)
{
    int iset_ua = 0;

    if (CHG_AICL_WORKING_FALL == me->aicl_work_state ||
        CHG_AICL_WORKING_RISE == me->aicl_work_state)
    {
        log_info("system is doing aicl, can not set the current!! \n");
        return 0;
    }

    /* get the max current in temp protecting */
    iset_ua = max77819_charger_temp_protect_max_ua(me);

    /* get the current will be set */
    iset_ua = min(iset_ua, me->ptemp_ctrl_info->sys_limit_current);
    iset_ua = min(iset_ua, me->charge_current_volatile);

    /* set chgcc current */
    return max77819_charger_set_charge_current(me, me->current_limit_volatile, iset_ua);
}

static int max77819_charger_set_current_by_temp(struct max77819_charger* me)
{
    /* get battery temp status, health */
    me->health = max77819_charger_battery_health(me);

    /* set chgcc current by health */
    return max77819_charger_set_current_by_health(me);
}

static int max77819_charger_set_voltage_by_health(struct max77819_charger* me)
{
    int vset_uv = 0;

    switch (me->health)
    {
    case POWER_SUPPLY_HEALTH_COOL:
        vset_uv = me->ptemp_ctrl_info->vmaxuv_cool_bat;
        break;
    case POWER_SUPPLY_HEALTH_WARM:
        vset_uv = me->ptemp_ctrl_info->vmaxuv_warm_bat;
        break;
    default:
        vset_uv = me->pdata->charge_termination_voltage;
        break;
    }

    /* get the terminate battery voltage will be set */
    vset_uv = min(vset_uv, (int)me->pdata->charge_termination_voltage);

    /* set chgcc current */
    return max77819_charger_set_termination_voltage(me, vset_uv);
}

static void max77819_charger_temp_protecting(struct max77819_charger *me)
{
    static int pre_health = POWER_SUPPLY_HEALTH_UNKNOWN;
    static int force_count = 0;

    /* get charge states && health */
    me->status = max77819_charger_battery_status(me);
    me->health = max77819_charger_battery_health(me);

    force_count++;
    /* check battery temperature type */
    if (me->health == pre_health && 
        me->btemp_chg_protecting &&
        (TEMP_PROTECTING_FORCE_TIME / TEMP_PROTECTING_WORK_TIME) > force_count)
    {
        log_vdbg("the battery health not change! state = %d \n", pre_health);
        return;
    }
    log_dbg("health=%d, protecting=%d, force_count=%d \n",
                me->health, me->btemp_chg_protecting, force_count);
    pre_health = me->health;
    me->btemp_chg_protecting = true;
    force_count = 0;

    /* check battery health to stop charging */
    if (POWER_SUPPLY_HEALTH_OVERHEAT == me->health ||
        POWER_SUPPLY_HEALTH_COLD == me->health ||
        POWER_SUPPLY_HEALTH_DEAD == me->health ||
        POWER_SUPPLY_HEALTH_UNKNOWN == me->health)
    {
         if (POWER_SUPPLY_STATUS_CHARGING == me->status)
        {
            log_dbg("battery temperature state = %d, to stop charge.\n", me->health);
            max77819_charger_set_enable(me, false);
        }
    }
    /* battery health changed, should be reset current & voltage, be resume charge */
    else
    {
        /* set chgcc current */
        max77819_charger_set_current_by_health(me);
        /* battery terminate voltage */
        max77819_charger_set_voltage_by_health(me);

        /* if not in charging, should be resume */
        if (POWER_SUPPLY_STATUS_CHARGING != me->status)
        {
            me->resuming_charging = false;
            max77819_charger_resume_charging(me);
            log_dbg("battery health = %d, status = %d, to resume charge.\n",
                    me->health, me->status);
        }
    }
}

static int max77819_charger_buck_mode(struct max77819_charger *me)
{
    int rc = 0;
    u8 en = 0;
    rc = max77819_charger_read_config(me, BUCK_EN, &en);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        log_err("get buck buck mode failed, rc = %d \n", rc);
        return 0;
    }
    log_vdbg("get buck mode: %s\n", en ? "enabling" : "disabling");
    return (en ? 1 : 0);
}

static int max77819_charger_resume_charging(struct max77819_charger *me)
{
    int rc = 0;
    int buck_mode;

    if (ma77819_charger_is_otg_mode(me))
    {
        log_vdbg("The chip is in otg mode, do nothing!\n");
        return 0;
    }

    if (!me->dev_enabled|| !me->bfactory_daig_chg ||
        (CHG_TIMER_OVER_STOP == me->stop_chg_time_out) ||
         me->bterm_chg_by_volt || me->bterm_chg_pause || me->bterm_chg_by_soc)
    {
        log_vdbg("Not resume: (chgen,chgdg,time_out,trm_v,pause,trm_s) = (%d,%d,%d,%d,%d,%d) \n",
                    me->dev_enabled, me->bfactory_daig_chg, me->stop_chg_time_out,
                    me->bterm_chg_by_volt, me->bterm_chg_pause, me->bterm_chg_by_soc);
        return 0;
    }

    /* get dcin cable present */
    me->present = max77819_charger_present_input(me);
    if (!me->present)
    {
        log_vdbg("Not resume: dcin = %d \n", me->present);
        return 0;
    }

    if (true == me->resuming_charging)
    {
        /* get charge & resume status */
        me->status = max77819_charger_battery_status(me);
        if (POWER_SUPPLY_STATUS_DISCHARGING == me->status)
        {
            me->resuming_charging = false;
        }
        else
        {
            log_vdbg("Not resume:(rechg,status) = (%d,%d) \n",
                        me->resuming_charging, me->status);
            return 0;
        }
    }

    /* get buck boost mode */
    buck_mode = max77819_charger_buck_mode(me);
    if (!buck_mode)
    {
        log_vdbg("Not resume:buck_mode = %d \n", buck_mode);
        return 0;
    }

    /* get battery soc */
    me->soc = max77819_charger_soc(me);
    if (me->soc > RESUME_CHARGE_CAPACTIY)
    {
        log_vdbg("Not resume:capacity = %d \n", me->soc);
        return 0;
    }

    /* get health to check if stoping charge */
    me->health = max77819_charger_battery_health(me);
    if (POWER_SUPPLY_HEALTH_OVERHEAT == me->health ||
        POWER_SUPPLY_HEALTH_COLD == me->health ||
        POWER_SUPPLY_HEALTH_DEAD == me->health ||
        POWER_SUPPLY_HEALTH_UNKNOWN == me->health)
    {
        log_vdbg("Not resume:over temp to stop charge, health = %d \n", me->health);
        return 0;
    }

    /* to resume charge */
    rc = max77819_charger_set_enable(me, true);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        log_err("enable charge failed, rc = %d !! \n", rc);
        return rc;
    }
    me->resuming_charging = true;
    log_dbg("resuming charge at soc = %d ! \n", me->soc);
    return 0;
}

/*========================================================================================
FUNCTION  max77819_charger_terminate_by_temp_voltage
DESCRIPTION
  According the battery temp & voltage, to check if it should terminate charge.
  Because the battery voltage can not be higher than a upper limit voltage when charging
  ander the cool or warm temp.
PARAMETERS
  struct max77819_charger *me : this module driver parameter struct
DEPENDENCIES
  me->health : battery health, only cool and warm status, this fucntion working.
  battery_uv : battery voltage, if value higher than the max voltage, need to stop charge.
  me->bterm_chg_by_soc: remark parameter, true:terminate charge; false: No terminate charge 
RETURN VALUE
  None
SIDE EFFECTS
  None
========================================================================================*/
static void max77819_charger_terminate_by_temp_voltage(struct max77819_charger *me)
{
    int rc = 0;
    static int term_num = 0;
    static int rechg_num = 0;
    int topoff_num = CHG_TOPOFF_VOLTAGE_TIME * (TIME_UNIT_MINUTE / CHG_TOPOFF_CHECK_TIEM);
    int batt_uv = 0, bat_term_uv = 0;

    /* get health, when health is not cool or warm, no need to terminate charge by temp contorl */
    me->health = max77819_charger_battery_health(me);
    if (POWER_SUPPLY_HEALTH_COOL == me->health)
    {
        /* get terminate voltage */
        bat_term_uv = me->ptemp_ctrl_info->vmaxuv_cool_bat;
    }
    else if (POWER_SUPPLY_HEALTH_WARM == me->health)
    {
        /* get terminate voltage */
        bat_term_uv = me->ptemp_ctrl_info->vmaxuv_warm_bat;
    }
    else
    {
        term_num = 0;
        rechg_num = 0;
        bat_term_uv = me->pdata->charge_termination_voltage;
        me->bterm_chg_by_volt = false;
        return;
    }

    /* get battery voltage */
    batt_uv = max77819_charger_battery_uv(me);
    if (batt_uv < 0)
    {
        log_err("get battery uv err value: %d", batt_uv);
        return;
    }

    /* check the battery voltage whether over the terminate voltage */
    if (batt_uv >= bat_term_uv)
    {
        term_num++;
        rechg_num = 0;
        if (term_num >= topoff_num)
        {
            /* terminate charging */
            rc = max77819_charger_set_enable(me, false);
            if (likely(!IS_ERR_VALUE(rc)))
            {
                log_info("time to terminate charging by voltage %duV, health = %d !! \n", bat_term_uv, me->health);
                me->bterm_chg_by_volt = true;
                term_num = 0;
            }
            else
            {
                log_err("disable charge failed !!\n");
            }
        }
    }
    else
    {
        term_num = 0;
        /* check the batter voltage fall to under vterminate_mv */
        if (batt_uv <= (bat_term_uv - RESUME_CHG_VOLTAGE_BUFF))
        {
            rechg_num++;
        }
        else
        {
            rechg_num = 0;
        }
        if (rechg_num >= topoff_num)
        {
            me->bterm_chg_by_volt = false;
            rechg_num = 0;
        }
    }
}

static void max77819_charger_terminate_by_soc_time(struct max77819_charger *me)
{
    int rc = 0;
    static int term_num = 0;
    int topoff_time_num = me->pdata->topoff_timer * 
            (TIME_UNIT_MINUTE / CHG_TOPOFF_CHECK_TIEM);

    /* get soc, when soc < 100%, no need to terminate charge by soc */
    me->soc = max77819_charger_soc(me);
    if (BATTERY_FULL_CAPACITY > me->soc)
    {
        term_num = 0;
        me->bterm_chg_by_soc = false;
        return;
    }

    /* already terminate by soc, no need to do next */
    if ( me->bterm_chg_by_soc)
    {
        term_num = 0;
        return;
    }

    term_num++;
    if (term_num >= topoff_time_num)
    {
        /* terminate charging */
        rc = max77819_charger_set_enable(me, false);
        if (likely(!IS_ERR_VALUE(rc)))
        {
            log_info("time to terminate charging by soc time %dMin !! \n", me->pdata->topoff_timer);
            me->bterm_chg_by_soc = true;
            term_num = 0;
        }
        else
        {
            log_err("disable charge failed !!\n");
        }
    }
}

static void max77819_charger_terminate_by_soc_loop_vbat(struct max77819_charger* me)
{
    int rc = 0;
    static int time_count_num = 0;
    int batt_uv = 0;
    int time_to_pause_chg_num = CHG_TOPOFF_LOOP_VOLT_TIME / CHG_TOPOFF_LOOP_TIME;

    /* get soc, when soc < 100%, no need to terminate charge by soc */
    me->soc = max77819_charger_soc(me);
    if (BATTERY_FULL_CAPACITY > me->soc)
    {
        me->bterm_chg_pause = false;
        me->bterm_chg_by_soc = false;
        time_count_num = 0;
        return;
    }

    /* already terminate by soc, no need to do next */
    if (me->bterm_chg_by_soc)
    {
        me->bterm_chg_pause = false;
        time_count_num = 0;
        return;
    }

    if (time_count_num < time_to_pause_chg_num)
    {
        me->bterm_chg_pause = false;
        time_count_num++;
    }
    else if (time_count_num == time_to_pause_chg_num)
    {
        /* pause charge to get real battery voltage */
        rc = max77819_charger_set_enable(me, false);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("disable charge failed !!\n");
            me->bterm_chg_pause = false;
        }
        else
        {
            time_count_num++;
            me->bterm_chg_pause = true;
        }
    }
    else
    {
        /* read real battery voltage and compare to terminate voltage */
        batt_uv = max77819_charger_battery_uv(me);
        if (batt_uv >= (me->pdata->charge_termination_voltage - CHG_TOPOFF_BATT_MARGIN_UV))
        {
            log_info("real voltage %d to terminate charging!!\n", batt_uv);
            me->bterm_chg_by_soc = true;
            me->bterm_chg_pause = false;
            time_count_num = 0;
        }
        else
        {
            /* resume charge if the battery voltage is low than the terminate voltage */
            rc = max77819_charger_set_enable(me, true);
            if (unlikely(IS_ERR_VALUE(rc)))
            {
                log_err("enable charge failed !!\n");
            }
            else
            {
                me->bterm_chg_pause = false;
                time_count_num = 0;
            }
        }
    }
}

static void max77819_charger_stop_charging_by_max_time(struct max77819_charger* me)
{
    int rc = 0;
    static int count_time_num = 0;
    int max_time_num = 0;
    union power_supply_propval val = {0};

    /* already stop charge by time out, clear the count time & do nothing */
    if (CHG_TIMER_OVER_STOP == me->stop_chg_time_out)
    {
        count_time_num = 0;
        return;
    }
    else if (CHG_TIMER_RESET == me->stop_chg_time_out)
    {
        /* reset the count num to recount */
        count_time_num = 0;
        me->stop_chg_time_out = CHG_TIMER_NONE;
        log_info("charge timer reset to 0 \n");
    }

    /* Not in charging, clear the count time & do nothing */
    me->status = max77819_charger_battery_status(me);
    if (POWER_SUPPLY_STATUS_CHARGING != me->status && !me->bterm_chg_pause)
    {
        if (0 != count_time_num)
        {
            count_time_num = 0;
            me->stop_chg_time_out = CHG_TIMER_NONE;
            log_info("not charging, charge timer reset to 0 \n");
        }
        return;
    }

    /* get current max value by "usb" power_supply to check dcin cable type is usb or dcp */
    rc = max77819_charger_get_ext_property(me->psy_ext, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
    if (likely(!IS_ERR_VALUE(rc)) && val.intval > CHG_CURRENT_TYPE_USB)
    {
        /* dcin is usb_dcp, the max time is 10 hours */
        max_time_num = CHG_CONT_TIME_MAX_DCP / CHG_CONT_TIME_WORK_TIME;
    }
    else
    {
        /* dcin is pc usb, the max time is 12 hours */
        max_time_num = CHG_CONT_TIME_MAX_USB / CHG_CONT_TIME_WORK_TIME;
    }

    count_time_num++;
    me->stop_chg_time_out = CHG_TIMER_COUNTING;
    if (count_time_num >= max_time_num)
    {
        rc = max77819_charger_set_enable(me, false);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("enable charge failed !!\n");
        }
        else
        {
            me->stop_chg_time_out = CHG_TIMER_OVER_STOP;
            count_time_num = 0;
#ifdef CONFIG_HUAWEI_DSM
            dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_TIMER_FAULT);
#endif
            log_err("charge time out, stop charge, time = %ds!! \n",
                        (max_time_num * CHG_CONT_TIME_WORK_TIME));
        }
    }
}

static void max77819_charger_check_to_aicl_fall(struct max77819_charger* me)
{
    static int aicl_count = 0;

    if (CHG_AICL_NEED_CHECK != me->aicl_work_state)
    {
        log_vdbg("system is no need to do aicl, state:%d !! \n", me->aicl_work_state);
        aicl_count = 0;
        return;
    }

    if (!max77819_charger_allow_to_aicl_fall(me))
    {
        aicl_count = 0;
        return;
    }

    if (!max77819_charger_buck_mode(me))
    {
        log_vdbg("the buck is disabled, no to do aicl!! \n");
        aicl_count = 0;
        return;
    }

    if (max77819_charger_in_aicl(me))
    {
        aicl_count++;
        if (AICL_MODE_COUNT_NUM_MAX <= aicl_count)
        {
            aicl_count = 0;
            me->aicl_work_state = CHG_AICL_WORKING_FALL;
            log_info("aicl work to adjust fall correct current!!\n");
            schedule_delayed_work(&me->aicl_work, msecs_to_jiffies(CHG_CURRENT_AICL_WORK_TIME));
        }
    }
    else
    {
        aicl_count = 0;
    }
}

static void max77819_charger_check_to_aicl_rise(struct max77819_charger* me)
{
    if (CHG_AICL_NEED_CHECK != me->aicl_work_state)
    {
        log_vdbg("system is no need to do aicl, state:%d !! \n", me->aicl_work_state);
        return;
    }

    if (!max77819_charger_allow_to_aicl_rise(me))
    {
        return;
    }

    if (!max77819_charger_buck_mode(me))
    {
        log_vdbg("the buck is disabled, no to do aicl!! \n");
        return;
    }

    if (!max77819_charger_in_aicl(me))
    {
        me->aicl_work_state = CHG_AICL_WORKING_RISE;
        log_info("aicl work to adjust rise correct current!!\n");
        schedule_delayed_work(&me->aicl_work, msecs_to_jiffies(CHG_CURRENT_AICL_WORK_TIME));
    }
}

static void max77819_charger_dcuvp_irq(struct max77819_charger* me)
{
    me->present = max77819_charger_present_input(me);

    if (me->present)
    {
        /* init charger */
        max77819_charger_init_dev(me);

        /* control charge enable */
        if (me->dev_enabled)
        {
            me->dev_enabled = false;
            max77819_charger_enable_control(me, true);
        }
        else
        {
            me->dev_enabled = true;
            max77819_charger_enable_control(me, false);
        }

        /* reset current */
        max77819_charger_set_current_by_dcin_type(me);
    }
}

static void max77819_charger_irqs_monitor(struct max77819_charger* me)
{
    u8 reg_val = 0;
    reg_val = max77819_charger_read_irq_status(me, CHGINT);

    if (!reg_val)
    {
        /* no any interrupt */
        return;
    }
    log_vdbg("charger interrupt, irq_reg=0x%02x..\n", reg_val);
    me->irqs_reg = reg_val;

    /* the IC will reset when CHG_IRQ_DC_UVP happends */
    /* so, must retry to init charge ic and to resume charging */
    if (me->irqs_reg & CHG_IRQ_DC_UVP)
    {
        log_info("DC under voltage interrupt..\n");
        max77819_charger_dcuvp_irq(me);
    }
}

#define max77819_charger_timer_work_delay_ok(time)\
({\
    bool bdly_ok_##__func__##__LINE__ = true;\
    static int dly_##__func__##__LINE__ = 0;\
    int dlyNum_##__func__##__LINE__ = 0;\
\
    dlyNum_##__func__##__LINE__ = time / MAXIM77819_TIMER_TIMEOUT;\
    if (time % MAXIM77819_TIMER_TIMEOUT)\
    {\
        dlyNum_##__func__##__LINE__ += 1;\
    }\
\
    dly_##__func__##__LINE__++;\
    if (dly_##__func__##__LINE__ < dlyNum_##__func__##__LINE__)\
    {\
        bdly_ok_##__func__##__LINE__ = false;\
    }\
    else\
    {\
        bdly_ok_##__func__##__LINE__ = true;\
        dly_##__func__##__LINE__ = 0;\
    }\
\
    bdly_ok_##__func__##__LINE__;\
})

static void max77819_charger_timer_work(struct work_struct *work)
{
    struct max77819_charger *me =
        container_of(work, struct max77819_charger, timer_work.work);

    __lock(me);

    /* delay some seconds */
    if (max77819_charger_timer_work_delay_ok(TEMP_PROTECTING_WORK_TIME))
    {
        max77819_charger_temp_protecting(me);
    }

    /* delay some seconds */
    if (max77819_charger_timer_work_delay_ok(RESUME_CHARGE_WORK_TIME))
    {
        max77819_charger_resume_charging(me);
    }

    /* delay some seconds */
    if (max77819_charger_timer_work_delay_ok(CHG_TOPOFF_CHECK_TIEM))
    {
        max77819_charger_terminate_by_soc_time(me);
        max77819_charger_terminate_by_temp_voltage(me);
    }

    if (max77819_charger_timer_work_delay_ok(CHG_TOPOFF_LOOP_TIME))
    {
        max77819_charger_terminate_by_soc_loop_vbat(me);
    }

    if (max77819_charger_timer_work_delay_ok(CHG_CONT_TIME_WORK_TIME))
    {
        max77819_charger_stop_charging_by_max_time(me);
    }

    if (max77819_charger_timer_work_delay_ok(AICL_MODE_FALL_WORK_TIME))
    {
        max77819_charger_check_to_aicl_fall(me);
    }

    if (max77819_charger_timer_work_delay_ok(AICL_MODE_RISE_WORK_TIME))
    {
        max77819_charger_check_to_aicl_rise(me);
    }

    if (max77819_charger_timer_work_delay_ok(CHG_IRQS_MONITOR_WORK_TIME))
    {
        max77819_charger_irqs_monitor(me);
    }

    /* Only dcin cable present and it is not otg, this work should be scheduled */
    if (likely(me->present && !ma77819_charger_is_otg_mode(me)))
    {
        loop_schedule_delayed_work(&me->timer_work, msecs_to_jiffies(MAXIM77819_TIMER_TIMEOUT));
    }
    __unlock(me);
}

//void max77819_charger_rechg(bool enable)

static void max77819_charger_otg_control(struct max77819_charger* me, bool enable)
{
    int rc;

    /* delete the mutex lock */
    if (me->dev_otg_mode == enable)
    {
        log_dbg("otg mode(dev_otg_mode = %s) do not change, do nothing!!\n",
                enable ? "enabled" : "disabled");
        return;
    }
    log_dbg("%s\n", enable ? "enabling otg" : "disabling otg");
    if(enable){
        rc = max77819_write(me->io, RBOOST_CTL2, 0x50);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("RBOOST_CTL2 write error [%d]\n", rc);
            goto out;
        }
        rc = max77819_write(me->io, RBOOST_CTL1, 0x01);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("RBOOST_CTL1 write error [%d]\n", rc);
            goto out;
        }
        rc = max77819_write(me->io, BAT2SOC_CTL, 0x20);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("BAT2SOC_CTL write error [%d]\n", rc);
            goto out;
        }
    }
    else{
        rc = max77819_write(me->io, BAT2SOC_CTL, 0x00);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("BAT2SOC_CTL write error [%d]\n", rc);
            goto out;
        }
        rc = max77819_write(me->io, RBOOST_CTL1, 0x00);
        if (unlikely(IS_ERR_VALUE(rc))) {
            log_err("RBOOST_CTL1 write error [%d]\n", rc);
            goto out;
        }
    }
    /* mark the chip otg mode */
    me->dev_otg_mode = enable;
out:
    if (unlikely(IS_ERR_VALUE(rc))) {
#ifdef CONFIG_HUAWEI_DSM
        /* if maxim otg enable/disable failed, record this log, and notify to the dsm server*/
        dump_registers_and_adc(charger_dclient, g_lbc_chip, DSM_MAXIM_ENABLE_OTG_FAIL);
#endif
        log_err("failed to enable/disable max77819 OTG [%d]\n", rc);
    }
}

static void max77819_charger_set_otg_by_dcin_type(struct max77819_charger* me)
{
    int rc;
    union power_supply_propval val = {0};

    rc = max77819_charger_get_ext_property(me->psy_ext, POWER_SUPPLY_PROP_SCOPE, &val);
    if (likely(!IS_ERR_VALUE(rc)))
    {
        if (POWER_SUPPLY_SCOPE_SYSTEM == val.intval)
        {
            /* the otg plugin, turn on the otg mode */
            max77819_charger_otg_control(me, true);
        }
        else
        {
            /* the otg plugin, turn on the otg mode */
            max77819_charger_otg_control(me, false);
        }
    }
    else
    {
        log_err("could not read USB scope property, rc=%d\n", rc);
    }
}

void notify_max77819_to_control_otg(bool enable)
{
    if (!global_Charger)
    {
        log_info("max77819 chip device not init,do nothing!\n");
        return;
    }
    max77819_charger_otg_control(global_Charger, enable);
}
EXPORT_SYMBOL(notify_max77819_to_control_otg);

/*========================================================================================
FUNCTION  max77819_charger_get_boost
DESCRIPTION
  read the boost register to get if the boost enabled
PARAMETERS
  struct max77819_charger *me : this module driver parameter struct
  en : boost enable status, 0: Reverse boost disabled, 1:Reverse boost enabled
DEPENDENCIES
  None
RETURN VALUE
  if 0, successed, else failed.
SIDE EFFECTS
  None
========================================================================================*/
static int max77819_charger_get_boost(struct max77819_charger* me, int* en)
{
    int rc;
    u8 cen = 0;

    rc = max77819_charger_read_config(me, RBOOSTEN, &cen);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    *en = !!cen;
    log_vdbg("<boost_enable> %s\n", *en ? "enabled" : "disabled");

out:
    return rc;
}

int get_max77819_boost_mode()
{
    int en = 0;
    if (!global_Charger)
    {
        pr_info("max77819 chip device not init,do nothing!\n");
        return 0;
    }

    max77819_charger_get_boost(global_Charger, &en);
    return en;
}
EXPORT_SYMBOL(get_max77819_boost_mode);

//remove the max77819_charger_isr function for dcin interrupt

void do_max77819_dcin_valid_irq(int present)
{
    if (!global_Charger)
    {
        log_info("max77819 chip device not init,do nothing!\n");
        return;
    }
    if (present != global_Charger->present)
    {
        global_Charger->present = present;
    }
    schedule_delayed_work(&global_Charger->irq_work, IRQ_WORK_DELAY);
}
EXPORT_SYMBOL(do_max77819_dcin_valid_irq);

int get_max77819_cable_max_current_ua(void)
{
    if (!global_Charger)
    {
        log_info("max77819 chip device not init,do nothing!\n");
        return 0;
    }

    if (likely(global_Charger->present))
    {
        return global_Charger->current_limit_volatile;
    }

    return 0;
}
EXPORT_SYMBOL(get_max77819_cable_max_current_ua);

static int max77819_charger_get_property (struct power_supply *psy,
        enum power_supply_property psp, union power_supply_propval *val)
{
    struct max77819_charger *me =
        container_of(psy, struct max77819_charger, psy);
    int rc = 0;
    /* delete the mutex lock */

    rc = max77819_charger_update(me);
    if (unlikely(IS_ERR_VALUE(rc))) {
        goto out;
    }

    switch (psp) {
    case POWER_SUPPLY_PROP_PRESENT:
        max77819_charger_get_ext_property(me->psy_qcom_charger, POWER_SUPPLY_PROP_PRESENT, val);
        break;

    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        val->intval = me->dev_enabled;
        break;
    case POWER_SUPPLY_PROP_FACTORY_DIAG:
        val->intval = me->bfactory_daig_chg;
        break;

    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = max77819_charger_battery_report_health(me);
        break;

    case POWER_SUPPLY_PROP_STATUS:
        val->intval = me->status;
        /* when the soc is 100% and dcin is connected, set the status full */
        if (me->present && !ma77819_charger_is_otg_mode(me))
        {
            /* get battery soc */
            me->soc = max77819_charger_soc(me);
            if (BATTERY_FULL_CAPACITY == me->soc)
            {
                val->intval = POWER_SUPPLY_STATUS_FULL;
            }
        }
        break;

    case POWER_SUPPLY_PROP_CHARGE_TYPE:
        val->intval = me->charge_type;
        break;

    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        val->intval = me->charge_current_volatile;
        break;

    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
        val->intval = me->current_limit_volatile;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        /* get battery soc */
        me->soc = max77819_charger_soc(me);
        val->intval = me->soc;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        max77819_charger_get_ext_property(me->psy_bms, POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
        break;
    case POWER_SUPPLY_PROP_TEMP:
        rc = max77819_charger_get_ext_property(me->psy_qcom_charger, POWER_SUPPLY_PROP_TEMP, val);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            val->intval = 250;
        }
        break;
    case POWER_SUPPLY_PROP_RESUME_CHARGING:
        val->intval = me->resuming_charging;
        break;
    case POWER_SUPPLY_PROP_CHARGE_LOG:
        val->strval = max77819_charger_register_info(me);
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        max77819_charger_get_ext_property(me->psy_bms, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, val);
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        max77819_charger_get_ext_property(me->psy_bms, POWER_SUPPLY_PROP_CURRENT_NOW, val);
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
        break;
    case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
        val->intval = me->ptemp_ctrl_info->sys_limit_current / 1000;
        break;

    default:
        rc = -EINVAL;
        goto out;
    }

out:
    log_vdbg("<get_property> psp %d val %d [%d]\n", psp, val->intval, rc);
    return rc;
}

static int max77819_charger_enable_control(struct max77819_charger *me,bool enable)
{
    int rc = 0;
    if (me->dev_enabled == enable)
    {
        log_dbg("charging_enabled(dev_enabled) do not change, do nothing!!\n");
        return 0;
    }
    me->dev_enabled = enable;
    if (!max77819_charger_present_input(me))
    {
        log_dbg("No dcin cable, do nothing!!\n");
        return 0;
    }

    if (enable)
    {
        log_dbg("when control enable charging, enable the buck and resum charging...\n");
        rc = max77819_charger_write_config(me, BUCK_EN, 1);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("when control enable charging, enable the buck failed!! rc=%d \n", rc);
            return rc;
        }
        me->resuming_charging = false;
        rc = max77819_charger_resume_charging(me);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("when control enable charging, resume charging failed!! rc=%d\n", rc);
            return rc;
        }
    }
    else
    {
        log_dbg("when control disable charging, stop charging and disable the buck...\n");
        rc = max77819_charger_set_enable(me,0);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("when control disable charging, stop charging failed!! rc=%d \n", rc);
            return rc;
        }
        rc = max77819_charger_write_config(me, BUCK_EN, 0);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("when control disable charging, disable the buck failed!! rc=%d \n", rc);
            return rc;
        }
    }
    return 0;
}

static int max77819_charger_diag_control(struct max77819_charger *me,bool enable)
{
    int rc = 0;
    if (me->bfactory_daig_chg == enable)
    {
        log_dbg("factory_daig_chg do not change, do nothing!!\n");
        return 0;
    }
    me->bfactory_daig_chg = enable;

    if (!max77819_charger_present_input(me))
    {
        log_dbg("No dcin cable, do nothing!!\n");
        return 0;
    }

    if (enable)
    {
        log_dbg("when factory diag enable charging, resum charging...\n");
        me->resuming_charging = false;
        rc = max77819_charger_resume_charging(me);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("when factory diag enable charging, resume charging failed!! rc=%d\n", rc);
            return rc;
        }
    }
    else
    {
        log_dbg("when factory diag disable charging, stop charging...\n");
        rc = max77819_charger_set_enable(me,0);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("when control disable charging, stop charging failed!! rc=%d \n", rc);
            return rc;
        }
    }
    return 0;
}

static int max77819_charger_sys_hot_limit_current(struct max77819_charger *me, int ua)
{
    if (me->ptemp_ctrl_info->sys_limit_current != ua)
    {
        /* set ua zero, means not to limit current by system hot */
        ua = (0 == ua) ? CHG_CURRENT_HOT_LIMIT_MAX : ua;

        me->ptemp_ctrl_info->sys_limit_current = 
            (ua > 0) ? min(ua, CHG_CURRENT_HOT_LIMIT_MAX) : 0;
        return max77819_charger_set_current_by_temp(me);
    }
    return 0;
}

static int max77819_charger_set_property (struct power_supply *psy,
        enum power_supply_property psp, const union power_supply_propval *val)
{
    struct max77819_charger *me =
        container_of(psy, struct max77819_charger, psy);
    int uA, rc = 0;

    __lock(me);

    switch (psp) {
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        max77819_charger_enable_control(me, !!val->intval);
        break;
    case POWER_SUPPLY_PROP_FACTORY_DIAG:
        max77819_charger_diag_control(me, !!val->intval);
        break;

    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        uA = abs(val->intval);
        rc = max77819_charger_set_charge_current(me, me->current_limit_volatile,
            uA);
        if (unlikely(IS_ERR_VALUE(rc))) {
            goto out;
        }

        me->charge_current_volatile  = uA;
        me->charge_current_permanent =
            val->intval > 0 ? uA : me->charge_current_permanent;
        break;

    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
        uA = abs(val->intval);
        rc = max77819_charger_set_charge_current(me, uA,
            me->charge_current_volatile);
        if (unlikely(IS_ERR_VALUE(rc))) {
            goto out;
        }

        me->current_limit_volatile  = uA;
        me->current_limit_permanent =
            val->intval > 0 ? uA : me->current_limit_permanent;
        break;

    case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
        max77819_charger_sys_hot_limit_current(me, (val->intval * 1000));
        break;

    default:
        rc = -EINVAL;
        goto out;
    }

out:
    log_vdbg("<set_property> psp %d val %d [%d]\n", psp, val->intval, rc);
    __unlock(me);
    return rc;
}

static int max77819_charger_property_is_writeable (struct power_supply *psy,
    enum power_supply_property psp)
{
    switch (psp) {
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
    case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
        return 1;

    default:
        break;
    }

    return -EINVAL;
}

static void max77819_charger_set_current_by_dcin_type(struct max77819_charger* me)
{
    int rc;
    bool do_aicl = false;
    union power_supply_propval val = {0};

    if (!max77819_charger_present_input(me))
    {
        log_info("The dcin cable is removed, do nothing!\n");
        return;
    }

    if (ma77819_charger_is_otg_mode(me))
    {
        log_info("The chip is in otg mode, do nothing!\n");
        return;
    }

    /* get current max value by "usb" power_supply */
    if (me->psy_ext && me->psy_ext->set_property)
    {
        me->psy_ext->get_property(me->psy_ext,
            POWER_SUPPLY_PROP_CURRENT_MAX, &val);
        log_vdbg("get the usb max current %d \n", val.intval);
        /* check dcin currrent, if less than 100mA */
        if (val.intval <= CHG_CURRENT_TYPE_SUSPEND)
        {
            me->current_limit_volatile = CHG_CURRENT_USB_LIMIT_MIN;
            me->charge_current_volatile = CHG_CURRENT_USB_LIMIT_MIN;
        }
        /* check dcin current, whether it is 500mA usb dcin */
        else if (val.intval <= CHG_CURRENT_TYPE_USB)
        {
            me->current_limit_volatile = CHG_CURRENT_USB_CABLE_MAX;
            me->charge_current_volatile = CHG_CURRENT_USB_IBATT_MAX;
        }
        /* check dcin current, whether it is 1100mA usb dcin */
        else
        {
            me->current_limit_volatile = me->current_limit_permanent;
            me->charge_current_volatile = me->charge_current_permanent;
            do_aicl = true;
        }

        if (!do_aicl)
        {
            me->aicl_work_state = CHG_AICL_NONE;
            rc = max77819_charger_set_current_by_temp(me);
            log_dbg("set charge current limit_ma:%d, chgcc_ma:%d \n",
                   me->current_limit_volatile,me->charge_current_volatile);
        }
        else
        {
            me->aicl_work_state = CHG_AICL_WORKING_FALL;
            rc = max77819_charger_set_charge_current(me,
                    me->current_limit_volatile, me->charge_current_volatile);
            /* do acil work to set right dclimt & chgcc current */
            schedule_delayed_work(&me->aicl_work, 0);
        }
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("set charge current error, rc = %d !!\n", rc);
        }
    }
}

static void max77819_charger_aicl_rise_work(struct max77819_charger* me)
{
    int rc;
    int ichgcc = 0;

    if ((CHG_AICL_WORKING_RISE != me->aicl_work_state) ||
        !max77819_charger_allow_to_aicl_rise(me))
    {
        me->aicl_work_state = CHG_AICL_NEED_CHECK;
        return;
    }

    ichgcc = me->charge_current_volatile;
    log_vdbg("current chgcc = %d \n", ichgcc);
    if(!max77819_charger_in_aicl(me))
    {
        me->aicl_work_state = CHG_AICL_WORKING_RISE;
        ichgcc = ichgcc + CHG_CURRENT_AICL_STEP;
        rc = max77819_charger_set_chgcc(me,ichgcc);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("set chgcc current error, rc = %d !! \n", rc);
        }
        else
        {
            log_vdbg("set chgcc %d \n", ichgcc);
            me->charge_current_volatile = ichgcc;
        }
    }
    else
    {
        me->aicl_work_state = CHG_AICL_NEED_CHECK;
        me->charge_current_volatile -= CHG_CURRENT_AICL_STEP; //back to last step
        log_dbg("aicl successed to rise chgcc =%d !! \n", me->charge_current_volatile);
        rc = max77819_charger_set_current_by_temp(me);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("set charge current error, rc = %d !!\n", rc);
        }
    }
}

static void max77819_charger_aicl_fall_work(struct max77819_charger* me)
{
    int rc;
    int ichgcc = 0;

    if ((CHG_AICL_WORKING_FALL != me->aicl_work_state) ||
        !max77819_charger_allow_to_aicl_fall(me))
    {
        me->aicl_work_state = CHG_AICL_NEED_CHECK;
        return;
    }

    ichgcc = me->charge_current_volatile;
    log_vdbg("current chgcc = %d \n", ichgcc);
    if (max77819_charger_in_aicl(me))
    {
        me->aicl_work_state = CHG_AICL_WORKING_FALL;
        ichgcc = ichgcc - CHG_CURRENT_AICL_STEP;
        rc = max77819_charger_set_chgcc(me, ichgcc);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("set chgcc current error, rc = %d !! \n", rc);
        }
        else
        {
            log_vdbg("set chgcc %d \n", ichgcc);
            me->charge_current_volatile = ichgcc;
        }
    }
    else
    {
        me->aicl_work_state = CHG_AICL_NEED_CHECK;
        log_dbg("aicl successed to fall chgcc =%d !! \n", me->charge_current_volatile);
        rc = max77819_charger_set_current_by_temp(me);
        if (unlikely(IS_ERR_VALUE(rc)))
        {
            log_err("set charge current error, rc = %d !!\n", rc);
        }
    }
}

static void max77819_charger_aicl_work(struct work_struct *work)
{
    struct max77819_charger *me =
        container_of(work, struct max77819_charger, aicl_work.work);
    __lock(me);

    if (!max77819_charger_present_input(me)){
        me->aicl_work_state = CHG_AICL_NEED_CHECK;
        goto end_out;
    }

    if (!max77819_charger_buck_mode(me))
    {
        log_vdbg("the buck is disabled, no to do aicl!! \n");
        goto end_out;
    }

    if (CHG_AICL_WORKING_FALL == me->aicl_work_state)
    {
        max77819_charger_aicl_fall_work(me);
    }
    else if (CHG_AICL_WORKING_RISE == me->aicl_work_state)
    {
        max77819_charger_aicl_rise_work(me);
    }

    if (CHG_AICL_NEED_CHECK == me->aicl_work_state ||
        CHG_AICL_NONE == me->aicl_work_state)
    {
        goto end_out;
    }

    loop_schedule_delayed_work(&me->aicl_work,
        msecs_to_jiffies(CHG_CURRENT_AICL_WORK_TIME));
end_out:
    __unlock(me);
}

static void max77819_charger_external_power_changed (struct power_supply *psy)
{
    struct max77819_charger *me =
        container_of(psy, struct max77819_charger, psy);
    struct power_supply *supplicant;
    int i;

    for (i = 0; i < me->pdata->num_supplicants; i++) {
        supplicant = power_supply_get_by_name(me->pdata->supplied_to[i]);
        if (likely(supplicant)) {
            power_supply_changed(supplicant);
        }
    }

    /* set the max current by current of dcin type */
    max77819_charger_set_current_by_dcin_type(me);
}

static enum power_supply_property max77819_charger_psy_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_CHARGE_TYPE,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,     /* charging current */
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, /* input current limit */
    POWER_SUPPLY_PROP_CAPACITY,                    /* maxim --jelphi*/
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_RESUME_CHARGING,
    POWER_SUPPLY_PROP_CHARGE_LOG,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_HOT_IBAT_LIMIT,
};
static void max77819_charger_get_supplied_data(
                struct device *dev, struct device_node *np,
                struct max77819_charger_platform_data *pdata)
{
    char** ppsupps_buf = NULL;
    int num_tmp = 0;
    int i;
    num_tmp = of_property_count_strings(np, "supplied_to");
    if (num_tmp <= 0)	{
        pdata->supplied_to = NULL;
        pdata->num_supplicants = 0;
        return;
    } else {
        ppsupps_buf = devm_kzalloc(
            dev, (num_tmp * sizeof(char*)), GFP_KERNEL);
    }
    
    if (unlikely(!ppsupps_buf)) 
    {
        log_err("supplied data out of memory!!\n");
        pdata->supplied_to = NULL;
        pdata->num_supplicants = 0;
        return;
    }
    else
    {
        for (i = 0; i < num_tmp; i++)
        {
            OF_HW_READ_PROPERTY_IDX_STRING(
                np, "supplied_to", i, ppsupps_buf[i]);
        }
        pdata->supplied_to = ppsupps_buf;
        pdata->num_supplicants = num_tmp;
    }
}

static void max77819_charger_free_supplied_data(
            struct device *dev, 
            struct max77819_charger_platform_data *pdata)
{
    if (pdata->supplied_to)
    {
        devm_kfree(dev, pdata->supplied_to);
    }
    pdata->supplied_to = NULL;
    pdata->num_supplicants = 0;
}
static void max77819_charger_get_dts_platdata (struct max77819_charger *me)
{
    struct device_node *np = me->dev->of_node;
    struct max77819_charger_platform_data *pdata;

    pdata = &g_platform_data;

    OF_HW_READ_PROPERTY_STRING(np,
        "psy_name", pdata->psy_name);

    OF_HW_READ_PROPERTY_STRING(np,
        "ext_psy_name", pdata->ext_psy_name);

    max77819_charger_get_supplied_data(me->dev, np, pdata);

    OF_HW_READ_PROPERTY_VAL(np,
        "chg_dcin_current_max", pdata->chg_dcin_current_max);
    pdata->chg_dcin_current_max = min(pdata->chg_dcin_current_max, (u32)CHG_CURRENT_DCIN_MAX);

    OF_HW_READ_PROPERTY_VAL(np,
        "chg_fast_current_max", pdata->chg_fast_current_max);
    pdata->chg_fast_current_max = min(pdata->chg_fast_current_max, (u32)CHG_CURRENT_FCC_MAX);

    OF_HW_READ_PROPERTY_VAL(np,
        "charge_termination_voltage", pdata->charge_termination_voltage);

    OF_HW_READ_PROPERTY_VAL(np,
        "topoff_timer", pdata->topoff_timer);
    pdata->topoff_timer = min((u32)CHG_TOPOFF_SOC_TIME_MAX, pdata->topoff_timer);

    OF_HW_READ_PROPERTY_VAL(np,
        "topoff_current", pdata->topoff_current);

    OF_HW_READ_PROPERTY_VAL(np,
        "charge_restart_threshold", pdata->charge_restart_threshold);

    OF_HW_READ_PROPERTY_BOOL(np, 
        "enable_coop", pdata->enable_coop);

    if (likely(pdata->enable_coop)) {
        OF_HW_READ_PROPERTY_STRING(np,
            "coop_psy_name", pdata->coop_psy_name);
    }

    OF_HW_READ_PROPERTY_BOOL(np, 
        "enable_thermistor", pdata->enable_thermistor);

    OF_HW_READ_PROPERTY_BOOL(np, 
        "enable_aicl", pdata->enable_aicl);

    OF_HW_READ_PROPERTY_VAL(np,
        "aicl_detection_voltage", pdata->aicl_detection_voltage);

    OF_HW_READ_PROPERTY_VAL(np,
        "aicl_reset_threshold", pdata->aicl_reset_threshold);

    me->pdata = pdata;
}

static void max77819_charger_get_platdata (struct max77819_charger *me)
{
#ifdef CONFIG_OF
    max77819_charger_get_dts_platdata(me);
#else /* CONFIG_OF */
    me->pdata = dev_get_platdata(me->dev) ?
        dev_get_platdata(me->dev) : ERR_PTR(-EINVAL);
#endif /* CONFIG_OF */
}

static int max77819_charger_get_temp_param(struct max77819_charger *me)
{
    int rc = 0;
    struct device *dev = me->dev;
    struct device_node *np = dev->of_node;
    int vbatt_id; 

    vbatt_id= maxim_get_battery_id_uv();
    rc = of_batterydata_read_tempctrl_info_maxim(np, &g_temp_ctrl_info, vbatt_id);
    me->ptemp_ctrl_info = &g_temp_ctrl_info;

    return rc;
}

const struct max77819_temp_control_info* max77819_charger_get_temp_ctrl_info(void)
{
    if (!global_Charger)
    {
        log_info("max77819 chip device not init,do nothing!\n");
        return NULL;
    }
    return global_Charger->ptemp_ctrl_info;
}
EXPORT_SYMBOL(max77819_charger_get_temp_ctrl_info);

static __always_inline
void max77819_charger_destroy (struct max77819_charger *me)
{
    struct device *dev = me->dev;

    cancel_delayed_work_sync(&me->timer_work);
    /* remove topoff_work cancle */
    cancel_delayed_work_sync(&me->aicl_work);

    cancel_delayed_work_sync(&me->log_work);

    // remove irq for dcin interrupt

    cancel_delayed_work_sync(&me->irq_work);

    if (likely(me->attr_grp)) {
        sysfs_remove_group(me->kobj, me->attr_grp);
    }

    if (likely(me->psy_this)) {
        power_supply_unregister(me->psy_this);
    }

#ifdef CONFIG_OF
    max77819_charger_free_supplied_data(me->dev, me->pdata);
#endif /* CONFIG_OF */

    mutex_destroy(&me->lock);
//  spin_lock_destroy(&me->irq_lock);

    devm_kfree(dev, me);
}

#ifdef CONFIG_OF
static struct of_device_id max77819_charger_of_ids[] = {
    { .compatible = "maxim,"MAX77819_CHARGER_NAME },
    { },
};
MODULE_DEVICE_TABLE(of, max77819_charger_of_ids);
#endif /* CONFIG_OF */

static int max77819_charger_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77819_dev *chip = dev_get_drvdata(dev->parent);
    struct max77819_charger *me;
    int rc = 0;

    log_dbg("attached\n");

    /* make sure this driver probe after the 'usb' driver */
    if (NULL == power_supply_get_by_name(POWR_SUPPLY_USB_NAME))
    {
        log_err("USB supply not found; deferring charger probe !!\n");
        return -EPROBE_DEFER;
    }

    me = devm_kzalloc(dev, sizeof(*me), GFP_KERNEL);
    if (unlikely(!me)) {
        log_err("out of memory (%uB requested)\n", sizeof(*me));
        return -ENOMEM;
    }
    global_Charger = me;
    dev_set_drvdata(dev, me);

    // remove irq && irq_lock for dcin interrupt
    mutex_init(&me->lock);
    me->io   = max77819_get_io(chip);
    me->dev  = dev;
    me->kobj = &dev->kobj;
    me->dev_initialized = false;
    /* init the chip otg mode false */
    me->dev_otg_mode = false;

    INIT_DELAYED_WORK(&me->irq_work, max77819_charger_irq_work);
    INIT_DELAYED_WORK(&me->log_work, max77819_charger_log_work);
    INIT_DELAYED_WORK(&me->aicl_work, max77819_charger_aicl_work);

    /* remove topoff_work init */
    INIT_DELAYED_WORK(&me->timer_work, max77819_charger_timer_work);

    max77819_charger_get_platdata(me);
    max77819_charger_get_temp_param(me);
    me->bfactory_daig_chg = true;
    me->resuming_charging = false;
    me->bterm_chg_by_volt = false;
    me->bterm_chg_by_soc = false;
    me->bterm_chg_pause = false;
    me->stop_chg_time_out = CHG_TIMER_NONE;
    me->aicl_work_state = CHG_AICL_NONE;
    me->btemp_chg_protecting = false;

    /* disable all IRQ */
    max77819_write(me->io, CHGINTM1,   0xFF);
    max77819_write(me->io, CHGINTMSK2, 0xFF);

    me->dev_enabled = true;
    me->current_limit_permanent   = me->pdata->chg_dcin_current_max;
    me->charge_current_permanent  = me->pdata->chg_fast_current_max;
    me->current_limit_volatile    = me->current_limit_permanent;
    me->charge_current_volatile   = me->charge_current_permanent;

    me->present = max77819_charger_present_input(me);

    me->psy.name                   = "max77819-charger";
    me->psy.type                   = POWER_SUPPLY_TYPE_BATTERY;
    me->psy.supplied_to            = me->pdata->supplied_to;
    me->psy.num_supplicants        = me->pdata->num_supplicants;
    me->psy.properties             = max77819_charger_psy_props;
    me->psy.num_properties         = ARRAY_SIZE(max77819_charger_psy_props);
    me->psy.get_property           = max77819_charger_get_property;
    me->psy.set_property           = max77819_charger_set_property;
    me->psy.property_is_writeable  = max77819_charger_property_is_writeable;
    me->psy.external_power_changed = max77819_charger_external_power_changed;

    max77819_charger_psy_init(me);

    rc = power_supply_register(dev, me->psy_this);
    if (unlikely(IS_ERR_VALUE(rc))) {
        log_err("failed to register power_supply class [%d]\n", rc);
        me->psy_this = NULL;
        goto abort;
    }

    // remove irq for dcin interrupt

    /* enable IRQ we want */
    max77819_write(me->io, CHGINTM1, (u8)~CHGINT1_DC_UVP);

    log_info("driver "DRIVER_VERSION" installed\n");

    /* check the otg mode */
    max77819_charger_set_otg_by_dcin_type(me);

    if (likely(me->present && !ma77819_charger_is_otg_mode(me)))
    {
        schedule_delayed_work(&me->irq_work, IRQ_WORK_DELAY);
    }
    /* schedule timer_work in the irq_work */
    max77819_charger_resume_log_work(me);
    return 0;

abort:
    dev_set_drvdata(dev, NULL);
    max77819_charger_destroy(me);
    return rc;
}

static int max77819_charger_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77819_charger *me = dev_get_drvdata(dev);

    log_dbg("remove \n");
    /* close the otg mode */
    max77819_charger_otg_control(me, false);

    dev_set_drvdata(dev, NULL);
    max77819_charger_destroy(me);

    return 0;
}

static void max77819_charger_shutdown(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77819_charger *me = dev_get_drvdata(dev);

    log_dbg("shutdown \n");

    /* close the otg mode */
    max77819_charger_otg_control(me, false);

}

#ifdef CONFIG_PM_SLEEP
static int max77819_charger_suspend (struct device *dev)
{
    struct max77819_charger *me = dev_get_drvdata(dev);

    log_vdbg("suspending\n");
    max77819_charger_suspend_log_work(me);

    return 0;
}

static int max77819_charger_resume (struct device *dev)
{
    struct max77819_charger *me = dev_get_drvdata(dev);

    log_vdbg("resuming\n");
    max77819_charger_resume_log_work(me);

    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77819_charger_pm, max77819_charger_suspend,
    max77819_charger_resume);

static struct platform_driver max77819_charger_driver =
{
    .driver.name            = DRIVER_NAME,
    .driver.owner           = THIS_MODULE,
    .driver.pm              = &max77819_charger_pm,
#ifdef CONFIG_OF
    .driver.of_match_table  = max77819_charger_of_ids,
#endif /* CONFIG_OF */
    .probe                  = max77819_charger_probe,
    .remove                 = max77819_charger_remove,
    .shutdown               = max77819_charger_shutdown,
};

static __init int max77819_charger_init (void)
{
    return platform_driver_register(&max77819_charger_driver);
}
late_initcall(max77819_charger_init);

static __exit void max77819_charger_exit (void)
{
    platform_driver_unregister(&max77819_charger_driver);
}
module_exit(max77819_charger_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
