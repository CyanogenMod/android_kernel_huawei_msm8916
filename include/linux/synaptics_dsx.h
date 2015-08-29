/*Add synaptics new driver "Synaptics DSX I2C V2.0"*/
/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _SYNAPTICS_DSX_H_
#define _SYNAPTICS_DSX_H_

#ifndef CONFIG_HUAWEI_KERNEL

/*
 * struct synaptics_rmi4_capacitance_button_map - 0d button map
 * @nbuttons: number of buttons
 * @map: button map
 */
struct synaptics_rmi4_capacitance_button_map {
	unsigned char nbuttons;
	unsigned char *map;
};

/*
 * struct synaptics_rmi4_platform_data - rmi4 platform data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @i2c_pull_up: pull up i2c bus with regulator
 * @power_down_enable: enable complete regulator shutdown in suspend
 * @irq_gpio: attention interrupt gpio
 * @irq_flags: flags used by the irq
 * @reset_flags: flags used by reset line
 * @reset_gpio: reset gpio
 * @panel_x: panel maximum values on the x
 * @panel_y: panel maximum values on the y
 * @disp_maxx: display panel maximum values on the x
 * @disp_maxy: display panel maximum values on the y
 * @disp_minx: display panel minimum values on the x
 * @disp_miny: display panel minimum values on the y
 * @panel_maxx: touch panel maximum values on the x
 * @panel_maxy: touch panel maximum values on the y
 * @panel_minx: touch panel minimum values on the x
 * @panel_miny: touch panel minimum values on the y
 * @reset_delay: reset delay
 * @gpio_config: pointer to gpio configuration function
 * @capacitance_button_map: pointer to 0d button map
 */
struct synaptics_rmi4_platform_data {
	bool x_flip;
	bool y_flip;
	bool i2c_pull_up;
	bool power_down_enable;
	bool disable_gpios;
	bool do_lockdown;
	unsigned irq_gpio;
	u32 irq_flags;
	u32 reset_flags;
	unsigned reset_gpio;
	unsigned panel_minx;
	unsigned panel_miny;
	unsigned panel_maxx;
	unsigned panel_maxy;
	unsigned disp_minx;
	unsigned disp_miny;
	unsigned disp_maxx;
	unsigned disp_maxy;
	unsigned reset_delay;
	const char *fw_image_name;
	int (*gpio_config)(unsigned gpio, bool configure);
	struct synaptics_rmi4_capacitance_button_map *capacitance_button_map;
};

#else/*CONFIG_HUAWEI_KERNEL*/

#ifdef CONFIG_HUAWEI_KERNEL
#define ENABLE_VIRTUAL_KEYS
#endif /*CONFIG_HUAWEI_KERNEL*/

/*
 * synaptics_dsx_cap_button_map - 0d button map
 * @nbuttons: number of 0d buttons
 * @map: pointer to array of button types
 */
struct synaptics_dsx_cap_button_map {
	unsigned char nbuttons;
	unsigned char *map;
};

#ifdef CONFIG_HUAWEI_KERNEL
#ifdef ENABLE_VIRTUAL_KEYS
struct syanptics_virtual_keys {
	struct kobj_attribute kobj_attr;
	u16 *data;
	int size;
};
#endif

struct syanptics_wakeup_keys {
	const uint16_t  *keys;
	uint8_t         size;
	uint8_t         enable_wkeys;/*just padding, not used*/
} __packed;

enum syanptics_gesture_flags {
	RMI4_EASY_WAKE_UP_GESTURE_DISABLED = 0x00,
	RMI4_EASY_WAKE_UP_GESTURE_ENABLED = 0x01,
};

enum syanptics_palm_flags {
	RMI4_PALM_SLEEP_GESTURE_DISABLED = 0x00,
	RMI4_PALM_SLEEP_GESTURE_ENABLED = 0x01,
};

enum syanptics_glove_mode {
	SYSTEM_START_IN_SKIN_MODE = 0,       //TP starts in skin mode by default,but can auto switch with glove mode.
	SYSTEM_START_IN_GLOVE_MODE = 1,    //TP starts in glove mode by default,but can auto switch with skin mode
	SYSTEM_LOCKED_TO_SKIN_MODE = 2,    //TP is locked to skin mode 
	SYSTEM_LOCKED_TO_GLOVE_MODE = 3, //TP is locked to glove mode
};

enum syanptics_glove_flags {
	RMI4_GLOV_FUNC_DISABLED = 0x00,
	RMI4_GLOV_FUNC_ENABLED = 0x01,
};

enum syanptics_holster_flags {
	RMI4_HOLSTER_FUNC_DISABLED = 0x00,
	RMI4_HOLSTER_FUNC_ENABLED = 0x01,
};
#endif /*CONFIG_HUAWEI_KERNEL*/

/*
 * struct synaptics_dsx_platform_data - dsx platform data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @irq_gpio: attention interrupt gpio
 * @power_gpio: power switch gpio
 * @power_on_state: power switch active state
 * @reset_gpio: reset gpio
 * @reset_on_state: reset active state
 * @irq_flags: irq flags
 * @panel_x: x-axis resolution of display panel
 * @panel_y: y-axis resolution of display panel
 * @power_delay_ms: delay time to wait after power-on
 * @reset_delay_ms: delay time to wait after reset
 * @reset_active_ms: reset active time
 * @regulator_name: pointer to name of regulator
 * @gpio_config: pointer to gpio configuration function
 * @cap_button_map: pointer to 0d button map
 */
struct synaptics_dsx_platform_data {
	bool x_flip;
	bool y_flip;
	bool swap_axes;
	bool reg_en;
	unsigned char grip_left_lsb;
	unsigned char grip_left_msb;
	unsigned char grip_right_lsb;
	unsigned char grip_right_msb;
	int irq_gpio;
	int power_gpio;
	int power_on_state;
	int reset_gpio;
	int reset_on_state;
	unsigned long irq_flags;
	unsigned int panel_x;
	unsigned int panel_y;
	unsigned int power_delay_ms;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
#ifdef CONFIG_HUAWEI_KERNEL
	unsigned char *regulator_vdd;
	unsigned char *regulator_vbus;
	unsigned int lcd_x;
	unsigned int lcd_y;
	unsigned int lcd_all;
	/* defination of double tap gesture zone */
	int dtz_x0;
	int dtz_y0;
	int dtz_x1;
	int dtz_y1;
	const char *product_name;
#ifdef ENABLE_VIRTUAL_KEYS
	struct syanptics_virtual_keys vkeys;
#endif
	bool gesture_enabled;
	bool glove_enabled;
	unsigned int easy_wakeup_supported_gestures;
	bool glove_supported;
	bool esd_support;
	bool low_power_support;
	bool glove_edge_switch_supported;
	bool grip_algorithm_supported;
	bool grip_algorithm_enabled;
	struct syanptics_wakeup_keys *wakeup_keys;
	unsigned char fast_relax_gesture;
	/*Add doze wakeup threshold param for tempreture performance*/
	unsigned char ofilm_dwt;
	unsigned char junda_dwt;
	unsigned char truly_dwt;
	unsigned char eely_dwt;
	unsigned char lensone_dwt;
	unsigned char gis_dwt;
	unsigned char yassy_dwt;
	bool holster_supported;
#endif /*CONFIG_HUAWEI_KERNEL*/
	int (*gpio_config)(int gpio, bool configure, int dir, int state);
	struct synaptics_dsx_cap_button_map *cap_button_map;
};
#ifdef CONFIG_HUAWEI_DSM
ssize_t synaptics_dsm_record_fw_err_info( int err_numb );
ssize_t synaptics_dsm_f34_pdt_err_info( int err_numb );
ssize_t synaptics_dsm_f54_pdt_err_info( int err_numb );
ssize_t synaptics_dsm_f34_read_queries_err_info( int err_numb );
ssize_t synaptics_dsm_fwu_init_pdt_props_err_info( int err_numb );
int synp_tp_report_dsm_err( int type, int err_numb );
struct dsm_client* tp_dsm_get_client(void);
#endif
#endif /*CONFIG_HUAWEI_KERNEL*/
#endif
