#ifndef __ADAPTER_TP__
#define __ADAPTER_TP__

#include <linux/atomic.h>

extern atomic_t touch_detected_yet;
extern struct touch_hw_platform_data touch_hw_data;

struct touch_hw_platform_data
{
        int (*touch_power)(int on);     /* Only valid in first array entry */
        int (*set_touch_interrupt_gpio)(void);/*it will config the gpio*/
        void (*set_touch_probe_flag)(int detected);/*we use this to detect the probe is detected*/
        int (*read_touch_probe_flag)(void);/*when the touch is find we return a value*/
        int (*touch_reset)(void);
        int (*get_touch_reset_gpio)(void);
       // int (*get_touch_resolution)(struct tp_resolution_conversion *tp_resolution_type);/*add this function for judge the tp type*/
        int (*read_button_flag)(void);
       // int (*get_touch_button_map)(struct tp_button_map *tp_button_map);
};

#endif
