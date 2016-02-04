#
# Makefile for the Synaptics DSX touchscreen driver.
#

# Each configuration option enables a list of files.

#/*Add synaptics new driver "Synaptics DSX I2C V2.0"*/
obj-$(CONFIG_MACH_HUAWEI) += hw_tp_common.o
obj-$(CONFIG_TOUCHSCREEN_HUAWEI_DSX_I2C) += synaptics_dsx_i2c.o synaptics_dsx_hw_i2c_ctl.o
obj-$(CONFIG_TOUCHSCREEN_HUAWEI_DSX_RMI_DEV) += synaptics_dsx_rmi_dev.o
obj-$(CONFIG_TOUCHSCREEN_HUAWEI_DSX_FW_UPDATE) += synaptics_dsx_fw_update.o
obj-$(CONFIG_TOUCHSCREEN_HUAWEI_DSX_TEST_REPORTING) += synaptics_dsx_test_reporting.o
obj-$(CONFIG_TOUCHSCREEN_HUAWEI_DSX_PROXIMITY) += synaptics_dsx_proximity.o
obj-$(CONFIG_TOUCHSCREEN_HUAWEI_DSX_ESD) += synaptics_dsx_esd.o
