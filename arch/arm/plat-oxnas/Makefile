#
# Makefile for the linux kernel.
#

obj-y :=
obj-m :=
obj-n :=
obj-  :=

obj-y += dma.o
obj-y += leon.o
obj-y += samba_reserve.o
obj-y += filter_misc.o

## -> [Walker Chen], 2010/07/21 - dome button test
#obj-m +=button-demo.o
## <-	End.
## -> [Walker Chen], 2010/07/21 - 7715 Vbus Detect
#obj-y +=vbus_button.o
## <-	End.
obj-y += btns_driver.o

obj-y += prealloc_init.o

obj-$(CONFIG_OXNAS_FAST_OPEN_FILTER) += fast_open_filter.o

obj-$(CONFIG_OXNAS_ZERO_COPY_RX_SUPPORT) += oxnas_net.o

obj-$(CONFIG_OXNAS_FAST_READS_AND_WRITES) += incoherent_sendfile.o

obj-$(CONFIG_OXNAS_FAST_WRITES_SUPPORT) += entry.o
obj-$(CONFIG_OXNAS_FAST_WRITES_SUPPORT) += oxnas_direct_net_to_disk.o
obj-$(CONFIG_OXNAS_FAST_WRITES_SUPPORT) += oxnas_direct_disk.o

obj-$(CONFIG_OXNAS_FAST_WRITES) += direct_writes.o

obj-$(CONFIG_TACHO_THERM_AND_FAN)		+= thermAndFan.o
obj-$(CONFIG_TACHO_THERM_AND_FAN_820)		+= thermAndFan-820.o
obj-$(CONFIG_OXNAS_AHB_MONITOR_MODULE)		+= oxnas-ahb-monitor.o
obj-$(CONFIG_OXNAS_PHY_TEMPERATURE_MODULE)	+= phy-temp-monitor.o
obj-$(CONFIG_I2S)				+= i2s.o
obj-$(CONFIG_OXNAS_USB_TEST_MODES)		+= usb-test-mode.o
obj-$(CONFIG_LEON_POWER_BUTTON_MONITOR)		+= power_button.o
obj-$(CONFIG_USER_RECOVERY_BUTTON_MONITOR)	+= user_recovery_button.o
obj-$(CONFIG_LEON_WATCHDOG_MODULE)		+= watchdog.o
obj-$(CONFIG_VENDOR_SCSI_SMD_IF_MODULE)		+= vendor_scsi.o
obj-$(CONFIG_OXNAS_GPIO_TEST)			+= ox820_gpio.o
obj-$(CONFIG_MODE_SWITCH_MONITOR)		+= switch-mode.o
