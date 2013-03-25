#############################################################
#
# linux
#
#############################################################

ifdef RAPTOR_DIR
LINUX_DIR:=$(RAPTOR_DIR)/kernel/omap
else
LINUX_DIR:=$(PACKAGE_DIR)/omap
endif

LINUX_EXPORTED_HEADERS := \
	$(LINUX_DIR)/drivers/parrot/char/pwm/pwm_ioctl.h \
	$(LINUX_DIR)/drivers/parrot/char/gpio2_ioctl.h

ifndef LINUXCONF_MK
	$(error LINUXCONF_MK is not defined)
endif

include $(LINUXCONF_MK)

#LINUX_MAKE_ARGS:=$(subst $(TARGET_CROSS),/opt/arm-2009q1/bin/arm-none-linux-gnueabi-, $(LINUX_MAKE_ARGS))

LINUX_EXTRA_CMD=$(Q) $(MAKE) $(LINUX_MAKE_ARGS) uImage V=$(EXTRA_V); \
$(Q) cp $(LINUX_DIR_BUILD)/arch/arm/boot/uImage $(BIN_DIR)/

