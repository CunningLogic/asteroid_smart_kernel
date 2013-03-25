EXTRA_CFLAGS += -DFPNUM='"66"' -DREASSOCIATION -DUAP_SUPPORT -D STA_SUPPORT -DWFD_SUPPORT	\
		-DSDIO_MULTI_PORT_TX_AGGR -DSDIO_MULTI_PORT_RX_AGGR \
		-DDFS_TESTING_SUPPORT -I$(src)/mlan

ifeq ($(CONFIG_88W8787_WLAN_REASSOCIATION),y)
EXTRA_CFLAGS += -DREASSOCIATION
endif

ifeq ($(CONFIG_88W8787_WLAN_MFG_CMD_SUPPORT),y)
EXTRA_CFLAGS +=-DMFG_CMD_SUPPORT
endif

ifeq ($(CONFIG_88W8787_WLAN_DEBUG),y)
EXTRA_CFLAGS += -O0 -DDEBUG_LEVEL1
endif

obj-$(CONFIG_88W8787_WLAN) += 88w8787_wlan.o

MOALOBJS =	mlinux/moal_main.o \
		mlinux/moal_ioctl.o \
		mlinux/moal_shim.o \
		mlinux/moal_sdio_mmc.o

MLANOBJS =	mlan/mlan_shim.o mlan/mlan_init.o \
		mlan/mlan_txrx.o \
		mlan/mlan_cmdevt.o mlan/mlan_misc.o \
		mlan/mlan_module.o

MLANOBJS +=	mlan/mlan_wmm.o
MLANOBJS +=	mlan/mlan_sdio.o
MLANOBJS +=	mlan/mlan_11n_aggr.o
MLANOBJS +=	mlan/mlan_11n_rxreorder.o
MLANOBJS +=	mlan/mlan_11n.o
MLANOBJS +=	mlan/mlan_11d.o
MLANOBJS +=	mlan/mlan_11h.o

#ifeq ($(CONFIG_STA_SUPPORT),y)
MLANOBJS +=	mlan/mlan_meas.o
MLANOBJS +=	mlan/mlan_cfp.o \
		mlan/mlan_scan.o \
		mlan/mlan_sta_ioctl.o \
		mlan/mlan_sta_rx.o \
		mlan/mlan_sta_tx.o \
		mlan/mlan_sta_event.o \
		mlan/mlan_sta_cmd.o \
		mlan/mlan_sta_cmdresp.o \
		mlan/mlan_join.o

MOALOBJS +=	mlinux/moal_priv.o \
		mlinux/moal_wext.o
#endif
#ifeq ($(CONFIG_UAP_SUPPORT),y)
MLANOBJS +=	mlan/mlan_uap_ioctl.o
MLANOBJS +=	mlan/mlan_uap_cmdevent.o
MLANOBJS +=	mlan/mlan_uap_txrx.o
MOALOBJS +=	mlinux/moal_uap.o
MOALOBJS +=	mlinux/moal_uap_priv.o
MOALOBJS +=	mlinux/moal_uap_wext.o
MLANOBJS +=	mlan/mlan_cfp.o
MOALOBJS +=	mlinux/moal_wext.o
#endif

ifdef CONFIG_PROC_FS
MOALOBJS += mlinux/moal_proc.o
ifeq ($(CONFIG_88W8787_WLAN_PROC_DEBUG),y)
MOALOBJS += mlinux/moal_debug.o
EXTRA_CFLAGS += -DPROC_DEBUG
endif
endif

WLANOBJS += $(MLANOBJS) $(MOALOBJS)

88w8787_wlan-objs := $(WLANOBJS)
