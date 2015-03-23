#
# (C) Copyright 2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# SPDX-License-Identifier:	GPL-2.0+
#

MINIMAL=

ifdef CONFIG_SPL_BUILD
ifdef CONFIG_SPL_INIT_MINIMAL
MINIMAL=y
endif
endif

ifdef MINIMAL
# necessary to create built-in.o
obj- := __dummy__.o
else
obj-$(CONFIG_FSL_CADMUS)	+= cadmus.o
obj-$(CONFIG_FSL_VIA)		+= cds_via.o
obj-$(CONFIG_FMAN_ENET)	+= fman.o
obj-$(CONFIG_FSL_PIXIS)	+= pixis.o
ifndef CONFIG_SPL_BUILD
obj-$(CONFIG_FSL_NGPIXIS)	+= ngpixis.o
endif
obj-$(CONFIG_FSL_QIXIS)	+= qixis.o
obj-$(CONFIG_PQ_MDS_PIB)	+= pq-mds-pib.o
ifndef CONFIG_SPL_BUILD
obj-$(CONFIG_ID_EEPROM)	+= sys_eeprom.o
endif
obj-$(CONFIG_FSL_SGMII_RISER)	+= sgmii_riser.o
ifndef CONFIG_RAMBOOT_PBL
obj-$(CONFIG_FSL_FIXED_MMC_LOCATION)	+= sdhc_boot.o
endif

ifdef CONFIG_FASTBOOT
obj-${CONFIG_ANDROID_RECOVERY} += recovery.o
endif

endif
