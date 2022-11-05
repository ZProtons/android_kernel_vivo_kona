# SPDX-License-Identifier: GPL-2.0-only
ifeq ($(TARGET_PRODUCT), lito)
$(warning  VIVO_CHG[SM7250]: $(BBK_PRODUCT_PLATFORM) code 1..)
obj-$(CONFIG_BATTERY_DETECT)   += sm7250/charger-monitor-v2.o
obj-$(CONFIG_QPNP_SMB5)			+= sm7250/step-chg-jeita.o sm7250/battery.o sm7250/vivo-qpnp-smb5.o sm7250/vivo-smb5-lib.o sm7250/pmic-voter.o sm7250/storm-watch.o sm7250/schgm-flash.o
obj-$(CONFIG_SMB1390_CHARGE_PUMP_PSY)	+= sm7250/smb1390-charger-psy.o sm7250/pmic-voter.o
obj-$(CONFIG_SMB1355_SLAVE_CHARGER)	+= sm7250/smb1355-charger.o sm7250/pmic-voter.o
obj-$(CONFIG_QPNP_QNOVO5)		+= sm7250/qpnp-qnovo5.o sm7250/battery.o sm7250/pmic-voter.o
obj-$(CONFIG_QPNP_FG_GEN4)		+= sm7250/qpnp-fg-gen4.o sm7250/fg-memif.o sm7250/fg-util.o sm7250/fg-alg.o sm7250/pmic-voter.o
obj-$(CONFIG_QPNP_QG)			+= sm7250/vivo-qpnp-qg.o sm7250/pmic-voter.o sm7250/qg-util.o sm7250/vivo-qg-soc.o sm7250/qg-sdam.o sm7250/qg-battery-profile.o sm7250/qg-profile-lib.o sm7250/fg-alg.o
obj-$(CONFIG_HL6111R)			+= sm7250/hl6111r.o

else ifeq ($(TARGET_PRODUCT), kona)
$(warning  VIVO_CHG[kona SM8250]: $(BBK_PRODUCT_PLATFORM) code 1..)
obj-$(CONFIG_BATTERY_DETECT)   += sm8250/charger-monitor-v2.o sm8250/charger-monitor-v3.o
obj-$(CONFIG_BOOST_CHARGER_BQ25790)	+= sm8250/vivo_bq25790_boost_charger.o
obj-$(CONFIG_DIALOG_DA9313_DIVIDER)	+= sm8250/da9313_div2.o
obj-$(CONFIG_QPNP_SMB5)			+= sm8250/step-chg-jeita.o sm8250/battery.o sm8250/vivo-qpnp-smb5.o sm8250/vivo-smb5-lib.o sm8250/pmic-voter.o sm8250/storm-watch.o sm8250/schgm-flash.o
obj-$(CONFIG_SMB1390_CHARGE_PUMP_PSY)	+= sm8250/smb1390-charger-psy.o sm8250/pmic-voter.o
obj-$(CONFIG_SMB1355_SLAVE_CHARGER)	+= sm8250/smb1355-charger.o sm8250/pmic-voter.o
obj-$(CONFIG_SMB1351_USB_CHARGER)	+= smb1351-charger.o pmic-voter.o battery.o
obj-$(CONFIG_QPNP_SMB2)			+= sm8250/step-chg-jeita.o sm8250/battery.o sm8250/qpnp-smb2.o sm8250/smb-lib.o sm8250/pmic-voter.o sm8250/storm-watch.o
obj-$(CONFIG_SMB138X_CHARGER)		+= sm8250/step-chg-jeita.o sm8250/smb138x-charger.o sm8250/smb-lib.o sm8250/pmic-voter.o sm8250/storm-watch.o sm8250/battery.o
obj-$(CONFIG_QPNP_FG_GEN3)		+= sm8250/qpnp-fg-gen3.o sm8250/fg-memif.o sm8250/fg-util.o
obj-$(CONFIG_QPNP_QNOVO5)		+= sm8250/qpnp-qnovo5.o sm8250/battery.o sm8250/pmic-voter.o
obj-$(CONFIG_QPNP_FG_GEN4)		+= sm8250/vivo-qpnp-fg-gen4.o sm8250/fg-memif.o sm8250/fg-util.o sm8250/fg-alg.o sm8250/pmic-voter.o
obj-$(CONFIG_QPNP_QG)			+= sm8250/qpnp-qg.o sm8250/pmic-voter.o sm8250/qg-util.o sm8250/qg-soc.o sm8250/qg-sdam.o sm8250/qg-battery-profile.o sm8250/qg-profile-lib.o sm8250/fg-alg.o
obj-$(CONFIG_HL6111R)			+= sm8250/hl6111r.o
obj-$(CONFIG_SMB1398_CHARGER)		+= sm8250/smb1398-charger.o sm8250/pmic-voter.o
obj-$(CONFIG_QPNP_SMBLITE)		+= sm8250/step-chg-jeita.o sm8250/battery.o sm8250/qpnp-smblite.o sm8250/smblite-lib.o sm8250/pmic-voter.o sm8250/storm-watch.o sm8250/schgm-flashlite.o
$(warning  VIVO_CHG: get the project base name : project_name=$(BBK_PRODUCT_MODEL).......)

# sxs 20201111 delete because android11  $(BBK_PRODUCT_MODEL)=null
#ccflags-y += -DVIVO_PROJECT_BASE_NAME="$(BBK_PRODUCT_MODEL)"
#ifeq ($(findstring PD2059,$(BBK_PRODUCT_MODEL)),PD2059)
#	ccflags-y += -DVIVO_BURN_FW_HV_H="vivo-burn_fw.h"
#else ifeq ($(findstring PD2059F_EX,$(BBK_PRODUCT_MODEL)),PD2059F_EX)
#	ccflags-y += -DVIVO_BURN_FW_HV_H="vivo-burn_fw.h" 
#else
#	ccflags-y += -DVIVO_BURN_FW_HV_H="vivo-burn_fw.h"
#endif

else
$(warning  VIVO_CHG: $(BBK_PRODUCT_PLATFORM) code 1..)
obj-$(CONFIG_QPNP_SMB5)			+= step-chg-jeita.o battery.o qpnp-smb5.o smb5-lib.o pmic-voter.o storm-watch.o schgm-flash.o
obj-$(CONFIG_SMB1390_CHARGE_PUMP_PSY)	+= smb1390-charger-psy.o pmic-voter.o
obj-$(CONFIG_SMB1355_SLAVE_CHARGER)	+= smb1355-charger.o pmic-voter.o
obj-$(CONFIG_SMB1351_USB_CHARGER)	+= smb1351-charger.o pmic-voter.o battery.o
obj-$(CONFIG_QPNP_SMB2)			+= step-chg-jeita.o battery.o qpnp-smb2.o smb-lib.o pmic-voter.o storm-watch.o
obj-$(CONFIG_SMB138X_CHARGER)		+= step-chg-jeita.o smb138x-charger.o smb-lib.o pmic-voter.o storm-watch.o battery.o
obj-$(CONFIG_QPNP_FG_GEN3)		+= qpnp-fg-gen3.o fg-memif.o fg-util.o
obj-$(CONFIG_QPNP_QNOVO5)		+= qpnp-qnovo5.o battery.o pmic-voter.o
obj-$(CONFIG_QPNP_FG_GEN4)		+= qpnp-fg-gen4.o fg-memif.o fg-util.o fg-alg.o pmic-voter.o
obj-$(CONFIG_QPNP_QG)			+= qpnp-qg.o pmic-voter.o qg-util.o qg-soc.o qg-sdam.o qg-battery-profile.o qg-profile-lib.o fg-alg.o
obj-$(CONFIG_HL6111R)			+= hl6111r.o
obj-$(CONFIG_SMB1398_CHARGER)		+= smb1398-charger.o pmic-voter.o
obj-$(CONFIG_QPNP_SMBLITE)		+= step-chg-jeita.o battery.o qpnp-smblite.o smblite-lib.o pmic-voter.o storm-watch.o schgm-flashlite.o
endif