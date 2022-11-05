ifeq ($(CONFIG_VIVO_TS),)
  CONFIG_VIVO_TS=y
endif

obj-$(CONFIG_VIVO_TS) += vivo_ts.o

vivo_ts-y += vts_core.o
vivo_ts-y += vts_node.o
vivo_ts-y += vts_incell.o
vivo_ts-y += vts_class.o
vivo_ts-y += vts_report.o
vivo_ts-y += vts_op.o
vivo_ts-y += vts_spi_drv.o

vivo_ts-y += $(addprefix samsung_y761/, \
				sec_ts.o \
				sec_ts_fn.o \
				sec_ts_fw.o \
				sec_ts_only_vendor.o \
			)


vivo_ts-y += vts_init.o
