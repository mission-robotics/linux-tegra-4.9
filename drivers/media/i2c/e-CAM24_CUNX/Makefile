ifneq (,$(filter $(CONFIG_VIDEO_ECAM),y m))
	ccflags-y += -I$(srctree)/drivers/media/platform/tegra/
	obj-m	+= ar0234.o
endif

ar0234:
	make -C $(KERNEL_PATH) M=$(PWD) modules

clean:
	make -C $(KERNEL_PATH) M=$(PWD) clean
