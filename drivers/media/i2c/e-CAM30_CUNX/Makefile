ifneq (,$(filter $(CONFIG_VIDEO_ECAM),y m))
	ccflags-y += -I$(srctree)/drivers/media/platform/tegra/ -Wno-error
	obj-m	+= ecam30.o
endif

ecam30:
	make -C $(KERNEL_PATH) M=$(PWD) modules

clean:
	make -C $(KERNEL_PATH) M=$(PWD) clean
