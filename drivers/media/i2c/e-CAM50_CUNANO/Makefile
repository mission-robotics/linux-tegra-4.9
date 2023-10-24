ifneq (,$(filter $(CONFIG_VIDEO_ECAM),y m))
	ccflags-y += -I$(srctree)/drivers/media/platform/tegra/ -Wno-error
	obj-m	+= ar0521.o
endif

ar0521:
	make -C $(KERNEL_PATH) M=$(PWD) modules
clean:
	make -C $(KERNEL_PATH) M=$(PWD) clean
