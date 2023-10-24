ifneq (,$(filter $(CONFIG_VIDEO_ECAM),y m))
	ccflags-y += -Wno-unused-variable -I$(srctree)/drivers/media/platform/tegra/ -Wno-error
	obj-m	+=  nile25.o
	nile25-objs :=  ar0234_common.o serdes.o 
endif

nile25:
	make -C $(KERNEL_PATH) M=$(PWD) modules
clean:
	make -C $(KERNEL_PATH) M=$(PWD) clean
