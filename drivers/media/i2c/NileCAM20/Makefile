ifneq (,$(filter $(CONFIG_VIDEO_ECAM),y m))
	ccflags-y += -Wno-unused-variable -I$(srctree)/drivers/media/platform/tegra/ -Wno-error
	obj-m	+=  ar0230.o
	ar0230-objs :=  ar0230_common.o serdes.o 
endif

ar0230:
	make -C $(KERNEL_PATH) M=$(PWD) modules
clean:
	make -C $(KERNEL_PATH) M=$(PWD) clean
