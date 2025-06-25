KERNELDIR := /home/lijiaxin/projects/loongarch/seekfree/LS2K0300_Library/ls2k0300_linux_4.19
CURRENT_PATH := $(shell pwd)
obj-m := v53l1x.o
build: kernel_moudles
kernel_moudles:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
clean:
	$(MAKE) -C $(KERNELDIR)	M=$(CURRENT_PATH) clean
