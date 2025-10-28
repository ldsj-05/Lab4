# file: pately_ldsj_lab4/Makefile
#/ EC535 Lab 4 - Traffic Light Controller
# Authors: Yash Patel (pately@bu.edu) and Leah Jones (ldsj@bu.edu)

ifneq ($(KERNELRELEASE),)
	obj-m := mytraffic.o 
else
	KERNELDIR := $(EC535)/bbb/stock/stock-linux-4.19.82-ti-rt-r33
	PWD := $(shell pwd)
	ARCH := arm
	CROSS := arm-linux-gnueabihf-

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) clean

endif
