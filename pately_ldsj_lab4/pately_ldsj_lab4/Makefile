# file: pately_ldsj_lab4/Makefile
#/ EC535 Lab 4 - Traffic Light Controller
# Authors: Yash Patel (pately@bu.edu) and Leah Jones (ldsj@bu.edu)

obj-m += mytraffic.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
