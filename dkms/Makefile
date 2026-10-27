obj-m += hp_camera_fixture.o hp_ov7251.o hp_ov8858.o ipu3-cio2.o

ifdef KERNELRELEASE
KDIR := /lib/modules/$(KERNELRELEASE)/build
else
KDIR ?= /lib/modules/$(shell uname -r)/build
endif
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
