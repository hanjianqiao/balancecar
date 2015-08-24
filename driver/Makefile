TARGET = speed
KDIR := /home/hjq/xeno/linux
ARCH = arm
CROSS_COMPILE = /home/hjq/xeno/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
PWD := $(shell pwd)
obj-m = $(TARGET).o

default:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

clean:
	rm *.o Module.symvers modules.order $(TARGET).ko $(TARGET).mod.c *~


