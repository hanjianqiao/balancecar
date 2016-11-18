TARGET = speed
KDIR := ~/pi/work/raspberry/linux-rpi-3.8.y/
ARCH = arm
CROSS_COMPILE = ~/pi/git/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
PWD := $(shell pwd)
obj-m = $(TARGET).o

default:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

clean:
	rm -f *.o Module.symvers modules.order $(TARGET).ko $(TARGET).mod.c *~
	rm -f .$(TARGET).*.cmd
	rm -rf .tmp_versions


