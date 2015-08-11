TARGET = balancecar.la
SUBFILES += main.c
SUBFILES += pwm.c
OPTIONS += -lm
OPTIONS += -lwiringPi

default:
	gcc $(SUBFILES) -o $(TARGET) $(OPTIONS)

prepare:
	sudo modprobe i2c-bcm2708
	sudo modprobe i2c-dev

run:
	sudo ./$(TARGET)

