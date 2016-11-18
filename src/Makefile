TARGET = balancecar.la
SUBFILES += main.c
SUBFILES += pwm.c libMotionSensor.a libI2Cdev.a -I../git/chip_MPU6050/src/
OPTIONS += -lm
OPTIONS += -lwiringPi

default:
	gcc $(SUBFILES) -o $(TARGET) $(OPTIONS) 

prepare:
	sudo modprobe i2c-bcm2708
	sudo modprobe i2c-dev

run:
	sudo ./$(TARGET)

clean:
	rm *~

