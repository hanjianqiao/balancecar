# balancecar

This project aims at transplanting control code from an STM32 based board onto RaspberryPi.

I copy pwm.h and pwm.c from https://github.com/metachris/RPIO.git

if YOU just wanna see the car run:
    This app consists of two part: speed driver and main runnable.
    speed.ko being compiled and generated by cross platform compiler should be inserted to kernel by command "sudo insmod speed.ko" before main runnalbe get working.
    balancecar.la only can get executed after "make prepare" and the speed.ko  is correctly inserted into kernel.
else if YOU wanna develop:
    in ./document/:
        speed_encoder, balance_car.pdf, motor_driver_board.pdf, and RegisterMap-MPU-6000A.pdf show you how hardware works.
        balance_car.pps shows the way software works.
else please contact me at hanjianqiao@gmail.com to inform your purpose, haha~~

