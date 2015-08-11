#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include "pwm.h"

// GPIO Modes (IN=0, OUT=1)
#define GPIO_MODE_IN    0
#define GPIO_MODE_OUT   1

#define  DevAddr  0x68	//slave MPU6050 IIC device address

unsigned char mpu6050_buffer[14];
unsigned short aaa,ggg;
volatile float gyro,acc;
volatile float PWM;
int speed_output_RH,speed_output_LH;
volatile float Kp_angle,Kd_angle,Kd_position;
volatile float Kp_position;
volatile float position,position_dot;
volatile float position_dot_filter;
volatile int Turn_Need,Speed_Need;

int channel = 0;

struct acc_dat{
	int x;
	int y;
	int z;
	int gx;
	int gy;
	int gz;
};

float angle, angle_dot;
float Q_angle=0.001, Q_gyro=0.003, R_angle=0.5, dt=0.01;

float P[2][2] = {
							{ 1, 0 },
							{ 0, 1 }
						};
	
float Pdot[4] ={0,0,0,0};

const char C_0 = 1;

float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

void Kalman_Filter(float angle_m,float gyro_m)
{
	angle+=(gyro_m-q_bias) * dt;
	
	Pdot[0]=Q_angle - P[0][1] - P[1][0];
	Pdot[1]= -P[1][1];
	Pdot[2]= -P[1][1];
	Pdot[3]=Q_gyro;
	
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
	
	
	angle_err = angle_m - angle;
	
	
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	
	
	angle	+= K_0 * angle_err;
	q_bias	+= K_1 * angle_err;
	angle_dot = gyro_m-q_bias;
}

void PWM_output (int PWM_LH,int PWM_RH)
{
	if (PWM_LH<0)
	{
		gpio_set(17, 1);
		PWM_LH*=-1;
	}
	else
	{
		gpio_set(17, 0);
	}
	
	if (PWM_LH>1800)
	{
		PWM_LH=1800;
	}
	if (PWM_RH<0)
	{
		gpio_set(22, 1);
		PWM_RH*=-1;
	}
	else
	{
		gpio_set(22, 0);
	}	
	if (PWM_RH>1800)
	{
		PWM_RH=1800;
	}
	add_channel_pulse(channel, 18, 0, PWM_LH);
	add_channel_pulse(channel, 23, 0, PWM_RH);

}

void adxl345_init(int fd)
{
	wiringPiI2CWriteReg8(fd, 0x6b, 0x00);
	wiringPiI2CWriteReg8(fd, 0x19, 0x07);
	wiringPiI2CWriteReg8(fd, 0x1a, 0x06);
	wiringPiI2CWriteReg8(fd, 0x1b, 0x18);
	wiringPiI2CWriteReg8(fd, 0x1c, 0x01);
}

void AD_calculate(void)
{
	aaa=mpu6050_buffer[2]*256+mpu6050_buffer[3]+32768;	
	acc=(32768-aaa*1.0)/16384 - 0.06;//+(2048-Get_Adc(0))*0.0005;
//	printf("%f\n", acc);
	if(acc>1)
		acc=1;
	else if(acc<-1)
		acc=-1;
	acc=57.3*asin(acc);
	ggg=mpu6050_buffer[8]*256+mpu6050_buffer[9]+32768;
	gyro=(32768-ggg*1.0)/131;
	
//	Kp_angle=Get_Adc(1)/10.0;
//	Kd_angle=Get_Adc(2)/1000.0;
//	Kp_position=Get_Adc(3)/2000.0;
//	Kd_position=Get_Adc(4)/5.0;

	Kp_angle=800.0;
	Kd_angle=100.0;
	Kp_position=0.0;
	Kd_position=0.0;
	
	Kalman_Filter(acc,gyro);

}
int speed_real_LH = 0,speed_real_RH = 0;
void PWM_calculate(void)	
{  	
	if(angle<-40||angle>40)
	{  
		add_channel_pulse(channel, 18, 0, 0);
		add_channel_pulse(channel, 23, 0, 0);
		return;
	}
	position_dot=(speed_real_LH+speed_real_RH)*0.5;
	position_dot_filter*=0.85;		
	position_dot_filter+=position_dot*0.15;
	
	position+=position_dot_filter;
	position+=Speed_Need;
	if(position<-800)
	{
		position=-800;
	}
	else if(position>800)
	{
		position=800;
	}
	PWM =-Kp_angle*angle-Kd_angle*angle_dot-Kp_position*position-Kd_position*position_dot_filter;	
	speed_output_LH=speed_output_RH=PWM;
	printf("PWM: %f\n", PWM);
	speed_output_RH+=Turn_Need;
	speed_output_LH-=Turn_Need;	
	PWM_output (speed_output_LH,speed_output_RH);	
}


struct acc_dat adxl345_read_xyz(int fd)
{
	struct acc_dat acc_xyz;
	int len = 14;
	int pos = 0;
	while (len) {
		mpu6050_buffer[pos] = wiringPiI2CReadReg8(fd, 0x3b + pos);
		++pos;
		--len;
    	}

	return acc_xyz;
}

/***********************************************************/

int
main(int argc, char **argv)
{
    // Very crude...
    if (argc == 2 && !strcmp(argv[1], "--pcm"))
        setup(PULSE_WIDTH_INCREMENT_GRANULARITY_US_DEFAULT, DELAY_VIA_PCM);
    else
        setup(PULSE_WIDTH_INCREMENT_GRANULARITY_US_DEFAULT, DELAY_VIA_PWM);

    // Setup demo parameters

	int fd;
	struct acc_dat acc_xyz;

    int demo_timeout = 10 * 1000000;
    int gpio = 18;
    int subcycle_time_us = SUBCYCLE_TIME_US_DEFAULT; //10ms;

	fd = wiringPiI2CSetup(DevAddr);

	if(-1 == fd){
		perror("I2C device setup error");	
	}

	adxl345_init(fd);

	// Setup channel
	init_channel(channel, subcycle_time_us);
	print_channel(channel);
	
	gpio_set_mode(17, GPIO_MODE_OUT);
	gpio_set_mode(22, GPIO_MODE_OUT);

	for(;;){
		adxl345_read_xyz(fd);
		AD_calculate();
		PWM_calculate();
		speed_real_LH=0;		
		speed_real_RH=0;
	}
	// Clear and start again
	clear_channel_gpio(0, 18);
	add_channel_pulse(channel, gpio, 0, 50);
	usleep(demo_timeout);

	// All done
	shutdown();
	exit(0);
}
