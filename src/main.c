#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include "pwm.h"
#include<fcntl.h>
#include<string.h>
#include "sensor.h"

#define BUFFER_LENGTH 2               ///< The buffer length (crude but fine)
static int receive[BUFFER_LENGTH];     ///< The receive buffer from the LKM
static int fd_speed;

// GPIO Modes (IN=0, OUT=1)
#define GPIO_MODE_IN    0
#define GPIO_MODE_OUT   1

#define  DevAddr  0x68	//slave MPU6050 IIC device address

unsigned short aaa,ggg;
volatile float gyr,acc;
volatile float PWM;
int speed_output_RH,speed_output_LH;
volatile float Kp_angle,Kd_angle,Kd_position;
volatile float Kp_position;
volatile float position,position_dot;
volatile float position_dot_filter;
volatile int Turn_Need,Speed_Need;	//work when we wanna control the move

int channel = 0;

struct acc_dat{
	int x;
	int y;
	int z;
	int gx;
	int gy;
	int gz;
};

/*************************************/
// part of Kalman filter
float angle, angle_dot;
float Q_angle=0.001, Q_gyr=0.003, R_angle=0.5, dt=0.01;

float P[2][2] = {
							{ 1, 0 },
							{ 0, 1 }
						};
	
float Pdot[4] ={0,0,0,0};

const char C_0 = 1;

float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

void Kalman_Filter(float angle_m,float gyr_m)
{
	angle+=(gyr_m-q_bias) * dt;
	
	Pdot[0]=Q_angle - P[0][1] - P[1][0];
	Pdot[1]= -P[1][1];
	Pdot[2]= -P[1][1];
	Pdot[3]=Q_gyr;
	
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
	angle_dot = gyr_m-q_bias;
}

// control the wheels' speed
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

// use Kalman filter filters MPU6050 data
void AD_calculate(void)
{
	aaa=accel[1]+32768;	
	acc=(32768-aaa*1.0)/16384 - 0.0545;//+(2048-Get_Adc(0))*0.0005;
	if(acc>1)
		acc=1;
	else if(acc<-1)
		acc=-1;
	acc=57.3*asin(acc);
	ggg=gyro[0]+32768;
	gyr=(32768-ggg*1.0)/131;
	
// 	origin code read these parameter from hardware
//	Kp_angle=Get_Adc(1)/10.0;
//	Kd_angle=Get_Adc(2)/1000.0;
//	Kp_position=Get_Adc(3)/2000.0;
//	Kd_position=Get_Adc(4)/5.0;

	Kp_angle=1000.0;
	Kd_angle=100.0;
	Kp_position=5.0;
	Kd_position=100.0;
	
	Kalman_Filter(acc,gyr);

}

// calculate the output speed
void PWM_calculate(void)	
{  	
	if(angle<-40||angle>40)
	{  
		add_channel_pulse(channel, 18, 0, 0);
		add_channel_pulse(channel, 23, 0, 0);
		return;
	}
	read(fd_speed, receive, BUFFER_LENGTH);
	position_dot=(receive[0]+receive[1])*0.5;
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
	printf("%d %d %d %d\n", receive[0], receive[1], position, PWM);
	speed_output_RH+=Turn_Need;
	speed_output_LH-=Turn_Need;	
	PWM_output (speed_output_LH,speed_output_RH);	
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

	struct acc_dat acc_xyz;

	int demo_timeout = 10 * 1000000;
	int gpio = 18;
	int subcycle_time_us = SUBCYCLE_TIME_US_DEFAULT; //10ms;

	ms_open();

	if(-1 == fd){
		perror("I2C device setup error");	
	}

	fd_speed = open("/dev/balancecar", O_RDWR);             // Open the device with read/write access
	if (fd_speed < 0){
		perror("Failed to open the device...");
		return errno;
	}

	// Setup channel
	init_channel(channel, subcycle_time_us);
	print_channel(channel);

	// control output direction
	gpio_set_mode(17, GPIO_MODE_OUT);
	gpio_set_mode(22, GPIO_MODE_OUT);

	for(;;){
		ms_update();
		AD_calculate();
		PWM_calculate();
		receive[0]=0;		
		receive[1]=0;
	}

	close(fd_speed);

	// Clear and start again
	clear_channel_gpio(0, 18);
	add_channel_pulse(channel, gpio, 0, 50);
	usleep(demo_timeout);

	// All done
	shutdown();
	exit(0);
}
