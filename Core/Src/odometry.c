#include "odometry.h"

long volatile millis=0,t=0;

TIM_HandleTypeDef* htim_right_encoder;
TIM_TypeDef* right_TIM;
int right_resolution;
int right_precision;
int right_sens;
volatile unsigned int current_right_count;
volatile unsigned int last_right_count;
volatile long d_right;
volatile long total_right_count;
volatile float total_right=0,d_right_counter=0;
volatile double right_speed,right_encoder_speed;

TIM_HandleTypeDef* htim_left_Encoder;
TIM_TypeDef* left_TIM;
int left_resolution;
int left_precision;
int left_sens;
volatile unsigned int current_left_count;
volatile unsigned int last_left_count;
volatile long d_left;
volatile long total_left_count;
volatile float total_left=0,d_left_counter=0,total_centre=0;
volatile double left_speed,left_encoder_speed;

volatile float dR,dL,dC;
volatile float left_radius,right_radius,spacing_encoder,spacing_wheel;
volatile float current_x=0;//           ==> 285
volatile float current_y=0;//        ==> 676
volatile float current_phi_rad=PI/2;//PI/2;
volatile double phi_speed,d_phi_counter;
volatile float current_phi_deg=90;

float ref_x,ref_y,dec=0;

void set_right_encoder(TIM_HandleTypeDef* htim, TIM_TypeDef* TIM, int resolution, int precision, int sens)
{
	htim_right_encoder = htim;
	right_TIM = TIM;
	right_resolution = resolution;
	right_precision = precision;
	right_sens=sens;
	HAL_TIM_Encoder_Start(htim_right_encoder,TIM_CHANNEL_1);
	right_TIM->CNT = 0;
	total_right_count = 0;
}

void set_left_encoder(TIM_HandleTypeDef* htim, TIM_TypeDef* TIM, int resolution, int precision, int sens)
{
	htim_left_Encoder = htim;
	left_TIM = TIM;
	left_resolution = resolution;
	left_precision = precision;
	left_sens=sens;
	HAL_TIM_Encoder_Start(htim_left_Encoder,TIM_CHANNEL_1);
	left_TIM->CNT = 0;
	total_left_count = 0;
}

void read_right_encoder(void)
{
	last_right_count = current_right_count;
	current_right_count = right_sens*right_TIM->CNT;
	d_right = current_right_count - last_right_count;

	if (d_right>30000)
		d_right = d_right - 65535;
	if (d_right<-30000)
		d_right = d_right + 65535;
	total_right_count = total_right_count + d_right;
}

void read_left_encoder(void)
{
	last_left_count = current_left_count;
	current_left_count = left_sens*left_TIM->CNT;
	d_left = current_left_count - last_left_count;
	if (d_left>30000)
		d_left = d_left - 65535;
	if (d_left<-30000)
		d_left = d_left + 65535;
	total_left_count = total_left_count + d_left;
}

void set_dimension(float right_wheel_radius, float left_wheel_radius, float encoder_spacing, float wheels_spacing)
{
	right_radius = right_wheel_radius;
	left_radius = left_wheel_radius;
	spacing_encoder = encoder_spacing;
	spacing_wheel = wheels_spacing;
}

void update_position(void)
{
	read_right_encoder();
	read_left_encoder();
	dR = ticks_to_distance(d_right,right_radius,right_resolution,right_precision);
	total_right += dR;
	d_right_counter += dR;
	dL = ticks_to_distance(d_left,left_radius,left_resolution,left_precision);
	total_left += dL;
	d_left_counter += dL;
	dC = (dR+dL)/2;
	total_centre+=dC;

	current_x += dC*cos(current_phi_rad);
	current_y += dC*sin(current_phi_rad);
	current_phi_rad += ((dR-dL)/spacing_encoder);
	d_phi_counter += ((dR-dL)/spacing_encoder);
	while (current_phi_rad>PI)
	{
		current_phi_rad -= 2*PI;
	}
	while (current_phi_rad<-PI)
	{
		current_phi_rad += 2*PI;
	}
	current_phi_deg = rad_to_deg(current_phi_rad);
	millis++;
	t++;
	ref_x = current_x + cos(current_phi_rad) *dec;
	ref_y = current_y + sin(current_phi_rad) *dec;
}

void speed_calcul(void)
{
	right_encoder_speed = d_right_counter/T*1000;
	d_right_counter = 0;
	left_encoder_speed = d_left_counter/T*1000;
	d_left_counter = 0;
	phi_speed = d_phi_counter/T*1000;
	d_phi_counter = 0;
	right_speed = (right_encoder_speed + left_encoder_speed)/2 + phi_speed * spacing_wheel/2;
	left_speed = (right_encoder_speed + left_encoder_speed)/2 - phi_speed * spacing_wheel/2;
}

void reset_position(void)
{
	current_x = 0;
	current_y = 0;
	current_phi_deg = 0;
	current_phi_rad = 0;
}

float ticks_to_distance(int x, float r, int resolution, int precision)
{
	return (x*2*PI*r/(resolution*precision));
}

float rad_to_deg(double x)
{
	return (x*360/(2*PI));
}
