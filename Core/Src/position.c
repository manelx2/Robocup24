/*
 * position.c
 *
 *  Created on: Sep 20, 2024
 *      Author: User
 */
#include "position.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h" // femma faza te3 hal object declaration it's not auto generated
#include "math.h"
/*local encoder variables*/
TIM_HandleTypeDef* htim_r_encoder;
TIM_TypeDef* r_TIM;
int r_resolution;
int r_precision;
int r_sens;
TIM_HandleTypeDef* htim_l_encoder;
TIM_TypeDef* l_TIM;
int l_resolution;
int l_precision;
int l_sens;
int d_lt,d_rt=0; //distance update in the 2 encoder in tick

/*odometry dimensions*/
float r_radius;
float l_radius;
float spacing_encoder;
float spacing_wheel;

/*position*/
float dR, dL;
float current_x ;
float current_y = 0;
float current_phi_deg ;
float current_phi_rad ;

/*counter*/
volatile unsigned int current_l_count;
volatile unsigned int current_r_count;
volatile unsigned int last_r_count;
volatile unsigned int last_l_count;
volatile unsigned int total_r_count;
volatile unsigned int total_l_count;

/*initialisation des encoder*/
void set_right(TIM_HandleTypeDef* htim, TIM_TypeDef* TIM, int resolution, int precision, int sens){
	    htim_r_encoder = htim;
		r_TIM = TIM;
		r_resolution = resolution;
		r_precision = precision;
		r_sens=sens;
		HAL_TIM_Encoder_Start(htim_r_encoder,TIM_CHANNEL_1);
		r_TIM->CNT = 0;
		total_r_count = 0;
}
void set_left(TIM_HandleTypeDef* htim, TIM_TypeDef* TIM, int resolution, int precision, int sens){
	    htim_l_encoder = htim;
		l_TIM = TIM;
		l_resolution = resolution;
		l_precision = precision;
		l_sens=sens;
		HAL_TIM_Encoder_Start(htim_l_encoder,TIM_CHANNEL_1); // why one channel?
		l_TIM->CNT = 0; //initialate register on 0
		total_l_count = 0;
}
/*reading encoder*/
void read_right(void){
	    last_r_count = current_r_count;
		current_r_count = r_sens*r_TIM->CNT;
		d_rt = current_r_count - last_r_count;
		if (d_rt>30000) //almost half the maximum fluctuation point
			d_rt = d_rt - 65535;
		if (d_rt<-30000)
			d_rt = d_rt + 65535;
		total_r_count = total_r_count + d_rt;
}
void read_left(void){
	last_l_count = current_l_count;
		current_l_count = l_sens*l_TIM->CNT;
		d_lt = current_l_count - last_l_count;
		if (d_lt>30000)
			d_lt = d_lt - 65535;  //ana na3ref maximum count value is 65535
		if (d_lt<-30000)
			d_lt = d_lt + 65535;
		total_l_count = total_l_count + d_lt;
}
void set_dimentions(float r_wheel_radius, float l_wheel_radius, float encoder_spacing, float wheels_spacing)
{
	r_radius = r_wheel_radius;
	l_radius = l_wheel_radius;
	spacing_encoder = encoder_spacing;
	spacing_wheel = wheels_spacing;
}



/*calcule des distance*/
float distance(int x, float r,int resolution,int precision){
	return (x*2*PI*r/(resolution*precision));
}
float angle(double x)
{
	return (x*360/(2*PI));
}

void where_am_I(void){
	    //read encoder
	    read_right();
	    read_left();

	    // Convert ticks to distances for both wheels
	    float dR = distance(d_rt, r_radius, r_resolution, r_precision);  // Right wheel distance
	    float dL = distance(d_lt, l_radius, l_resolution, l_precision);      // Left wheel distance

	    // Calculate the average distance moved by the center of the robot
	    float dC = (dR + dL) / 2;  // Center distance is the average of the two wheels

	    // Update the robot's x and y position using the current orientation (angle)
	    current_x += dC * cos(current_phi_rad);  // Update x position
	    current_y += dC * sin(current_phi_rad);  // Update y position

	    // Update the robot's orientation based on the difference between the wheel movements
	    current_phi_rad += (dR - dL) / spacing_encoder;  // Update angle (radians)

	    // Normalize the angle to stay between -π and π radians
	    if (current_phi_rad > PI)
	        current_phi_rad -= 2 * PI;
	    else if (current_phi_rad < -PI)
	        current_phi_rad += 2 * PI;
}
void origin_again(void)
{
	current_x = 0;
	current_y = 0;
	current_phi_deg = 0;
	current_phi_rad = 0;
}
