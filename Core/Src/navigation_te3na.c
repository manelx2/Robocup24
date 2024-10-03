/*
 * navigation_te3na.c
 *
 *  Created on: Oct 3, 2024
 *      Author: User
 */
#include "navigation_te3na.h"
#include "odometry.h"
#include "motor.h"
#include "math.h"
#include <stdlib.h>
#include <stdbool.h>
/*
extern volatile long millis,t;
extern unsigned long long int delta,t0;
bool started=false;





//Motors related variables
extern volatile long d_right;
extern volatile long d_left;
extern int PWM_R,PWM_L;
extern int PWM_Max;
extern int PWM_R_Min,PWM_L_Min;
extern int PWM_R_Min_Rot,PWM_L_Min_Rot;
int PWM_R_sign_counter,PWM_L_sign_counter;

//Odometry related variables
extern volatile float total_right,total_left, total_centre;
extern volatile float  current_x,current_y,current_phi_deg,current_phi_rad;
extern float ref_x, ref_y;
extern float spacing_encoder,spacing_wheel,dec;
extern volatile double right_speed, left_speed;

bool tirette=true;

int sum=0;


//Robot Navi Related Variables
float goal_distance, target_angle;
float accel_dist, decel_dist;
int PWM_LB,PWM_RB;
int coef_correct_angle=70; //50 Correction rotate
int right_correction=0,left_correction=0;

//Speed Regulation
float speed_ref, ramp = 400, rampR=100,rampC=500;//max 1500 //deja 800 vitesse kbira
int sens;
double right_error=0,i_right_error=0;
double left_error=0,i_left_error=0;
float kp = 8, ki = 0.7;//9.1 2.1

//Move
int coef_correct_dist = 30;//25

//Trajectory
float target_x, target_y;
float target_x_prime, target_y_prime;
float right_target_speed, left_target_speed;

// curv
float remain_distC=0,goalC=0;
float speed_refR=0,speed_refL=0,prev_speed_refR=0,prev_speed_refL=0,new_speed_refR=0,new_speed_refL=0,speedC=0,speed_refC=0;
float kpL = 15.5, kiL = 1.75;
float kpR = 15.5, kiR = 1.75;
float corde=0,tetaC=0,phi_prim=0,corde_angle=0,Xc=0,Yc=0,Rayon=0,phi_target_rad=0,sens_de_mouvement=0;
float Distance_empietement=50;


float x_obst,y_obst;
float x_obst_abs,y_obst_abs;



float l1,l2,l3,r1,r2,r3 ;
double xN,yN;
float wl1=70,wl2=50,wl3=10,wr1=70,wr2=-50,wr3=-10 ;

float constrain (float x,float min,float Max)
{
	if (x<min) return min;
	if (x>Max) return Max;
	else return(x);
}

void init (void)
{
	total_right=0;
	total_left=0;
	total_centre=0;
	PWM_L_sign_counter=0;
	PWM_R_sign_counter=0;
	i_right_error = 0;
	i_left_error = 0;
	right_error=0;
	left_error=0;
}
void move_distance(float distance,float speed)
{
	init();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	int z=0;
	//Set accel/decel distance
	if (fabs(distance) < (speed*speed/ramp))
	{
		accel_dist = fabs(distance)/2;
		decel_dist = fabs(distance)/2;
		speed = sqrt (2*ramp*accel_dist);
	}
	else
	{
		accel_dist = (float)0.5*speed*speed/ramp;
		decel_dist = (float)0.5*speed*speed/ramp;
	}
	while(((fabs(total_right-distance)>2)||(fabs(total_left-distance)>2))&& (speed*z/500)<((fabs(distance)*360/1000)+50))
	{   z++;
		x_obst_abs=current_x+x_obst*sin(current_phi_rad)+y_obst*cos(current_phi_rad);
		y_obst_abs=current_y+y_obst*sin(current_phi_rad)-x_obst*cos(current_phi_rad);
//			loop();
//			HAL_Delay(5);
		t0=t;

        }
		//Accel/Decel Speed Set
		if (((total_right+total_left)/2 -distance)<0)
			sens = 1;
		else
			sens = -1;
		if (fabs((total_right+total_left)/2) < accel_dist)
			speed_ref = sens*50+sens*(constrain(sqrt (ramp*fabs(total_right+total_left))-50,0,1000));

		else if (fabs((total_right+total_left)/2 -distance) < decel_dist)
			if(i==0)
				speed_ref = sens*10+sens*constrain((sqrt(2*ramp*fabs((total_right+total_left)/2 -distance))-10),0,1000);//fabs((total_right+total_left)/2 -distance)
			else
				speed_ref = sens*10+sens*constrain((sqrt(2*ramp_evitement*fabs((total_right+total_left)/2 -distance))-10),0,1000);
		else
			speed_ref = sens*speed;
		//Right wheel regulation
		right_error = speed_ref - right_speed;
		i_right_error += right_error;
		PWM_RB = kp * right_error + ki * i_right_error;
		if (PWM_RB>PWM_Max) PWM_RB = PWM_Max;
		if (PWM_RB<-PWM_Max) PWM_RB = -PWM_Max;
		//Left wheel regulation
		left_error = speed_ref - left_speed;
		i_left_error += left_error;
		PWM_LB = kp * left_error + ki * i_left_error;
		if (PWM_LB>PWM_Max) PWM_LB = PWM_Max;
		if (PWM_LB<-PWM_Max) PWM_LB = -PWM_Max;
		//Orientation Correction²
		left_correction = coef_correct_dist * (total_right-total_left);
		right_correction = - left_correction;

		PWM_R = PWM_RB + right_correction ;
		PWM_L = PWM_LB + left_correction;

		//Execution
		run_motors();
		do delta=t-t0;
		while (delta<T);//taslih
	}
	i=0;
	stop_motors();
}

void rotate(float angle, float speed)
{
	init();
	//Set accel/decel distance
	goal_distance = angle * PI * spacing_encoder/ 180;
	if (fabs(goal_distance) < (2*speed*speed/rampR))
	{
		accel_dist = fabs(goal_distance)/2;
		decel_dist = fabs(goal_distance)/2;
		speed = sqrt(rampR*accel_dist);
	}
	else
	{
		accel_dist = (float)speed*speed/rampR;
		decel_dist = (float)speed*speed/rampR;
	}
	while( (fabs(total_right-total_left)<fabs(goal_distance) || fabs(total_right-total_left)> fabs(goal_distance)+2 ) && evitementFlag )
	{
		nh.spinOnce();
		t0=t;
		//Accel/Decel Speed Set
		if (((total_right-total_left)-goal_distance)<0)
			sens = 1;
		else
			sens = -1;
		if (fabs((total_right-total_left)) < accel_dist)
			speed_ref = sens*constrain(sqrt(rampR*fabs(total_right-total_left)),50,1000);
		else if (fabs((total_right-total_left)-goal_distance) < decel_dist)
			speed_ref = sens*constrain(sqrt(rampR*fabs((total_right-total_left)-goal_distance)),10,1000);
		else
			speed_ref = sens*speed;
		//Right wheel regulation
		right_error = speed_ref - right_speed;
		i_right_error += right_error;
		PWM_RB = kp * right_error + ki * i_right_error;
		//Left wheel regulation
		left_error = - speed_ref - left_speed;
		i_left_error += left_error;
		PWM_LB = kp * left_error + ki * i_left_error;
		//Position Correction;
		left_correction = coef_correct_angle * (total_right + total_left);
		right_correction = - left_correction;
		PWM_R = PWM_RB + right_correction;
		PWM_L = PWM_LB - left_correction;
		//Execution
		run_motors();
		do delta=t-t0;
		while (delta<T);
	}
	stop_motors();
}*/
