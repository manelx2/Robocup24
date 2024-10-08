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
void move_distance(float distance, float speed)
{
    init();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    // Set acceleration and deceleration distances
    if (fabs(distance) < (speed * speed / ramp)) {
        accel_dist = fabs(distance) / 2;
        decel_dist = fabs(distance) / 2;
        speed = sqrt(2 * ramp * accel_dist);
    } else {
        accel_dist = 0.5 * speed * speed / ramp;
        decel_dist = 0.5 * speed * speed / ramp;
    }

    while (fabs(total_right - distance) > 2 || fabs(total_left - distance) > 2) {
        // Set movement direction (forward or backward)
        sens = ((total_right + total_left) / 2 - distance) < 0 ? 1 : -1;

        // Adjust speed for acceleration and deceleration phases
        if (fabs((total_right + total_left) / 2) < accel_dist) {
            speed_ref = sens * constrain(sqrt(ramp * fabs(total_right + total_left)), 0, 1000);
        } else if (fabs((total_right + total_left) / 2 - distance) < decel_dist) {
            speed_ref = sens * constrain(sqrt(2 * ramp * fabs((total_right + total_left) / 2 - distance)), 0, 1000);
        } else {
            speed_ref = sens * speed;
        }

        // Right wheel regulation
        right_error = speed_ref - right_speed;
        i_right_error += right_error;
        PWM_RB = kp * right_error + ki * i_right_error;
        PWM_RB = constrain(PWM_RB, -PWM_Max, PWM_Max);

        // Left wheel regulation
        left_error = speed_ref - left_speed;
        i_left_error += left_error;
        PWM_LB = kp * left_error + ki * i_left_error;
        PWM_LB = constrain(PWM_LB, -PWM_Max, PWM_Max);

        // Orientation correction
        float correction = coef_correct_dist * (total_right - total_left);
        PWM_R = PWM_RB + correction;
        PWM_L = PWM_LB - correction;

        // Execute motor commands
        run_motors();
    }

    // Stop motors when distance is reached
    stop_motors();
}

void rotate(float angle, float speed)
{
    init();

    // Set the goal distance based on the rotation angle
    goal_distance = angle * PI * spacing_encoder / 180;

    // Set acceleration and deceleration distances
    if (fabs(goal_distance) < (2 * speed * speed / rampR)) {
        accel_dist = fabs(goal_distance) / 2;
        decel_dist = fabs(goal_distance) / 2;
        speed = sqrt(rampR * accel_dist);
    } else {
        accel_dist = speed * speed / rampR;
        decel_dist = speed * speed / rampR;
    }

    // Rotate until the desired angle is reached or there's an obstacle (evitementFlag)
    while (fabs(total_right - total_left) < fabs(goal_distance) && evitementFlag) {
        // Set rotation direction (clockwise or counter-clockwise)
        sens = ((total_right - total_left) - goal_distance) < 0 ? 1 : -1;

        // Adjust speed for acceleration and deceleration phases
        if (fabs(total_right - total_left) < accel_dist) {
            speed_ref = sens * constrain(sqrt(rampR * fabs(total_right - total_left)), 50, 1000);
        } else if (fabs(total_right - total_left - goal_distance) < decel_dist) {
            speed_ref = sens * constrain(sqrt(rampR * fabs(total_right - total_left - goal_distance)), 10, 1000);
        } else {
            speed_ref = sens * speed;
        }

        // Right wheel regulation
        right_error = speed_ref - right_speed;
        i_right_error += right_error;
        PWM_RB = kp * right_error + ki * i_right_error;
        PWM_RB = constrain(PWM_RB, -PWM_Max, PWM_Max);

        // Left wheel regulation
        left_error = -speed_ref - left_speed;
        i_left_error += left_error;
        PWM_LB = kp * left_error + ki * i_left_error;
        PWM_LB = constrain(PWM_LB, -PWM_Max, PWM_Max);

        // Position correction for maintaining angle stability
        float correction = coef_correct_angle * (total_right + total_left);
        PWM_R = PWM_RB + correction;
        PWM_L = PWM_LB - correction;

        // Execute motor commands
        run_motors();
    }

    // Stop motors when rotation is complete
    stop_motors();
}

