/*
 * robot_navi.h
 *
 *  Created on: Oct 2, 2024
 *      Author: User
 */

#ifndef INC_ROBOT_NAVI_H_
#define INC_ROBOT_NAVI_H_



#ifdef __cplusplus
extern "C" {
#endif
#include "Odometry.h"
#include "Motors.h"
#include "math.h"
#include <stdlib.h>
#include <stdbool.h>
void move_distance(float distance,float speed); //distance (mm); speed (mm/s)
void asta3(float distance,float speed); //distance (mm); speed (mm/s)
void asta3L(float distance,float speed); //distance (mm); speed (mm/s)
void rotate(float angle, float speed); //angle (degree) ; speed (mm/s) (Wheel linear speed)
void orientate (float orientation, float speed);
void Robot_Locate(float goal_x, float goal_y, float speed);
void trajectory (void);
void curv(float R,float theta ,float speed);
void Robot_locateCurv(float x, float y, float phi_target ,float speed); //phi_target(degree) phi_target==180 || ==-180 selon le sens souhaité
void Multi_Curv(float R,float theta ,float speed, int i,int n);
void Robot_Locate_Multi_Curv( float** matrix, int n , int speed);//!!!!speed==300 // matrix contains the (x,y) of the desired locations your want the robot to go to in order
void movement_sequence(float speed);
void calculateNode(double xR, double yR, double xC, double yC, double xO, double yO, double rO, double* xN, double* yN);
void orientate2 (float orientation, float speed);
bool outsideofmap(float x,float y);

void Robot_Locateobst(float x,float y,float speed);
//test fonction
void move(float speed, int delay);
//
void Calcul_Curv(float x, float y,float phi_target);
//
void allocation (int taille);
//
float** create_points(float,float,float,float);
//
void move_distance2_9otbi(float distance,float speed);

#ifdef __cplusplus
} // end extern "C"
#endif



#endif /* INC_ROBOT_NAVI_H_ */
