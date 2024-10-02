#include "main.h"

void set_motors (TIM_HandleTypeDef* htim, int MAX_PWM, uint32_t forward_right, uint32_t backward_right, uint32_t forward_left, uint32_t backward_left);
void run_motors (void);
void stop_motors (void);
void run_forward (int right_PWM, int left_PWM);
void set_PWM_min (int PWM_Min_R, int PWM_Min_L, int PWM_Min_R_Rot, int PWM_Min_L_Rot);

void PWM_sign_change_counter(void);
