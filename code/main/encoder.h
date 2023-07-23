//************************************
//         encoder.h    
//************************************

#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"

#define PPR 730                //Pulses per Revolution
#define DIST_PER_REV 172.79   //In mm per rev

#define R_EN_A 3    //Left encoder pin A
#define R_EN_B 2    //Left encoder pin B
#define L_EN_A 5    //Right encoder pin A
#define L_EN_B 4    //RIght encoder pin B

#define DT_ENCODER 0.1    //Encoder speed calc sampling time

#define ALPHA 0.6


extern int current_pos_L;
extern int last_pos_L;
extern int current_pos_R;
extern int last_pos_R;

extern double curr_dist_R;
extern double curr_dist_L;

extern double encoder_speed_R;
extern double encoder_speed_L;

extern boolean  L_A_set1;
extern boolean  L_B_set1;
extern boolean  R_A_set1;
extern boolean  R_B_set1;

void update_encoder_R_A();
//void update_encoder_R_B();
void update_encoder_L_A();
void update_encoder_L_B();
void initialise_encoders();
double calculate_encoder_dist_L();
double calculate_encoder_dist_R();

void calc_encoder_speeds();

#endif //ENCODER_H
