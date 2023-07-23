//************************************
//         encoder.cpp    
//************************************
#include "Arduino.h"
#include "encoder.h"

//---------------------------------------------------------------------

  int current_pos_L;
  int current_pos_R;
  int last_pos_L;
  int last_pos_R;

  double encoder_speed_L;
  double encoder_speed_R;

  boolean L_A_set1;
  boolean L_B_set1;
  boolean R_A_set1;
  boolean R_B_set1;
  
  static float last_dist_R;
  static float last_dist_L;
  double curr_dist_R;
  double curr_dist_L;
/*
 * Initialiser for both encoders
 */
void initialise_encoders(void) {
  pinMode(L_EN_A, INPUT);
  pinMode(L_EN_B, INPUT);
  pinMode(R_EN_A, INPUT);
  pinMode(R_EN_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(L_EN_A), update_encoder_L_A, RISING);
//  attachInterrupt(digitalPinToInterrupt(L_EN_B), update_encoder_L_B, RISING);
  attachInterrupt(digitalPinToInterrupt(R_EN_A), update_encoder_R_A, RISING);

  current_pos_L = 0;
  current_pos_R = 1;
  last_pos_L = 0;
  last_pos_R = 1;

  encoder_speed_R = 0;
  encoder_speed_L = 0;
  last_dist_R = 0;
  last_dist_L = 0;
  curr_dist_R = 0;
  curr_dist_L = 0;

  L_A_set1 = false;
  L_B_set1 = false;
  R_A_set1 = false;
  R_B_set1 = false;
}

/*
 * Interrupt for the right encoder
 */
void update_encoder_R_A(void){
  //delay(1);
  R_A_set1 = digitalRead(R_EN_A);               //Checks the A pin transition
  R_B_set1 = digitalRead(R_EN_B);               //Checks the B pin transition
  
  if ((R_A_set1 == HIGH && R_B_set1 == LOW) || (R_A_set1 == LOW && R_B_set1 == LOW)) {
    current_pos_R += 1;
  } else if ((R_A_set1 == HIGH && R_B_set1 == HIGH) || (R_A_set1 == LOW && R_B_set1 == LOW)){
    current_pos_R += -1;
  }
//  Serial.print("Right: ");
//  Serial.println(current_pos_R);
}

/*
 * Interrupt for the left encoder
 */
void update_encoder_L_A(void){
  L_B_set1 = digitalRead(L_EN_B);               //Checks the B pin transition

//  Serial.print(L_A_set1);
//  Serial.print(", ");
//  Serial.println(L_B_set1);

  if (L_B_set1 == LOW) {
    current_pos_L += -1;
  } else if (L_B_set1 == HIGH) {
    current_pos_L += 1;
  }
//  Serial.print("Left A: ");
//  Serial.println(current_pos_L);
}

void update_encoder_L_B(void){
  L_A_set1 = digitalRead(L_EN_A);               //Checks the A pin tranistion
  
//  Serial.print(L_A_set1);
//  Serial.print(", "); 
//  Serial.println(L_B_set1);

  if (L_A_set1 == HIGH) {
    current_pos_L += 1;
  } else if (L_A_set1 == LOW){
    current_pos_L += -1;
  }
//  Serial.print("Left B: ");
//  Serial.println(current_pos_L);
}

/*
 * Calculates the distance (position) of each encoder
 */
double calculate_encoder_dist_L(void){
  double dist_L = 0;
  if ((last_pos_L != current_pos_L)) {
    last_pos_L = current_pos_L;
  }
  dist_L = DIST_PER_REV / PPR * current_pos_L;    // distance of encoder in mm
//  Serial.print("Left Encoder Distance: ");
//  Serial.print(dist_L, 3);
//  Serial.print(", ");
//  Serial.println(current_pos_L);

  return dist_L;
}

double calculate_encoder_dist_R(void){
  double dist_R = 0;
  if ((last_pos_R != current_pos_R)) {
    last_pos_R = current_pos_R;
  }
  dist_R = DIST_PER_REV * current_pos_R / PPR;
//  Serial.print("Right Encoder Distance: ");
//  Serial.print(dist_R, 3);
//  Serial.print(", ");
//  Serial.println(current_pos_R);

  return dist_R;
}


void calc_encoder_speeds(void) {
  curr_dist_R = calculate_encoder_dist_R();
  curr_dist_L = calculate_encoder_dist_L();
//  Serial.print("DISTANCES: ");
//  Serial.print(curr_dist_R);
//  Serial.print(", ");
//  Serial.println(curr_dist_L);

  curr_dist_R = ALPHA*curr_dist_R - (1-ALPHA)*last_dist_R;
  curr_dist_L = ALPHA*curr_dist_L - (1-ALPHA)*last_dist_L;

  encoder_speed_R = ALPHA*((curr_dist_R - last_dist_R) / DT_ENCODER) - (1 - ALPHA)*encoder_speed_R;   // mm/s
  //encoder_speed_R = (curr_dist_R - last_dist_R) / DT_ENCODER;  
  encoder_speed_L = ALPHA*((curr_dist_L - last_dist_L) / DT_ENCODER) - (1 - ALPHA)*encoder_speed_L;
  //encoder_speed_L = (curr_dist_L - last_dist_L) / DT_ENCODER;
  last_dist_R = curr_dist_R; 
  last_dist_L = curr_dist_L;
//  Serial.print("SPEEDS: ");
//  Serial.print(encoder_speed_L, 3);
//  Serial.print(", ");
//  Serial.println(encoder_speed_R, 3);
}
