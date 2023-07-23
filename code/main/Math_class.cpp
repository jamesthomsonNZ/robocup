//************************************
//         Math_class.cpp    
//************************************

//contains the PID controller and location tracking

#include "Arduino.h"
#include "Math_class.h"
#include "encoder.h"
#include "Control_class.h"

/*
 *      PID_class
 *      
 *initalliser for PID_class
 */
void PID_class::initial(float P_constant, float I_constant, float D_constant, double delta_t, int max_offset) {
  _P_constant = P_constant;
  _I_constant = I_constant;
  _D_constant = D_constant;
  _delta_t = delta_t;
  _max_offset = max_offset;
  _desired_value = 0;
  
  _error_intergral = 0;
  _offset_value = 0;
}

void PID_class::PID_output(int current_value){
  int error = _desired_value - current_value;

//  Serial.print(_desired_value);
//  Serial.print(", ");
//  Serial.println(current_value);
  
  double P;
  double I;
  double D;

  // Proportional Gain
  if (_P_constant != 0) {
    P = (1.0/_P_constant) * error;
  } else {
    P = 0;
  }

  // Integral Gain
  if (_I_constant != 0){ 
    _error_intergral = _error_intergral + 0.5 * (_old_error + error) * _delta_t;
    I = (1.0/_I_constant) * _error_intergral;
  } else {
    I = 0;
  }

  // Differential Gain
  if (_D_constant != 0 && (error - _old_error) != 0){
    D = ((error - _old_error)/_delta_t) * (1.0/_D_constant);
  } else {
    D = 0;
  }

  _old_error = error;
//  Serial.print("Gain errors: ");
//  Serial.print(P);
//  Serial.print(", ");
//  Serial.print(I);
//  Serial.print(", ");
//  Serial.println(D);
//  Serial.print(", Error difference: ");
//  Serial.println(_error_intergral + (0.5 * (_old_error + error)*_delta_t));

//  Serial.print("Gain constants: ");
//  Serial.print(_P_constant);
//  Serial.print(", ");
//  Serial.print(_I_constant);
//  Serial.print(", ");
//  Serial.println(_D_constant);

  _offset_value = P + I + D;
  

  if ((-1*_max_offset) > _offset_value){
    _offset_value = -1 * _max_offset;
  }else if (_max_offset < _offset_value){
    _offset_value = _max_offset;
  }
  //Serial.println(_offset_value);
}

void PID_class::reset_values(void){
  _error_intergral = 0;
  _old_error = 0;
}

//******************************************************************************************************

/*
 * Location tracking class
 */

void Location_class::initial(void) {
  _x_dist = 0;
  _y_dist = 0;
  _angle = 0;
  _av_dist = 0;
}

void Location_class::calc_location(void) {
  _prev_dist = _av_dist;                                        // Sets the old distance
  _av_dist = (curr_dist_R + curr_dist_L) / 2;                   // Calculates the distance travelled from the encoders 
    
  _x_dist = _x_dist + cos(globalimu._heading * DEG_TO_RAD) * (_av_dist - _prev_dist);   // Calculates the x-location from home base
  _y_dist = _y_dist + sin(globalimu._heading * DEG_TO_RAD) * (_av_dist - _prev_dist);   // Calculates the y-location form home base

  _angle = 180 - atan2(_y_dist, _x_dist) * RAD_TO_DEG;          // Calculates the angle of the robot away from home base
}
