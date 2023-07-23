//************************************
//         Control_class.cpp    
//************************************

// Contains the Movement controller and object state controller

#include "Arduino.h"
#include "Control_class.h"
#include "IO_class.h"
#include "logic.h"
#include <Servo.h>
#include "Math_class.h"

imu_class globalimu;
high_state High_level_state;

void control_init(){
  globalimu.initial();
  High_level_state = SEARCHING;
}

//------------------------------------------------------------------------------------------------------------------------------------

void movement_class::initial(double tolerance){
  _state = STATIONARY;
  _tolerance = tolerance;
  _angle = 0;
 PID_turning_angle.initial(TURN_P_GAIN, TURN_I_GAIN, TURN_D_GAIN, 0.01, 30);
 Servo left;
 Servo right;
 Left_motor.initial(LEFT_MOTOR_PIN, 1500, left, LEFT_SIDE);
 Right_motor.initial(RIGHT_MOTOR_PIN, 1500, right, RIGHT_SIDE);
}

//****************************************************************************

void movement_class::forward(int speed_value){

  if (Right_motor._value != speed_value) {
    Right_motor.reset_error();
  }
  if (Left_motor._value != speed_value) {
    Left_motor.reset_error();
  }

  Right_motor._value = speed_value;
  Left_motor._value = speed_value;

  //sets into steady state if within a tolereance
  double speed_tolerance_low = speed_value * (1 - _tolerance);
  double speed_tolerance_high = speed_value * (1 + _tolerance);
  if (((Left_motor._value < speed_tolerance_high) || (Left_motor._value > speed_tolerance_low)) && ((Right_motor._value < speed_tolerance_high) || (Right_motor._value > speed_tolerance_low))){
    _state = STEADY;
  } else {
    _state = FORWARD;
  }
}

//****************************************************************************

void movement_class::stationary(){
  int speed_value = 1500;

  Right_motor._value = speed_value;
  Left_motor._value = speed_value;

  //sets into steady state if within a tolereance
  double speed_tolerance_low = speed_value * (1 - _tolerance);
  double speed_tolerance_high = speed_value * (1 + _tolerance);
  if (((Left_motor._value < speed_tolerance_high) || (Left_motor._value > speed_tolerance_low)) && ((Right_motor._value < speed_tolerance_high) || (Right_motor._value > speed_tolerance_low))){
    _state = STEADY;
  } else {
    _state = FORWARD;
  } 
}

//****************************************************************************

void movement_class::turn(int angle) {
  //Serial.println(globalimu._heading);

  //Turns the angle inputed
  if ((angle > globalimu._heading + _angle_tol) || (angle < globalimu._heading - _angle_tol)){
    _state = TURNING;
    PID_turning_angle._desired_value = angle;
    PID_turning_angle.PID_output(globalimu._heading);
    double turning_correction = PID_turning_angle._offset_value;
  
    double motor_correction = TURN_RATIO * turning_correction;
  
    int Left_correction =  motor_correction; 
    int Right_correction = -1*motor_correction;

    if (Right_motor._value != Right_correction) {
      Right_motor.reset_error();
    }
    if (Left_motor._value != Left_correction) {
      Left_motor.reset_error();
    }
    
    Right_motor._value = Right_motor._value + Right_correction;
    Left_motor._value = Left_motor._value + Left_correction;

    if (Right_motor._value < 1300) {
      Right_motor._value = 1100;
    } else if (Left_motor._value < 1300){
      Left_motor._value = 1200;
    }
  //  Serial.println(Left_correction);
  //  //Serial.print(",");
  //  Serial.println(Right_correction);
    
  } else {
    PID_turning_angle.reset_values();
    _state = STEADY;
  }
}

//****************************************************************************

void movement_class::motor_update(){
  Right_motor.Setvalue(Right_motor._value);
  Left_motor.Setvalue(Left_motor._value);
}


void movement_class::twitch(){
  Right_motor.Setvalue(1200);
  Left_motor.Setvalue(1800);
  delay(500);
  Right_motor.Setvalue(1500);
  Left_motor.Setvalue(1500);
}
//------------------------------------------------------------------------------------------------------------------------------------

void states_class::initial(){
  _change_left = NO_DETECTION;
  _change_right = NO_DETECTION;
  _change_front = NO_DETECTION;
  in_channel = false;

  _delay_count = 0;
  Serial.println("states initalised");
}

//****************************************************************************

static int ramp_counter_R = 0;
static int ramp_counter_L = 0;

// Refreshes each obstacle state based on what the sensors are seeing
void states_class::state_refresh(){
  bool print = true;
  int right_angle = R_ANGLE._dist;
  int left_angle = L_ANGLE._dist;
  int right_flat = R_FLAT._dist;
  int left_flat = L_FLAT._dist;
  int front = FRONT._dist;
  int low_left = US_LEFT._dist;
  int low_right = US_RIGHT._dist;
//********************************
  //check front
  if(print == true){
    Serial.print(" Front State: ");
  }
  if(right_angle <= DIST_THRESHOLD && left_angle <= DIST_THRESHOLD && front <= DIST_THRESHOLD){
    if ((left_angle*cos((90-LEFT_SENSOR_ANGLE)*RAD_TO_DEG)/10) == front/10 && (right_angle*cos((90-RIGHT_SENSOR_ANGLE)*RAD_TO_DEG)/10) == (front/10)) {
      _front_state = PARALLEL;
      if(print == true){
        Serial.print("PARALLEL");
      }
    } else {
      _front_state = WALL;
      if(print == true){
        Serial.print("WALL");
      }
      
    }
  } else if(front <= DIST_THRESHOLD) {
    _front_state = FLAT_OBJECT;
    if(print == true){
        Serial.print("FLAT_OBJECT");
      }
  } else {
    _front_state = NO_DETECTION;
    if(print == true){
      Serial.print("NA");
    }
  }
//********************************
  //check right side
  if(print == true){
      Serial.print(" Right State: ");
    }

  if(right_angle <= DIST_THRESHOLD && right_flat <= DIST_THRESHOLD){
    // if the wall is parallel to the robot 
    if((right_angle*cos(RIGHT_SENSOR_ANGLE*RAD_TO_DEG)/10) == (right_flat/10)){
      _right_state = PARALLEL;
      if(print == true){
        Serial.print("PARALLEL");
      }
      
    } else {
      _right_state = WALL;
      if(print == true){
        Serial.print("WALL");
      }

    }
  // if the object is only picked up by one right side sensor
  } else if(right_angle <= DIST_THRESHOLD && right_flat > DIST_THRESHOLD && (_front_state != PARALLEL && _front_state != WALL)){
    _right_state = ANGLE_OBJECT;
    if(print == true){
      Serial.print("ANGLE_OBJECT");
    }
    
  } else if (right_angle > DIST_THRESHOLD && right_flat <= DIST_THRESHOLD){
    _right_state = FLAT_OBJECT;
    if(print == true){
      Serial.print("FLAT_OBJECT");
    }

  } else {
    _right_state = NO_DETECTION;
    if(print == true){
      Serial.print("NA");
    }
  }
  
//********************************
  // check left side
  if(print == true){
    Serial.print(" Left State: ");
  }
  if(left_angle <= DIST_THRESHOLD && left_flat <= DIST_THRESHOLD){
    // if the wall is parallel to the robot 
    if((left_angle*cos(LEFT_SENSOR_ANGLE*RAD_TO_DEG)/10) == (left_flat/10)){
      if(print == true){
        Serial.print("PARALLEL");
      }
      _left_state = PARALLEL;
    } else {
      _left_state = WALL;
      if(print == true){
        Serial.print("WALL");
      }      
    }
  // if the object is only picked up by one left side sensor
  } else if(left_angle <= DIST_THRESHOLD && left_flat > DIST_THRESHOLD && (_front_state != PARALLEL || _front_state != WALL)){
    _left_state = ANGLE_OBJECT;
    if(print == true){
      Serial.print("ANGLE_OBJECT");
    }
  } else if (left_angle > DIST_THRESHOLD && left_flat <= DIST_THRESHOLD){
    _left_state = FLAT_OBJECT;
    if(print == true){
      Serial.print("FLAT_OBJECT");
    }
  } else {
    _left_state = NO_DETECTION;
    if(print == true){
      Serial.print("NA");
    }
  }

//*********************************
  // checks lower left side
  if(print == true){
    Serial.print(" Low Left State: ");
  }
  if ((low_left <= WEIGHT_DIST_THRESHOLD) && (left_angle - low_left >= DIFFERENTIAL_THRESHOLD) && (High_level_state != PICKUP)) {
    if(print == true){
      Serial.print(ramp_counter_L);
    }  
    if (ramp_counter_L > RAMP_THRESHOLD_PERIOD) {
      _low_left = RAMP;
      if(print == true){
        Serial.print("RAMP");
      }
    } else {
      _low_left = WEIGHT;
      if(print == true){
        Serial.print("WEIGHT");
      }
    }
    ramp_counter_L += 1;
  } else {
    _low_left = NO_DETECTION;
    ramp_counter_L = 0;
    if(print == true){
      Serial.print("NA");
    }    
  }

//*********************************
  // checks lower right side
  if(print == true){
    Serial.print(" Low Right State: ");
  }
  if ((low_right <= WEIGHT_DIST_THRESHOLD) && (right_angle - low_right >= DIFFERENTIAL_THRESHOLD) && (High_level_state != PICKUP)) {
    if(print == true){
      Serial.print(ramp_counter_R);
    }
    if (ramp_counter_R > RAMP_THRESHOLD_PERIOD) {
      _low_right = RAMP;
      if(print == true){
        Serial.print("RAMP");
      }
    } else {
      _low_right = WEIGHT;
      if(print == true){
        Serial.print("WEIGHT");
      }
    }
    ramp_counter_R += 1;
  } else {
    _low_right = NO_DETECTION;
    ramp_counter_R = 0;
    if(print == true){
      Serial.print("NA");
    }    
  }

//********************************
  if(print == true){
    Serial.println("");
  }
  
}


//***********************************************************************************************************************

void states_class::do_state(){
  int heading = Move._angle;
  //Serial.println(heading);
  Move_State drive_direction = FORWARD;
  //Boolean to check if any states have changed
  bool change_in_state = _change_left != _left_state || _change_right != _right_state || _change_front != _front_state || _change_low_right != _low_right || _change_low_left != _low_left;

  // if the states have changed and if overall state is searching
  if((change_in_state || _delay_count == 0) && High_level_state == SEARCHING){    // 1st level
    _change_left = _left_state;
    _change_right = _right_state;
    _change_front = _front_state;
    _change_low_right = _low_right;
    _change_low_left = _low_left;

    if (High_level_state == RETURN_HOME) {
      
    }

//****************************************************************************   
    //if there's a weight and not trying to go home
    if (_low_left == WEIGHT && High_level_state != RETURN_HOME) {
      //heading += -60;
    } else if (_low_right == WEIGHT && High_level_state != RETURN_HOME) {
      //heading += 60;
    }
    //if there's a ramp
    if (_low_left == RAMP) {
      //heading += 60;
    } else if (_low_right == RAMP) {
      //heading += 60;
    }
    //if there's a channel
    if (in_channel == true) {                                             // 2nd level
      if ((_left_state == WALL || _left_state == PARALLEL) && (_right_state == WALL || _right_state == PARALLEL)) {
        drive_direction = REVERSE;
      } else if (_left_state == WALL || _left_state == PARALLEL){
        heading += -50;
        in_channel = false;
      } else {
        heading += 50;
        in_channel = false;
      }
 
    // if the front is clear 
    } else if(_front_state == NO_DETECTION) {                                            // 2nd level

      // if there is something only on the left side
      if (_left_state != NO_DETECTION && _right_state == NO_DETECTION){       
        if (_left_state == ANGLE_OBJECT){                                   
          heading += 45;
        } else if (_left_state == WALL){                                         
          heading += 30;
        }
      // if there is something only on the right side
      } else if ( _right_state != NO_DETECTION ){                           
        if (_right_state == ANGLE_OBJECT){                                      
          heading += -45;
        } else if (_right_state == WALL){                                         
          heading += -30;
        }
      }

    //if there is a wall in front
    } else if (_front_state != NO_DETECTION){                                     // 2nd level

      if((_left_state == WALL || _left_state == PARALLEL) && (_right_state == WALL || _right_state == PARALLEL)){
        in_channel = true;
        drive_direction = REVERSE;
      //if there's an object in front (not a wall)
      } else if (_front_state == FLAT_OBJECT || _front_state == WALL){                                  
        if(_left_state != NO_DETECTION ) {
          heading += 40;
        } else  {
          heading += -40;
        }
      //if stuck in a channel

       //if in a corner
      } else if(_left_state != NO_DETECTION ) {
        heading += 45;
      } else {
        heading += -45;
      }
    }
  

    
//****************************************************************************
  // if the states are the same as last call
  } else if (_delay_count >= TIME_DELAY) {                                        // 1st level
    _delay_count = 0;
  } else {
    _delay_count += 1;
  }

  //DISABLE IF TROUBLESHOOTING
  // keeps the robot moving -- will probably have to change for other states
  if (heading == Move._angle && High_level_state == RETURN_HOME) {
    Serial.println("I'm returning HOME!");
    heading = Location._angle;       // Sets the heading to the value to reach home if there are no obstacles and in return home state
  }
  //Serial.println(heading);
  if (High_level_state == PICKUP || High_level_state == CALIBRATION_STATE) {      // Stationary if in picup or calibration
    Move.stationary();
  } else if (High_level_state == SEARCHING || High_level_state == RETURN_HOME){   // Move if in searching state according to calculated heading
    Move.turn(heading);
    Serial.print(globalimu._heading);
    Serial.print(", "); 
    Serial.println(heading);
    if (Move._state == STEADY){
      if (drive_direction == FORWARD){
        Move.forward(1920);
      } else if (drive_direction == REVERSE) {
        Move.forward(1150);
      }
    }
  }
}
