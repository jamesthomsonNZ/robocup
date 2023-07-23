//************************************
//         logic.cpp       
//************************************
#include "logic.h"
#include "Arduino.h"
#include <Servo.h> 
#include "IO_class.h"
#include "Control_class.h"
#include "pickup.h"
#include "encoder.h"
#include "ultrasonic.hpp"
#include "Math_class.h"
#include <Adafruit_NeoPixel.h>

// set up motors and sensors
sensor_class R_ANGLE;
sensor_class L_ANGLE;
sensor_class R_FLAT;
sensor_class L_FLAT;
sensor_class FRONT;
sensor_class US_LEFT;
sensor_class US_RIGHT;

Adafruit_NeoPixel pixels(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

int Red_colour = 70;
int Green_colour = 0;
int Blue_colour = 70;
bool Red_colour_state = false;
bool Green_colour_state = false;
bool Blue_colour_state = false;

uint16_t usDistArray[US_NUM];
//ultrasounds insert here
movement_class Move;
Location_class Location;
states_class State_of_it;

void logic_inital(void){
  TOF_init();           //Initialiser for the TOF's
  Move.initial(0.01);   //Initialiser for the Controller
  usCounterInit();      //Initialiser for the ultrasonic counters
  Location.initial();   //Initialiser for the location tracking

  usAddToArray(US_TRIG_0, US_ECHO_0, LEFT_SIDE);
  usAddToArray(US_TRIG_1, US_ECHO_1, RIGHT_SIDE);
  
  R_ANGLE.initial(5, TOF_ALPHA, TOF_L1);   //Initialiser for each separate TOF sensor class
  L_ANGLE.initial(4, TOF_ALPHA, TOF_L1);
  R_FLAT.initial(0, TOF_ALPHA, TOF_L0);
  L_FLAT.initial(1, TOF_ALPHA, TOF_L0);
  FRONT.initial(2, TOF_ALPHA, TOF_L0);
  US_LEFT.initial(LEFT_SIDE, US_ALPHA, US);
  US_RIGHT.initial(RIGHT_SIDE, US_ALPHA, US); 

  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif

  pixels.begin();

  
  
  State_of_it.initial();
  pickup_initial();
  High_level_state = SEARCHING; //CALIBRATION_STATE;
}

void LED_colour_change(void) {
  //pixels.clear();
 
  Red_colour_state = (Red_colour > 90)? true : Red_colour_state;
  Red_colour_state = (Red_colour <= 0)? false : Red_colour_state;
  Green_colour_state = (Green_colour > 90)? true : Green_colour_state;
  Green_colour_state = (Green_colour <= 0)? false : Green_colour_state;
  Blue_colour_state = (Blue_colour > 90)? true : Blue_colour_state;
  Blue_colour_state = (Blue_colour <= 0)? false : Blue_colour_state;

  if (Red_colour_state == true) {
    Red_colour += -1;
  } else {
    Red_colour += 1;
  }

  if (Green_colour_state == true) {
    Green_colour += -1;
  } else {
    Green_colour += 1;
  }

  if (Blue_colour_state == true) {
    Blue_colour += -1;
  } else {
    Blue_colour += 1;
  }

  for(int i=0; i<NUM_LEDS; i++) {

    pixels.setPixelColor(i, pixels.Color(Red_colour, Green_colour, Blue_colour));
    pixels.show();
  }
}

/*
 *      sample_sensors
 *      
 *samples the sensors
 */
void TOF_sensor_sample(void) {
  R_ANGLE.sensor_sample();
  L_ANGLE.sensor_sample();
  R_FLAT.sensor_sample();
  L_FLAT.sensor_sample();
  FRONT.sensor_sample();
}

void us_sensor_sample(void) {
  usCalcArray(usArray, US_NUM, usDistArray);
  US_LEFT.sensor_sample();
  US_RIGHT.sensor_sample();
  usPingArray(usArray, US_NUM);
}

void update_motors(void) {
  Move.motor_update();
  
}

void states_update(void){
  State_of_it.state_refresh();
}

void update_location(void) {
  globalimu.calc_heading();
  Location.calc_location();
}

void move_function(void){
//  Serial.print(L_FLAT._dist);
//  Serial.print(", ");
//  Serial.print(L_ANGLE._dist);
//  Serial.print(", ");
//  Serial.print(FRONT._dist);
//  Serial.print(", ");
//  Serial.print(R_ANGLE._dist);
//  Serial.print(", ");
//  Serial.println(R_FLAT._dist);
//  Serial.print("US Sensors: ");
//  Serial.print(US_LEFT._dist);
//  Serial.print(", ");
//  Serial.println(US_RIGHT._dist);

//  Serial.print(digitalRead(3));
//  Serial.print(" ,");
//  Serial.println(digitalRead(2));
  
  State_of_it.do_state();
  //Move.forward(1950);
}
