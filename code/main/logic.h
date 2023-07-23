//************************************
//         logic.h       
//************************************

#ifndef LOGIC_H_
#define LOGIC_H_

#include "IO_class.h"
#include "Control_class.h"
#include "Math_class.h"
#include <Servo.h>

#define NUM_LEDS 16     //Number of LED's and the corresponding pin
#define DATA_PIN 33

#define TOF_ALPHA 0.6   //Alpha values for the recurrsive filter
#define US_ALPHA 0.6

extern sensor_class R_ANGLE;
extern sensor_class L_ANGLE;
extern sensor_class R_FLAT;
extern sensor_class L_FLAT;
extern sensor_class FRONT;
extern sensor_class US_LEFT;
extern sensor_class US_RIGHT;
extern movement_class Move;
extern Location_class Location;

extern uint16_t usDistArray[US_NUM];

void logic_inital();

/*
 *      sample_sensors
 *      
 *samples the sensors
 */
void TOF_sensor_sample();

void us_sensor_sample();
void LED_colour_change(void);

void update_motors();

void update_location();

void states_update(void);

/* 
 *       movement
 *       
 *very basic wall avoidance and robot movement
 */
void move_function();



#endif /* LOGIC_H_ */
