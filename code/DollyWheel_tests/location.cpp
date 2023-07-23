//************************************
//         location.cpp       
//************************************

 // This file contains functions used to return to and
 // detect bases

#include "location.h"
#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Local definitions

// encoder constants
const int phaseA = 3;
const int phaseB = 2;
int encoder_pos = 0;
//ratio 20:8
//wheal diameter 66.5mm
// circumference 208.915mm
// 20 pulses per rev
const double distance_per_tick = 4.1783; //mm

//accelerometer constants
double old_gyro = 0;
double angular_intergral = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//location constants
double x = 0;
double y = 0;
int old_encoder_pos = 0;

/*
 *      location_init
 *      
 *initalises the location variables and functions
 */
void location_init(/* Parameters */){

  //set encoder input pins
  pinMode(phaseA, INPUT);
  pinMode(phaseB, INPUT);

  //sets interrupt for input of phaseA rising edge
  attachInterrupt(digitalPinToInterrupt(phaseA), encoderphaseA_interrupt, RISING);

  //makes a warning that the imu isn't detected
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
}

/*
 *      encoderphaseA_interrupt
 *      
 *does the interupt on the rising edge of the
 *encoders phase A
 */
void encoderphaseA_interrupt(){
  encoder_pos ++;
  //Serial.println("phaseA \n");
  //Serial.println(encoder_pos);
}

/*
 *      heading
 *      
 *calculates the robots heading using the imu
 */
void heading(){

  //gets data from gyro
  sensors_event_t gyro_data;
  bno.getEvent(&gyro_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
  double new_gyro = gyro_data.gyro.z;

  //intergrated that data to get heading
  angular_intergral += 0.5 * (old_gyro + new_gyro) * 0.1;

  //for intergration
  old_gyro = new_gyro;
}

/*
 *      get_heading
 *      
 *returns the current heading value
 */
int get_heading(){
  return angular_intergral;
}

/*
 *      location
 *      
 * calculates the robots current location with the imu
 * and encoder
 */
void location(){

  // calculated its reletive location
  x = x + (encoder_pos - old_encoder_pos) * distance_per_tick * cos(angular_intergral);
  y = y + (encoder_pos - old_encoder_pos) * distance_per_tick * sin(angular_intergral);

  old_encoder_pos = encoder_pos;

  //Serial.print(x);
  //Serial.print(", ");
  //Serial.println(y);
}
