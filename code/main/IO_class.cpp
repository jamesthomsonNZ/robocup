//************************************
//         IO_class.cpp    
//************************************

// contains the TOF (partial us) sensor, motor and IMU code

#include "Arduino.h"

#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

#include "IO_class.h"
#include "Math_class.h"
#include "encoder.h"
#include "ultrasonic.hpp"
#include "logic.h"

//-----------------------------------------------------------------------
/* 
 *  TOF init objects
 */
// Start Address for the TOF's Teensy pin
const byte SX1509_ADDRESS = 0x3F;
// The number of sensors in your system.
const uint8_t sensorCount = 3; //number for each sensor type i.e. 2 L0's and 2 L1's

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL0[8] = {0,1,2,3}; //actually only using 0, 1 and 2
const uint8_t xshutPinsL1[8] = {4,5,6,7}; //actually only using 4 and 5

SX1509 io; // Create an SX1509 object to be used throughout
VL53L0X sensorsL0[sensorCount];
VL53L1X sensorsL1[sensorCount-1];

/* 
 *  TOF Init Function
 */
void TOF_init()
{
  io.begin(SX1509_ADDRESS);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
  }

  for (uint8_t i = 0; i < sensorCount-1; i++)
  {
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }

  // L0 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);

    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L0 ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);

    sensorsL0[i].startContinuous(TOF_PERIOD);
  }

  // L1 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount-1; i++)   // is sensorCount-1 as there is one less L1 sensor than L0's
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);

    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);

    sensorsL1[i].startContinuous(TOF_PERIOD);
  }
  Serial.println("TOF initialised");
}

//-----------------------------------------------------------------------
/*
 *      sensor_class
 *      
 *initalliser for sensor_class
 */
void sensor_class::initial(int pin, double alpha, sensor_type type){
  _pin = pin;
  _value = 0;
  _alpha = alpha;
  _type = type;
  _dist = 0;
}

/*
 *      sensor_sample
 *      
 *samples the sensor
 */
void sensor_class::sensor_sample(){
  if (_type == TOF_L0) {
    _value = sensorsL0[_pin].readRangeContinuousMillimeters();
    if (_value > L0_MAX_RANGE) { _value = L0_MAX_RANGE; }
    if (sensorsL0[_pin].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  } else if (_type == TOF_L1){
    _value = sensorsL1[_pin-4].read();
    if (_value > L1_MAX_RANGE) { _value = L1_MAX_RANGE; }
    if (sensorsL1[_pin-4].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  } else if (_type == US) {
    _value = usDistArray[_pin];
    if (_value > US_MAX_RANGE) { _value = US_MAX_RANGE; }
  }
  _dist = _alpha*_dist + (1-_alpha)*_value;
}

//-----------------------------------------------------------------------

/*
 *      motor_class
 *      
 *initalliser for motor_class
 */
 void motor_class::initial(int pin, int value, Servo motor_name, int side) {
  motor_name.attach(pin);
  motor_name.writeMicroseconds(value);
  Serial.println("Motors init");
  
  _value = value;
  _motor_name = motor_name;
  _side = side;
  _encoder_dist = 0;
  _prev_dist = 0;
  _encoder_v = 0;
  _dt = 0.1;

  if (_side == LEFT_SIDE) {
    Serial.println("LEFT");
     motor_pid_L.initial(MOTOR_P_GAIN, MOTOR_I_GAIN, MOTOR_D_GAIN, MOTOR_CONTROL_PERIOD, 400);
  } else if (_side == RIGHT_SIDE) {
    Serial.println("RIGHT");
     motor_pid_R.initial(MOTOR_P_GAIN, MOTOR_I_GAIN, MOTOR_D_GAIN, MOTOR_CONTROL_PERIOD, 400);
  }
}

void motor_class::Setvalue(int value) {
  //Serial.println(value);
  _value = value;
  // Calculates the encoder speeds for each respective motor
  if (_side == LEFT_SIDE) {
    _encoder_v = encoder_speed_L;
//    Serial.print(_encoder_v);
//    Serial.print(", ");
  } else if (_side == RIGHT_SIDE) {
    _encoder_v = encoder_speed_R;
//    Serial.println(_encoder_v);
  }
 


  double correction = 0;
  if (_side == LEFT_SIDE) {
    // Converts the speed of the encoder from mm/s to a relative motor speed
     _encoder_v = _encoder_v * MOTOR_SPEED_RATIO_L + MOTOR_SPEED_OFFSET;
     motor_pid_L._desired_value = _value;
     motor_pid_L.PID_output(_encoder_v);
     correction = motor_pid_L._offset_value;
  } else if (_side == RIGHT_SIDE) {
    // Converts the speed of the encoder from mm/s to a relative motor speed
     _encoder_v = _encoder_v * MOTOR_SPEED_RATIO_R + MOTOR_SPEED_OFFSET;
     motor_pid_R._desired_value = _value;
     motor_pid_R.PID_output(_encoder_v);
     correction = motor_pid_R._offset_value;
  }
   //Serial.println(value);

  if(_side == WINCH || _side == MAGNET) {      //Added in both sides to ignore encoders !!!!!!!!!IMPORTANT!!!!!!!!!
    correction = 0;
    _value = value;
    _encoder_v = value;
  }
  
  int input_speed = _encoder_v+correction;

  if ((input_speed) > 1950) {
    //correction = 0;
    input_speed = 1950;
  } else if ((input_speed) < 1000) {
    //correction = 0;
    input_speed = 1000;
  }

  if(input_speed > 1550 && input_speed < 1700){
    input_speed = 1700;    
  }else if (input_speed < 1450 && input_speed > 1300){
    input_speed = 1300;    
  }
  
//  Serial.print("Input speed: ");
//  Serial.print(input_speed);
//  Serial.print(", Correction: ");
//  Serial.print(correction);
//  Serial.print(", Motor: ");
//  Serial.print(_side);
//  Serial.print(", Encoder_v: ");
//  Serial.println(_encoder_v);
  _motor_name.writeMicroseconds(input_speed); //input_speed
  //_value = _value+correction;
}

//-----------------------------------------------------------------------

/*
 *      reset_error
 *      
 *resets the error gains and correction for PID control.
 */
void motor_class::reset_error() {
  motor_pid_R.reset_values();
  motor_pid_L.reset_values();
  //Serial.println("values have been reset");
}


//-----------------------------------------------------------------------

/*
 *      imu_class
 *      
 *initalliser for imu_class
 */
void imu_class::initial(){

  _old_gyro = 0;
  _heading = 0;
  _bno = Adafruit_BNO055(55, 0x28);

  if (!_bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
}

void imu_class::calc_heading(){


  //gets data from gyro
  sensors_event_t orientationData;
  _bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  double new_gyro = orientationData.orientation.x;
  _heading = new_gyro; // * RAD_TO_DEG;
  //intergrated that data to get heading
  //_heading += 0.5 * (_old_gyro + new_gyro) * dt;
  if (_heading > 180.0) {
    _heading = _heading - 360;
  }

  //for intergration
  _old_gyro = _heading;
} 
