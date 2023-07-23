//************************************
//         IO_class.h    
//************************************
#ifndef IO_CLASS_H
#define IO_CLASS_H
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

#include "Arduino.h"
#include <Servo.h>
#include "Math_class.h"
#include "ultrasonic.hpp"

//Values for motors
#define MOTOR_SPEED_RATIO_R 14.7     //Switched for reversed motor polarity !!!!!!!!!! original 725.8
#define MOTOR_SPEED_RATIO_L 14.2     //Switched for reversed motor polarity !!!!!!!!!! original 725.8
#define MOTOR_SPEED_OFFSET 1500

#define MOTOR_CONTROL_PERIOD 0.1
#define MOTOR_P_GAIN 2.5
#define MOTOR_I_GAIN 1
#define MOTOR_D_GAIN 0

//Values for the TOF sensors
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35
#define TOF_PERIOD 60
#define L0_MAX_RANGE 1200
#define L1_MAX_RANGE 2300

#define US_MAX_RANGE 3000

typedef enum {
  LEFT_SIDE = 0,
  RIGHT_SIDE = 1,
  WINCH = 2,
  MAGNET = 3
} Robot_Side;

typedef enum {
  US = 0,
  TOF_L0,
  TOF_L1
} sensor_type;


//-----------------------------------------------------------------------

void TOF_init (void);

class sensor_class {
  public:

    /*
    *      sensor_class
    *      
    *initalliser for sensor_class
    */
    void initial (int pin, double alpha, sensor_type type);

    /*
    *      sensor_sample
    *      
    *samples the sensor
    */
    void sensor_sample();

    int _filtered_value; //filtered sensor value

    double _dist;

  private:
 
    int _pin; //pin the sensor is attached to
    int _value; //current sampled sensor value
    double _alpha; //recursive filter constant
    sensor_type _type;
};

//-----------------------------------------------------------------------

class motor_class {
  public:

    /*
    *      motor_class
    *      
    *initalliser for motor_class
    */
    void initial(int pin, int value, Servo motor_name, int side);

    void Setvalue(int value);

    void reset_error();

    int _value; //motor speed value

  private:
    Servo _motor_name; //motor Servo name
    PID_class motor_pid_L;
    PID_class motor_pid_R;
    double _encoder_dist;
    double _prev_dist;
    double _encoder_v;
    int _side;
    float _dt;
};



//-----------------------------------------------------------------------

class imu_class {
  public:

    void initial();
    void calc_heading();

    double _heading;

 private:

  //accelerometer constants
    double _old_gyro;
    Adafruit_BNO055 _bno;
    double dt = 0.01;
    
};

#endif //IO_CLASS_H
