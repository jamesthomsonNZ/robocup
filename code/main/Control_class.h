//************************************
//         Control_class.h    
//************************************
#ifndef CONTROL_CLASS_H
#define CONTROL_CLASS_H

#include "Arduino.h"
#include <Servo.h>
#include "IO_class.h"

#define TRACK_WIDTH 150
#define LEFT_MOTOR_PIN 1
#define RIGHT_MOTOR_PIN 0

#define DIST_THRESHOLD 300
#define WEIGHT_DIST_THRESHOLD 500
#define DIFFERENTIAL_THRESHOLD 50
#define RAMP_THRESHOLD_PERIOD 20    //10 is approx 0.1s

#define RIGHT_SENSOR_ANGLE 30   //deg
#define LEFT_SENSOR_ANGLE 30    //deg
#define TIME_DELAY 10           //time delay to update movements in the same state

#define TURN_P_GAIN 20
#define TURN_I_GAIN 2
#define TURN_D_GAIN 0
#define TURN_RATIO 30


extern imu_class globalimu;

typedef enum {
  FORWARD = 0,
  REVERSE = 1,
  TURNING = 2,
  STATIONARY = 3,
  STEADY = 4
} Move_State;

typedef enum {
  PARALLEL = 0,
  WALL = 1,
  ANGLE_OBJECT = 2,
  FLAT_OBJECT = 3,
  WEIGHT = 4,
  RAMP = 5,
  NO_DETECTION = 6
} object_state;

typedef enum {
  SEARCHING = 0,
  PICKUP = 1,
  RETURN_HOME = 2,
  CALIBRATION_STATE = 3
} high_state;

extern high_state High_level_state;

void control_init();

class movement_class {
  public:

    void initial (double tolerance);

    void turn (int angle);

    void forward (int speed_value);

    //void reverse (int speed_value);

    void motor_update();

    void twitch();

    void stationary ();

    Move_State _state;

    int _angle;

    int _speed_value;

  private: 

    double _tolerance;
    PID_class PID_turning_angle;
    motor_class Left_motor;
    motor_class Right_motor;
    double _angle_tol = 10;
};

class states_class {
  public:

    void initial();

    void state_refresh();

    void do_state();

  private:
    object_state _left_state;
    object_state _right_state;
    object_state _front_state;
    object_state _low_right;
    object_state _low_left;
    
    object_state _change_left;
    object_state _change_right;
    object_state _change_front;
    object_state _change_low_right;
    object_state _change_low_left;

    bool in_channel;
    int _delay_count;
};
#endif //CONTROL_CLASS_H
