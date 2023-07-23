//************************************
//         pickup.h    
//************************************
#ifndef PICKUP_H
#define PICKUP_H

#include "Arduino.h"
#include "Control_class.h"
#include "IO_class.h"

#define WINCH_MOTOR_PIN 29
#define WINCH_LIMIT_PIN 31
#define IR_LIMIT_PIN 30
#define INDUCTOR_PIN 20

#define STATIONARY_VALUE 1500

//magnet settings
#define MAGNET_PIN 28
#define MAGNET_ON_VALUE 1980

//stepper stuff
#define STEP_DIR_PIN 27
#define STEP_STEP_PIN 22  
#define STEP_LIMIT_PIN 32
#define FRONT_LIMIT 26
#define LEAD_OF_THREAD 8 //in mm
#define STEP_ANGLE 0.9 //in deg
#define POSITION_DIST 75

//smart servo
#define PIN_SW_RX 7
#define PIN_SW_TX 8
#define SERVO_GATE 4
#define SERVO_BACK_DOOR 3
#define SERVO_LOCK_IN 2
#define OPEN_VALUE 125
#define CLOSED_VALUE 285
#define PICK_UP_VALUE 285 
#define LOCK_IN_OPEN 75
#define LOCK_IN_CLOSED 450

#define CLOSED_FLAP_VALUE 1000
#define OPEN_FLAP_VALUE 400

//delay thresholds
#define DETECT_DUMMY_PERIOD 50
#define OPEN_GATE_PERIOD 50

typedef enum {
  PICK = 0,
  PLACE = 1,
  NOTHING = 2
} direction_type;

typedef enum {
  REVERSE_STEP = 0,
  FORWARD_STEP = 1
} stepper_direction;

typedef enum {
  CALIBRATION = 0,
  NORMAL = 1,
  DROPOFF = 2,
  HOME = 3
} stepper_mode;

//****************************************************************************
/*PICKUP_10MS
  -- called every 10ms
  */
void pickup_10ms (void);

//****************************************************************************
/*PICKUP_INITIAL
  initalises pick up functions
  */
void pickup_initial();

//****************************************************************************
/*SET_BACK_DOOR
  shuts and opens back door.
  */
void set_back_door(bool open_flap);

//****************************************************************************
/*SET_LOCK_IN
  shuts and opens back door.
  */
void set_lock_in(bool open_flap);

//****************************************************************************
/*OBJECT_DETECTED
  interrupt triggered function from weight trigger IR limit switch
  */
void object_detected();

//****************************************************************************
/*INDUCTOR_CHECK
  checks if the inductor has detected anything in the IR limit has been triggered.
  -- called every 10ms
  */
void inductor_check (void);

//****************************************************************************
/*MOVE_DOOR
  moves the door to the inputted position
  */
void move_door(int gate_position);

//****************************************************************************
/*DUMMY_REJECT
  Opens gate for set period to reject a weight.
  -- called every 10ms
  */
void dummy_reject();

//****************************************************************************
/*UPDATE_VALUES
  updates the winch up and down values dependent on the way it is wound
  */
void update_values();

//****************************************************************************
/*WINCH_MOTOR_MOVE
  main winch motor functionailty
  -- called every 10ms
  */
void Winch_motor_move();

//****************************************************************************
/*STEPPER_MOTOR
  excutes stepper motor states
  -- called every 1ms
  */
void stepper_motor();

//****************************************************************************
/*WINCH_CALIBRATION
  calibrates the winch
  -- is delay based --
  */
void winch_calibration ();

#endif //PICKUP_H
