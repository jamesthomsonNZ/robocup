
/********************************************************************************
 *                               ROBOCUP TEMPLATE                              
 *        
 *  
 *  This is a template program design with modules for 
 *  different components of the robot, and a task scheduler
 *  for controlling how frequently tasks sholud run
 *  
 *  
 *  written by: Logan Chatfield, Ben Fortune, Lachlan McKenzie, Jake Campbell
 *  
 ******************************************************************************/

#include <Servo.h>                  //control the DC motors
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler 

// Custom headers
#include "logic.h"
#include "Control_class.h"
#include "encoder.h"
#include "pickup.h"

//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Task period Definitions
// ALL OF THESE VALUES WILL NEED TO BE SET TO SOMETHING USEFUL !!!!!!!!!!!!!!!!!!!!
#define HUNDY_MS_PERIOD                      100
#define TEN_MS_PERIOD                        10
#define ONE_MS_PERIOD                        1


// Task execution amount definitions
// -1 means indefinitely
#define ALWAYS       -1

// Pin deffinitions
#define IO_POWER  49
#define START_BUTTON 23

// Serial deffinitions
#define BAUD_RATE 9600

// Start button period of held
#define START_BUTTON_PERIOD 10

//**********************************************************************************
// Task Scheduler and Tasks
//**********************************************************************************

/* The first value is the period, second is how many times it executes
   (-1 means indefinitely), third one is the callback function */

// Tasks for reading sensors 
Task tRead_sensors(5*TEN_MS_PERIOD ,      ALWAYS ,    &TOF_sensor_sample); //needed
Task tdrive_motor_updates(HUNDY_MS_PERIOD ,        ALWAYS ,       &update_motors);//needed
Task tMovement_task(TEN_MS_PERIOD ,     ALWAYS ,      &move_function);
Task tPickUp_task(TEN_MS_PERIOD ,     ALWAYS ,      &pickup_10ms);
Task tstepper_task(HUNDY_MS_PERIOD ,     ALWAYS ,      &stepper_motor);
Task tEncoder_task(HUNDY_MS_PERIOD,     ALWAYS,       &calc_encoder_speeds);
Task tUS_Sensors_task(3*TEN_MS_PERIOD,    ALWAYS,       &us_sensor_sample);
Task tLocation_task(HUNDY_MS_PERIOD,    ALWAYS,       &update_location);
Task tLEDs(1, 1, &LED_colour_change);
Task tstates(TEN_MS_PERIOD, ALWAYS, &states_update);

Scheduler taskManager;

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
void task_init();

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("init started");
  pin_init();
  task_init();
  Serial.println("task and pin initalised");
  Wire.begin();
  Serial.println("wire initalised");
  logic_inital();
  Serial.println("logic initalised");
  initialise_encoders();
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work) 
// Set as high or low
//**********************************************************************************
void pin_init(){
    
    //Serial.println("Pins have been initialised \n"); 
    pinMode(13, OUTPUT);
    pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
    digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
    pinMode(23, INPUT);
}

//**********************************************************************************
// Initialise the tasks for the scheduler
//**********************************************************************************
void task_init() {  
  
  // This is a class/library function. Initialise the task scheduler
  taskManager.init();     
 
  // Add tasks to the scheduler
  taskManager.addTask(tRead_sensors); 
  taskManager.addTask(tdrive_motor_updates); 
  taskManager.addTask(tMovement_task);
  taskManager.addTask(tPickUp_task);
  taskManager.addTask(tstepper_task);
  taskManager.addTask(tEncoder_task);
  taskManager.addTask(tUS_Sensors_task);
  taskManager.addTask(tLocation_task);
  taskManager.addTask(tLEDs);
  taskManager.addTask(tstates);  
  

  //enable the tasks
  tRead_sensors.enable();
  tdrive_motor_updates.enable();
  tMovement_task.enable();
  tPickUp_task.enable();
  tstepper_task.enable();
  tEncoder_task.enable();
  tUS_Sensors_task.enable();
  tLocation_task.enable();
  tLEDs.enable();
  tstates.enable();
}


static int start_f = 0;

//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
  if (High_level_state == CALIBRATION_STATE) {
    winch_calibration();
    High_level_state = SEARCHING;
  }

  // Starts the main taskscheduler if the BLUE 'START' button is pressed
  if (digitalRead(START_BUTTON) == HIGH) {
    start_f = 1;
  }
  if (start_f) {
    taskManager.execute();    //execute the scheduler
  }
  
}
