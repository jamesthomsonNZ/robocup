//************************************
//         pickup.cpp    
//************************************
#include "Arduino.h"
#include "Control_class.h"
#include "pickup.h"
#include "IO_class.h"
#include "logic.h"
#include <SoftwareSerial.h>
#include <HerkulexServo.h>


motor_class magnet;

volatile int weight_count = 0;
volatile int retry_count = 0;

//object detection globals
bool f_reject = false;
volatile int not_metal_count;
volatile int reject_count;
bool f_door_open = false;
bool f_OBJECT_DETECTED = false;

//stepper globals
bool step_f = false;
volatile int steps_counted = 0;
bool move_stepper_f = false;
stepper_mode step_mode;
bool step_return_state;

//winch globals
volatile int winch_state = 0;
volatile int delay_count = 0;
bool f_reverse_winch = false;
volatile int up_value = 0;
volatile int down_value = 0;
motor_class winch_motor;
direction_type winch_high_state = NOTHING;

//smart servo shit
SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    servo_gate(herkulex_bus, SERVO_GATE);
HerkulexServo    servo_back_door(herkulex_bus, SERVO_BACK_DOOR);
HerkulexServo    servo_lock_in(herkulex_bus, SERVO_LOCK_IN);

//****************************************************************************
/*PICKUP_10MS
  -- called every 10ms
  */
void pickup_10ms (void) {
  inductor_check();
  dummy_reject();
  Winch_motor_move();
}


//****************************************************************************
/*PICKUP_INITIAL
  initalises pick up functions
  */
void pickup_initial (){
  //attaches interrupt to IR Limit switch
  attachInterrupt(digitalPinToInterrupt(IR_LIMIT_PIN), object_detected, CHANGE);

  //intialises winch and magnet as motor objects
  Servo winch;
  Servo magnet_servo;
  winch_motor.initial(WINCH_MOTOR_PIN, 1500, winch, WINCH);
  magnet.initial(MAGNET_PIN, 1500, magnet_servo, MAGNET);

  //sets pin modes
  pinMode(WINCH_LIMIT_PIN, INPUT);
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_STEP_PIN, OUTPUT);
  pinMode(FRONT_LIMIT, INPUT_PULLUP);
  analogWriteFrequency(22, 1000);

  //make sure gate is closed
  servo_serial.begin(115200); //set servo serial port for communication
  delay(500);
  servo_gate.setTorqueOn();
  servo_back_door.setTorqueOn();
  servo_lock_in.setTorqueOn();
  move_door(CLOSED_VALUE);
  set_lock_in(true);
  set_back_door(true); // true == open

  digitalWrite(13,LOW);

  //Set zeros
  not_metal_count = 0;
  reject_count = 0;

  step_mode = NORMAL;

  Serial.println("Pick-Up initaiated");
}

//****************************************************************************
/*SET_BACK_DOOR
  shuts and opens back door.
  */
void set_back_door(bool open_flap){
  herkulex_bus.prepareSynchronizedMove(100);
  if (!open_flap){
    servo_back_door.setPosition(CLOSED_FLAP_VALUE);
  } else if (open_flap) {
    servo_back_door.setPosition(OPEN_FLAP_VALUE);
  }
  herkulex_bus.executeMove();
}

//****************************************************************************
/*SET_LOCK_IN
  shuts and opens back door.
  */
void set_lock_in(bool open_flap){
  herkulex_bus.prepareSynchronizedMove(100);
  if (!open_flap){
    servo_lock_in.setPosition(LOCK_IN_CLOSED);
  } else if (open_flap) {
    servo_lock_in.setPosition(LOCK_IN_OPEN);
  }
  herkulex_bus.executeMove();
}

//****************************************************************************
/*OBJECT_DETECTED
  interrupt triggered function from weight trigger IR limit switch
  */
void object_detected (void){
  f_OBJECT_DETECTED = true; 
}

//****************************************************************************
/*INDUCTOR_CHECK
  checks if the inductor has detected anything in the IR limit has been triggered.
  -- called every 10ms
  */
void inductor_check (void) {
  int inductor = digitalRead(INDUCTOR_PIN);

  //if IR Limit has been triggered
  if ((f_OBJECT_DETECTED == true || inductor == LOW ) && winch_high_state == NOTHING && step_mode == NORMAL){
    
    //Induction sensor is triggered
    if (inductor == LOW) {
      set_lock_in(false);
      weight_count += 1;      
      set_back_door(false);
      //high level state changes to PICKUP when stepper starts to move
      //start pick up sequence
      winch_high_state = PICK;

      not_metal_count = 0;

    //increment count if not metallic
    } else if(not_metal_count < DETECT_DUMMY_PERIOD && High_level_state != PICKUP){
      not_metal_count += 1;

    //if count is over threshold then reject
    } else if (not_metal_count >= DETECT_DUMMY_PERIOD && High_level_state != PICKUP) {
      //reject weight
      f_reject = true;
      //reset dummy count
      not_metal_count = 0;
    }
    
    //reset flag
    f_OBJECT_DETECTED = false;
  }
  
  //if the inductor is not on and dummy count has been started
  if(inductor == HIGH && not_metal_count < DETECT_DUMMY_PERIOD && not_metal_count > 0){
    not_metal_count += 1;

  //if dummy count is greater than a period then dispose.
  } else if (not_metal_count >= DETECT_DUMMY_PERIOD) {
    f_reject = true;
    not_metal_count = 0;
  }

  if (weight_count >= 1){
    //set_back_door(false);
    digitalWrite(13, HIGH);
    //High_level_state = RETURN_HOME;    
  } else {
    //set_back_door(true);
  }
  
} 

//****************************************************************************
/*MOVE_DOOR
  moves the door to the inputted position
  */
void move_door(int gate_position){
  herkulex_bus.prepareSynchronizedMove(100);
  servo_gate.setPosition(gate_position);
  herkulex_bus.executeMove();
}

//****************************************************************************
/*DUMMY_REJECT
  Opens gate for set period to reject a weight.
  -- called every 10ms
  */
void dummy_reject(){
  
  if(f_reject == true){
    //if its open then add to the counter
    if (f_door_open == true) {
      reject_count += 1;
    // if door shut then open
    } else if (f_door_open == false) {
      // open the door
      move_door(OPEN_VALUE);
      Serial.println("Open door");
      f_door_open = true;
    }
  } 

  //if open longer than period then close
  if (f_reject == true && reject_count > OPEN_GATE_PERIOD){
    //close the door
    move_door(CLOSED_VALUE);
    Serial.println("closed door");
    f_door_open = false;
    reject_count = 0;
    f_reject = false;
  }
}

//****************************************************************************
/*UPDATE_VALUES
  updates the winch up and down values dependent on the way it is wound
  */
void update_values() {
  if (f_reverse_winch == false) {
    up_value = 1900;
    down_value = 1100;
  } else {
    up_value = 1100;
    down_value = 1900;
  }
}

//****************************************************************************
/*WINCH_MOTOR_MOVE
  main winch motor functionailty
  -- called every 10ms
  */
void Winch_motor_move() {
  update_values();
  int winch_limit = digitalRead(WINCH_LIMIT_PIN);
  
  if(winch_high_state == PICK){

    Serial.print("winch state PICK ");
    Serial.println(winch_state);
  
    //first pick up segment
    if(winch_state == 0){
      move_door(PICK_UP_VALUE);
      delay_count += 1;

      if (delay_count > 20) { 
        //lowers winch
        winch_motor.Setvalue(down_value);
        magnet.Setvalue(MAGNET_ON_VALUE);
        Serial.println("Magnet ON");
        winch_state = 1;
        High_level_state = PICKUP; // stops 
        delay_count = 0;
      }
    //second pickup segment
    //lowers winch to pick up weight until it hits the top
    } else if (winch_state == 1) {
      //winch has returned to top
      if (winch_limit == HIGH) {
        winch_motor.Setvalue(STATIONARY_VALUE);
        f_reverse_winch = not(f_reverse_winch);
        update_values();
        winch_state = 2;        
      }

    // third pickup segment
    //sets the winch below top deck so theres no rubbing.
    // open loop timing based
    } else if (winch_state == 2) {
      
      delay_count += 1;
      if (delay_count == 5){
        winch_motor.Setvalue(down_value);
        
      } else if (delay_count >= 10){
        winch_motor.Setvalue(STATIONARY_VALUE);
        delay_count = 0;

        //tells the stepper to go back.
        winch_state = 0;

        int inductor = digitalRead(INDUCTOR_PIN);
        int IR = digitalRead(IR_LIMIT_PIN);

        // if the pick up fails then retry
        if (inductor == LOW || IR == HIGH) {
          winch_state = 0;
          retry_count += 1;
          Move.twitch();
        } else {
          step_mode = DROPOFF;
          winch_high_state = NOTHING;
        }

        // if failed to many times then move on
        if (retry_count >= 3) {
          winch_high_state = NOTHING;
          retry_count = 0;
          f_reject = true; 
          weight_count += -1;
          High_level_state = SEARCHING;
          set_lock_in(true);
        }
      }
    }

  } else if(winch_high_state == PLACE){

    Serial.print("winch state PLACE ");
    Serial.println(winch_state);
    //segment 1 of winch drop off
    // drop weight in tray
    if (winch_state == 0) {
      winch_motor.Setvalue(down_value);
      delay_count += 1;
  
      //basically a delay of 500ms
      // turns off magnet and pauses winch
      if (delay_count >= 15) {
      winch_motor.Setvalue(STATIONARY_VALUE);
      magnet.Setvalue(STATIONARY_VALUE);
      Serial.println("Magnet OFF");
      delay_count = 0;
      winch_state = 1;
      }
  
    //segment 2 of winch drop off
    // start winch accent
    } else if (winch_state == 1) {
      delay_count += 1;
      
      // waiting for pause of 100ms before going up
      if (delay_count >= 10) {
        winch_motor.Setvalue(up_value);
        delay_count = 0;
        winch_state = 2;
      }
  
    //segment 3 of winch drop off 
    } else if (winch_state == 2) {
      if (winch_limit == HIGH){
        winch_motor.Setvalue(STATIONARY_VALUE);
        winch_state = 3;
      }
  
    //segment 4 of winch drop off 
    // sets winch below desk to avoid rubbing
    } else if (winch_state == 3) {
      delay_count += 1;
      if (delay_count == 5){
        winch_motor.Setvalue(down_value);
        
      } else if (delay_count >= 15){
        winch_motor.Setvalue(STATIONARY_VALUE);
        delay_count = 0;
  
        //tells the stepper to go back.
        step_mode = HOME;
        winch_state = 0;
        winch_high_state = NOTHING;
        High_level_state = SEARCHING;
      }
    }
  }
}

//****************************************************************************
/*STEPPER_MOTOR
  excutes stepper motor states
  -- called every 1ms
  */
void stepper_motor(){
  //calibrating position
  
  if (step_mode == CALIBRATION) {
    Serial.println("Step Cali");
    // go to limit
    digitalWrite(STEP_DIR_PIN, REVERSE_STEP);
    int stepper_limit = digitalRead(STEP_LIMIT_PIN);
    while(stepper_limit == LOW){
      stepper_limit = digitalRead(STEP_LIMIT_PIN);
      digitalWrite(STEP_STEP_PIN,LOW);
      delayMicroseconds(20);
      digitalWrite(STEP_STEP_PIN,HIGH);
      delay(1);
    }

    // go to postion
    digitalWrite(STEP_DIR_PIN, FORWARD_STEP);
    int distance = POSITION_DIST; // [mm]
    int steps = (distance * 360) / (LEAD_OF_THREAD * STEP_ANGLE);
    Serial.println(steps);
    analogWrite(22, 200);
    int Front_limit = digitalRead(FRONT_LIMIT);
    while(Front_limit == LOW) {
      Front_limit = digitalRead(FRONT_LIMIT);
    }
    analogWrite(22, 0);
    step_mode = NORMAL;

  Serial.println("Stepper Calibration Complete");
    
  } else if (step_mode == DROPOFF) {
    set_lock_in(true);
    Serial.println("STEP MODE DROPOFF");
    //set the direction of the stepper to be backward.
    digitalWrite(STEP_DIR_PIN, REVERSE_STEP);

    // move until limit is hit
    int stepper_limit = digitalRead(STEP_LIMIT_PIN);

    analogWrite(STEP_STEP_PIN, 200);
    if(stepper_limit == HIGH){
      analogWrite(STEP_STEP_PIN, 0);
      step_mode = NORMAL;

      //tell winch motor to place the weight. 
      winch_high_state = PLACE;
    }
    
  } else if (step_mode == HOME) {
    int distance = POSITION_DIST; // [mm]
    int steps = (distance * 360) / (LEAD_OF_THREAD * STEP_ANGLE);
    int timer_steps = ((steps)/100 + 30);
    Serial.println(timer_steps);
    
    if (step_return_state == false) {
      //set the direction of the stepper to be forward.
      digitalWrite(STEP_DIR_PIN, FORWARD_STEP);
  
      //move till in postion
      analogWrite(STEP_STEP_PIN, 200);
      step_return_state = true;
    } else if (step_return_state == true) {
      int Front_limit = digitalRead(FRONT_LIMIT);
      if (Front_limit == HIGH){
        analogWrite(STEP_STEP_PIN, 0);
        step_mode = NORMAL;
        High_level_state = SEARCHING;
        step_return_state = false;
      }
    }
    
  }
  
}

//****************************************************************************
/*WINCH_CALIBRATION
  calibrates the winch
  -- is delay based --
  */
void winch_calibration (){
  Serial.print("Winch limit hit ");
  Serial.println(digitalRead(WINCH_LIMIT_PIN));
  Serial.println("Calibrating Winch");

  // goes a way and checks if it hits the limit 
  winch_motor.Setvalue(1200);
  delay(100);
  winch_motor.Setvalue(1500);
  delay(100);
  int winch_limit = digitalRead(WINCH_LIMIT_PIN);
  if (winch_limit == HIGH){
    f_reverse_winch = true;

  // goes other way until it hits a limit
  } else {
    while(winch_limit == LOW) {
      winch_limit = digitalRead(WINCH_LIMIT_PIN);
      winch_motor.Setvalue(1800);
      delay(100);
      winch_motor.Setvalue(1500);
    }
    winch_motor.Setvalue(1500);
    if (winch_limit == HIGH){
      f_reverse_winch = false;
    }
  }

  //lowers a bit lower than limit
  update_values();
  winch_motor.Setvalue(down_value);
  delay(100);
  winch_motor.Setvalue(1500);

  step_mode = CALIBRATION;
  stepper_motor();
  Serial.print("Reverse Winch "); 
  Serial.println(f_reverse_winch);
  Serial.println("Total Calibration Complete");
}
