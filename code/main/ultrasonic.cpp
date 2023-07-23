#include <Arduino.h>
#include "ultrasonic.hpp"


/**
 * Array to hold data for the array of ultrasonic sensors
 */
UltrasonicSensor_t usArray[US_NUM]; 


#ifdef US_0
void usCB_0(void) {
    usHandleInterrupt(&usArray[0]); 
}
#endif

#ifdef US_1
void usCB_1(void) {
    usHandleInterrupt(&usArray[1]); 
}
#endif

#ifdef US_2
void usCB_2(void) {
    usHandleInterrupt(&usArray[2]); 
}
#endif

#ifdef US_3
void usCB_3(void) {
    usHandleInterrupt(&usArray[3]); 
}
#endif


void (*usCallbacks[]) (void) = {
    #ifdef US_0 
        usCB_0, 
    #endif 
    #ifdef US_1
        usCB_1,
    #endif
    #ifdef US_2
        usCB_2,
    #endif
    #ifdef US_3
        usCB_3,
    #endif
}; 


// Initialise the counters
void usCounterInit(void) {
    // Set up the timer source
    // Bits 8 - 6 indicate to use the 24MHz internal high speed clock
    // Bit 0 enables the counter
    GPT1_CR = (GPT1_CR & (~(1 << 6 | 1 << 8))) | (1 << 7) | (1 << 0); 

    // Set the prescaler to 35 so that 1mm = 1 tick
    GPT1_PR = 140; 
}


// Add an ultrasonic sensor to the array and attach the interrupt
void usAddToArray(uint8_t trigPin, uint8_t echoPin, uint8_t num) {
    // Set the correct pin mode
    pinMode(trigPin, OUTPUT); 
    pinMode(echoPin, INPUT); 

    // Set the parameters in the array
    usArray[num].trigPin = trigPin; 
    usArray[num].echoPin = echoPin; 

    // Attach the interrupts
    attachInterrupt(echoPin, usCallbacks[num], CHANGE); 
}


// Create a new ultrasonic sensor struct
UltrasonicSensor_t usNew(uint8_t trigPin, uint8_t echoPin) {
    // Set the correct pin mode
    pinMode(trigPin, OUTPUT); 
    pinMode(echoPin, INPUT); 

    // Create a sensor and set its parameters
    UltrasonicSensor_t sensor; 
    sensor.trigPin = trigPin; 
    sensor.echoPin = echoPin; 
    sensor.sentTime = 0; 
    sensor.recvTime = 0; 

    // Return our UltrasonicSensor object
    return sensor; 
}


// Ping a single ultrasonic sensor
void usPing(UltrasonicSensor_t* sensor) {
    digitalWriteFast(sensor->trigPin, LOW); 
    delayMicroseconds(2); 
    digitalWriteFast(sensor->trigPin, HIGH); 
    delayMicroseconds(10); 
    digitalWriteFast(sensor->trigPin, LOW); 
    sensor->pulseSentTime = GPT1_CNT; 
}


// Ping an array of the specified number of ultrasonic sensors
void usPingArray(UltrasonicSensor_t sensors[], uint8_t num) {
    // Write all trig pins in the array low for 2us
    for (int i = 0; i < num; ++i) 
        digitalWriteFast(sensors[i].trigPin, LOW); 
    delayMicroseconds(2); 

    // Write all trig pins in the array high for 10us
    for (int i = 0; i < num; ++i) 
        digitalWriteFast(sensors[i].trigPin, HIGH); 
    delayMicroseconds(10); 

    // Write all trig pins in the array low until next call
    for (int i = 0; i < num; ++i) {
        digitalWriteFast(sensors[i].trigPin, LOW); 
        sensors[i].pulseSentTime = GPT1_CNT; 
    }
}


// Calculate distance for single sensor
uint16_t usCalc(UltrasonicSensor_t* sensor) {
    uint32_t sentTime = sensor->sentTime; 
    uint32_t recvTime = sensor->recvTime; 
    uint32_t pulseSentTime = sensor->pulseSentTime; 

    if (sentTime > recvTime || pulseSentTime > recvTime) {
        // Then either the pulse has not come back or the 
        // GPT counter has overflown, in either case an error
        // has occurred
        return 0; 
    } else {
        // Numbers here are reasonable. The prescaler has been 
        // chosen so that 1 tick = 1mm, so we just need to return
        // the difference of the two times. 
        return recvTime - sentTime; 
    }
}


// Calculate distance for an array of sensors
void usCalcArray(UltrasonicSensor_t sensors[], uint8_t num, uint16_t* distances) {
    for (int i = 0; i < num; ++i) {
        distances[i] = usCalc(sensors + i); 
    }
}
