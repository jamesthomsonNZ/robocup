/**
 * @file ultrasonic.hpp
 * @author Ethan Ng (eng42@uclive.ac.nz)
 * @brief 
 * @version 0.1
 * @date 2022-08-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef ULTRASONIC_H
#define ULTRASONIC_H


#include <Arduino.h>
#include <stdint.h>

// Number of ultrasonic sensors used
#define US_NUM 2
// Parameters for ultrasonic sensor 0
#define US_0
#define US_TRIG_0 15
#define US_ECHO_0 14
// Parameters for ultrasonic sensor 1
#define US_1
#define US_TRIG_1 25
#define US_ECHO_1 24
/**
 * @brief Struct to hold all data about a single ultrasonic sensor
 * 
 */
typedef struct {
    uint8_t trigPin; 
    uint8_t echoPin; 
    volatile uint32_t sentTime; 
    volatile uint32_t recvTime; 
    uint32_t pulseSentTime; 
} UltrasonicSensor_t; 


/**
 * @brief Default array of ultrasonic sensors
 * 
 */
extern UltrasonicSensor_t usArray[US_NUM]; 


/**
 * @brief Initialises the timer (GPT1) in order to perform the timing of 
 * when the interrupts are received. 
 * 
 */
void usCounterInit(void); 


/**
 * @brief Adds an ultraonic sensor to the array (stored in ultrasonic.cpp), 
 * sets the parameters and pin modes of the two US pins, and attaches the 
 * corresponding interrupt to the echo pin. 
 * 
 * @param trigPin Trigger pin for ultrasonic sensor
 * @param echoPin Echo pin for ultrasonic sensor
 * @param num Ultrasonic sensor number in the array
 */
void usAddToArray(uint8_t trigPin, uint8_t echoPin, uint8_t num); 


/**
 * @brief Creates a new UltrasonicSensor_t struct based on the provided data
 * and returns this struct. Not recommended for  use; just use usAddToArray 
 * with the default array instead unless there is a special need to just get
 * an UltrasonicSensor_t object without adding it to the array. Because the 
 * struct is what carries the send and receive times, you have to do the ISR
 * attachment yourself here. 
 * 
 * @param trigPin Trigger pin for ultrasonic sensor
 * @param echoPin Echo pin for ultrasonic sensor
 * @return UltrasonicSensor_t The created ultrasonic sensor struct
 */
UltrasonicSensor_t usNew(uint8_t trigPin, uint8_t echoPin);


/**
 * @brief Pings a single ultrasonic sensor (2us low, 10us high, low) 
 * 
 * @param sensor Pointer to the UltrasonicSensor_t struct
 */
void usPing(UltrasonicSensor_t* sensor); 


/**
 * @brief Pings an array of ultrasonic sensors, starting from the sensor
 * pointed to by the sensors parameter and counting until num sensors have 
 * been pinged. The pinging is done concurrently (all brought low, wait 2us, 
 * all brought high, wait 10us, all brought low). 
 * 
 * @param sensors Pointer to the first sensor in the array to ping
 * @param num Number of sensors to ping concurrently
 */
void usPingArray(UltrasonicSensor_t sensors[], uint8_t num); 


/**
 * @brief Calculate the distance based on the echo time and received time
 * in the sensor. Call this function approximately 30ms after the ping, so 
 * that you can guarantee that the echo has either returned or will never
 * return. Returns 0 if it thinks an error has occurred. 
 * 
 * @param sensor Pointer to the sensor to calculate the distance measurement for
 * @return uint16_t Distance measurement in mm
 */
uint16_t usCalc(UltrasonicSensor_t* sensor); 


/**
 * @brief Calculates the distance based on the echo time for an entire array; see
 * usCalc
 * 
 * @param sensors Pointer to the first sensor in the array to calculate distance
 * measurements for
 * @param num Number of sensors in the array
 * @param distances Array to store the distance measurements in mm
 */
void usCalcArray(UltrasonicSensor_t sensors[], uint8_t num, uint16_t* distances); 


/**
 * @brief Handles the interrupt, sets either the sent time or the received time to
 * the current counter time depending on whether the pin is high. This is because
 * the echo pin will first be brought high and then brought low when the echo is 
 * received back. This function should only be called by one of the ISRs. 
 * 
 * @param sensor Pointer to the sensor to handle the interrupt for
 */
inline void usHandleInterrupt(UltrasonicSensor_t* sensor) {
    if (digitalReadFast(sensor->echoPin)) {
        sensor->sentTime = GPT1_CNT; 
    } else {
        sensor->recvTime = GPT1_CNT; 
    }
}


#endif
