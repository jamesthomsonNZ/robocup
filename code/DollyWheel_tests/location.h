//************************************
//         location.h    
//************************************


#ifndef LOCATION_H
#define LOCATION_H

/*
 *      location_init
 *      
 *initalises the location variables and functions
 */
void location_init();


/*
 *      encoderphaseA_interrupt
 *      
 *does the interupt on the rising edge of the
 *encoders phase A
 */
void encoderphaseA_interrupt();

/*
 *      heading
 *      
 *calculates the robots heading using the imu
 */
void heading();

/*
 *      get_heading
 *      
 *returns the current heading value
 */
int get_heading();

/*
 *      location
 *      
 * calculates the robots current location with the imu
 * and encoder
 */
void location();

#endif /* LOCATION_H */
