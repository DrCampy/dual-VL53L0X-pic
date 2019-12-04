/* Defines bit position for used pins */
#define XSHUT_L 15
#define XSHUT_R 12
#define INT_L 14
#define INT_R 13
#define INT_OUT 11
#define DIP1 7
#define DIP2 6
#define DIP3 5
/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__PIC24E__)
    	#include <p24Exxxx.h>
    #elif defined (__PIC24F__)||defined (__PIC24FK__)
	#include <p24Fxxxx.h>
    #elif defined(__PIC24H__)
	#include <p24Hxxxx.h>
    #endif
#endif

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system.h"        /* System funct/params, like osc/peripheral config */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */


/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    TRISBbits |= (1 << XSHUT_L) + (1 << XSHUT_R);
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    
    /*
     * .X_Shut low for both sensors
     * .X_Shut_R high
     * (wait)
     * .Configure SensorR for 2.8V
     * .Change adress of right sensor
     * .X_Shut_L high
     * (wait)
     * .Configure sensorL for 2.8V
     * Init
     * Check if we have to do  SPAD calibration otherwise loads it
     * Temp calibration
     * Check if we need to do offset or xTalk calibration otherwise load them
     * 
    */
    
    
    /* TODO <INSERT USER APPLICATION CODE HERE> */

    while(1)
    {

    }
}
