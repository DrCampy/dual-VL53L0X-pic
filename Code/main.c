/* Defines bit position for used pins */
#define XSHUT_L 15
#define XSHUT_R 12
#define INT_L 14
#define INT_R 13
#define INT_OUT 11
#define DIP1 7
#define DIP2 6
#define DIP3 5

/*Define needed for the API to use I2C in 2.8V*/
#define USE_I2C_2V8


/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
#include <xc.h>            /* Device header file                              */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include <libpic30.h>      /* __delay_ms function                             */
#include "Api/inc/core/vl53l0x_api.h" /*VL53L0X Api                           */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
volatile bool i2c_slave_ready = false;


/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    
    /* Configure the oscillator for the device */
    ConfigureOscillator();
    
    VL53L0X_Dev_t r, l;
    VL53L0X_DEV RightSensor = &r, LeftSensor = &l;
    
    /*Configure shutdown pins as outputs*/
    TRISB &= !((1 << XSHUT_L) + (1 << XSHUT_R));
    
    /*Wake up right sensor*/
    LATB = 1 << XSHUT_R;
    __delay_ms(2);
    
    /*VL53L0X_DataInit(VL53L0X_DEV Dev)*/
    VL53L0X_SetDeviceAddress(RightSensor, 0x54);
    
    /*Turns on left sensor*/
    LATB |= 1 << XSHUT_L;
    __delay_ms(2);
    
    /*
     * .X_Shut low for both sensors
     * .X_Shut_R high
     * (wait)
     * .Change adress of right sensor
     * .X_Shut_L high
     * (wait)
     * Init
     * Check if we have to do  SPAD calibration otherwise loads it
     * Temp calibration
     * Check if we need to do offset or xTalk calibration otherwise load them
     * 
    */
    
    
    /* TODO <INSERT USER APPLICATION CODE HERE> */

    while(1)
    {
        if(i2c_slave_ready == true){
            /*Manage slave I2C*/
            i2c_slave_ready = false;
        }
    }
}
