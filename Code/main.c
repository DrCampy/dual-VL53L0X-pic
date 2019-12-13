/* Defines bit position for used pins */
#define XSHUT_L 15
#define XSHUT_R 12
#define INT_L 14
#define INT_R 13
#define INT_OUT 11

#define DIP1pin RB5
#define DIP2pin RB6
#define DIP3pin RB7

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
    bool DIPS[3];
    DIPS[0] = PORTB.DIP1pin; /* Calibration Mode */
    DIPS[1] = PORTB.DIP2pin; /* LED Mode or Calibration Data Management */
    DIPS[3] = PORTB.DIP3pin; /* Slave I2C address */
    VL53L0X_Dev_t r, l;
    VL53L0X_DEV RightSensor = &r, LeftSensor = &l;
    VL53L0X_Error StatusL = VL53L0X_ERROR_NONE, StatusR = VL53L0X_ERROR_NONE;
    
    bool calMode, dataReset, ledMode;
    uint8_t slaveI2CAddress;
    
    calMode = DIPS[0];
    dataReset = calMode && DIPS[1];
    ledMode = calMode || DIPS[1];
    if(DIPS[2] == false){
        slaveI2CAddress = 0x42;
    }else{
        slaveI2CAddress = 0x44;
    }
    
    RightSensor->I2cDevAddr = 0x52;
    
    /*Configure shutdown pins as outputs*/
    TRISB &= !((1 << XSHUT_L) + (1 << XSHUT_R));
    
    /*Wake up right sensor*/
    LATB = 1 << XSHUT_R;
    __delay_ms(2);
    
    /*VL53L0X_DataInit(VL53L0X_DEV Dev)*/
    LeftSensor->I2cDevAddr = 0x52;
    VL53L0X_SetDeviceAddress(RightSensor, 0x54); /*TODO what if the address was already set ?*/
    LeftSensor->I2cDevAddr = 0x54;

    /*Turns on left sensor*/
    LATB |= 1 << XSHUT_L;
    __delay_ms(2);
    
    StatusR = VL53L0X_DataInit(RightSensor);
    StatusL = VL53L0X_DataInit(LeftSensor);
    
    /*Mainly used for debug, useless once in production ?*/
    if(!(StatusR == VL53L0X_ERROR_NONE && StatusL == VL53L0X_ERROR_NONE)){
        /*Do something ?*/
    }
    
    StatusR = VL53L0X_StaticInit(RightSensor);
    StatusL = VL53L0X_StaticInit(LeftSensor);
    
    /*Mainly used for debug, useless once in production ?*/
    if(!(StatusR == VL53L0X_ERROR_NONE && StatusL == VL53L0X_ERROR_NONE)){
        /*Do something ?*/
    }
    /*
     * ReferenceSPADManagement
     * To be performed once after manufacturing especially if cover glass is
     * used. The resulting data has to be stored on the host.
     */
    VL53L0X_PerformReferenceSPADManagement(RightSensor, ..., ...);
    VL53L0X_PerformRefCaliration();
    
    /*White target*/
    VL53L0X_PerformOffsetCalibration();
    
    /*Set grey target*/
    VL53L0X_PerformXTalkCalibration();
    
    
    /*
     * DataInit
     * StaticInit
     * PerformRefSpadManagement
     */
    
    /*
     * .X_Shut_R high V
     * (wait) V
     * .Change adress of right sensor V
     * .X_Shut_L high V
     * (wait) V
     * Init
     * Check if we have to do  SPAD calibration otherwise loads it
     * Temp calibration
     * Check if we need to do offset or xTalk calibration otherwise load them
     * 
    */

    while(1)
    {
        if(i2c_slave_ready == true){
            /*Manage slave I2C*/
            i2c_slave_ready = false;
        }
    }
}
