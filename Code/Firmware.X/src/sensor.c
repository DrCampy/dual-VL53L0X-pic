#include <xc.h>
#include <stdbool.h>
#include "Api/inc/core/vl53l0x_api.h"
#include "config.h"
#include "sensor.h"

VL53L0X_Dev_t __RightSensor_, __LeftSensor_;
VL53L0X_DEV RightSensor, LeftSensor; /*Sensors handles*/

/* Config Updated flag */
bool CONFIG_UPDATEDflag = 1; //Run updateConfig() at least once

/*Config low register*/
CONFIG_LBITS CONFIG_Lbits = {{0b11000000}}; //N.B. Old GCC bug forces use of {{.}}
CONFIG_LBITS prevCONFIG_Lbits = {{0b11000000}};

/*Config high register*/
CONFIG_HBITS CONFIG_Hbits = {{4}};
CONFIG_HBITS prevCONFIG_Hbits = {{4}};

uint8_t I2C_ADDRESSvalue    = 0x42;

/* Distances */
uint8_t leftDist    = 0;
uint8_t rightDist   = 0;
uint8_t avgDist     = 0;
uint8_t minDist;
uint8_t maxDist;

void powerOffRightSensor(){
    LATBbits.LATB12 = 0;
}

void powerOnRightSensor(){
    LATBbits.LATB12 = 1;
}

void powerOffLeftSensor(){
    LATBbits.LATB15 = 0;
}

void powerOnLeftSensor(){
    LATBbits.LATB15 = 1;
}

/*
 * Apply the new config to the device.
 * Structure of config_l:
 * L_EN    R_EN    XTALK    AUTO_INC    CONT_MODE    CONV    FINISHED    UNUSED
 * 
 * L_EN: Left sensor enabled (true/false)
 * R_EN: Right sensor enabled (true/false)

 * XTALK: Crosstalk compensation enabled (true/false)
 * 
 * AUTO_INC: auto incrementation of I2C registers enabled (true/false)
 * 
 * CONT_MODE: If 1 the sensor will take measurements continuously once CONV is 
 *              set to 1.
 *            If 0 the sensor will perform one measurement with each enabled
 *              sensor once CONV is set to 1. CONV is set back to 0 after measurements.
 * 
 * CONV: Start conversion. In case CONT_MODE = 1, starts measurements until
 *        CONV is set back to 0 again.
 * 
 * CONV_FINISHED: Set to 1 once measurement over
 * 
 * Structure of config_h:
 * INT_MODE    DURATION<0:5>
 * 
 * INT_MODE<0:1>:
 *      11: reserved
 *      10: Raise interrupt at each sensor update
 *      01: Raise interrupt once both sensors have been updated
 *      00: Interrupts disabled
 * 
 * DURATION<0:5>:
 * Determines the max duration allocated to each measurement.
 * Final duration is:
 * 20ms + 3*DURATION<0:5>ms
 * Ranges from 20ms to 84ms
 * 
 */
void setConfigL(uint8_t config_l){
    CONFIG_Lbits.value = (config_l & 0b11111100); ;
    
    CONFIG_UPDATEDflag = true;
}

void setConfigH(uint8_t config_h){
    CONFIG_Hbits.value = config_h;
    
    CONFIG_UPDATEDflag = true;
}

void enableXTalk(){
    VL53L0X_SetXTalkCompensationEnable(RightSensor, 1);
    VL53L0X_SetXTalkCompensationEnable(LeftSensor, 1);
}

void disableXTalk(){
    VL53L0X_SetXTalkCompensationEnable(RightSensor, 0);
    VL53L0X_SetXTalkCompensationEnable(LeftSensor, 0);    
}

uint8_t getConfigL(){
    return CONFIG_Lbits.value;
}

uint8_t getConfigH(){
    return CONFIG_Hbits.value;
}

void updateConfig(){ //static variables for previous states
    if(CONFIG_UPDATEDflag == false){
        return; //COnfig was not updated since last execution
    }

    // L_EN flag
    if(prevCONFIG_Lbits.L_EN != CONFIG_Lbits.L_EN){
        if(CONFIG_Lbits.L_EN){
            powerOnLeftSensor();
        }else{
            powerOffLeftSensor();
        }    
    }
    
    //R_EN flag
    if(prevCONFIG_Lbits.R_EN != CONFIG_Lbits.R_EN){
        if(CONFIG_Lbits.R_EN){
            powerOnRightSensor();
        }else{
            powerOffRightSensor();
        }
    }
    
    //XTALK Flag
    if(prevCONFIG_Lbits.XTALK != CONFIG_Lbits.XTALK){
        if(CONFIG_Lbits.XTALK){
            enableXTalk();
        }else{
            disableXTalk();
        }    
    }
    
    //INT_MODE, CONV, CONT_MODE are managed automatically by the main code and
    // do not need to be "applied".

    //DURATION
    if(prevCONFIG_Hbits.DURATION != CONFIG_Hbits.DURATION){
        uint32_t totDuration = 1000*(20+CONFIG_Hbits.DURATION*3);
        VL53L0X_SetMeasurementTimingBudgetMicroSeconds(RightSensor, totDuration);
        VL53L0X_SetMeasurementTimingBudgetMicroSeconds(LeftSensor, totDuration);
    }
    
    prevCONFIG_Lbits = CONFIG_Lbits;
    prevCONFIG_Hbits = CONFIG_Hbits;
    
    CONFIG_UPDATEDflag = false;
}

/*
 * Raises the interrupt signal
 */
void raiseInt(){
    LATBbits.LATB11 = 1;
}

/*
 * Reset the interrupt signal
 */
void resetInt(){
    LATBbits.LATB11 = 0;
}

/*
 * Initializes the sensor's structure with default data, including default I2C
 * address as 0x00 (general call)
 */
void initVL53L0X(){
    RightSensor = &__RightSensor_;
    LeftSensor = &__LeftSensor_;
    RightSensor->I2cDevAddr = 0x52;
    LeftSensor->I2cDevAddr = 0x52;
}

void measurementFinished(){
    CONFIG_Lbits.CONV_FINISHED = 1;
    
    //If we are not in continuous mode we stop the measurement
    if(!CONFIG_Lbits.CONT_MODE){ 
        CONFIG_Lbits.CONV = 0;
    }
}