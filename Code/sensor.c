#include <xc.h>
#include <stdbool.h>
#include "Api/inc/core/vl53l0x_api.h"
#include "config.h"
#include "sensor.h"

VL53L0X_DEV RightSensor, LeftSensor; /*Sensors handles*/

/* Config Updated flag */
bool CONFIG_UPDATEDflag = 1;

/*Config low register*/
bool L_ENflag = 1;
bool R_ENflag = 1;
bool XTALKflag = 0;
bool AUTO_INCflag = 0;
bool CONT_MODEflag = 0;
bool CONVflag = 0;
bool CONV_FINISHEDflag = 0;

/*Config high register*/
uint8_t INT_MODEflags = 00; /* 2 bits */
uint8_t DURATIONval; /* 6 bits */

uint8_t I2C_ADDRESSvalue;

/* Distances */
uint8_t leftDist = 0;
uint8_t rightDist = 0;
uint8_t avgDist = 0;
uint8_t *minDist;
uint8_t *maxDist;

void powerOffRightSensor(){
    LATB &= !(1 << XSHUT_R);
}

void powerOnRightSensor(){
    LATB |= 1 << XSHUT_R;
}

void powerOffLeftSensor(){
    LATB &= !(1 << XSHUT_L);
}

void powerOnLeftSensor(){
    LATB |= 1 << XSHUT_L;
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
 * 20ms + DURATION<0:5>ms
 * Ranges from 20ms to 84ms
 * 
 */
void setConfigL(uint8_t config_l){
    // L_EN flag
    L_ENflag = (config_l &= L_EN) != 0;
    
    //R_EN flag
    R_ENflag = (config_l &= R_EN) != 0;
    
    //XTALK Flag
    XTALKflag = (config_l &= XTALK) != 0;
    
    //AUTO_INT flag
    AUTO_INCflag = (config_l &= AUTO_INC) != 0;
    
    //CONT_MODE flag
    CONT_MODEflag = (config_l &= CONT_MODE) != 0;
    
    //CONV_FLAG
    CONVflag = (config_l &= CONV) != 0;
    
    CONFIG_UPDATEDflag = true;
}

void setConfigH(uint8_t config_h){
    //INT_MODE flags
    INT_MODEflags = (config_h & 0b11000000) >> 6;
    
    //DURATION value
    DURATIONval = (config_h & 0b00111111) >> 6;

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
    uint8_t configL = 0;
    if(L_ENflag){
        configL += L_EN;
    }
    if(R_ENflag){
        configL += R_EN;
    }
    if(XTALKflag){
        configL += XTALK;
    }
    if(AUTO_INCflag){
        configL += AUTO_INC;
    }
    if(CONT_MODEflag){
        configL += CONT_MODE;
    }
    if(CONVflag){
        configL += CONV;
    }
    if(CONV_FINISHEDflag){
        configL += CONV_FINISHED;
    }
    
    return configL;
}

uint8_t getConfigH(){
    uint8_t configH = 0;
    
    configH += (INT_MODEflags & 0b00000011) << 6;
    configH += (DURATIONval & 0b00111111);
    
    return configH;
}

void updateConfig(){
    if(CONFIG_UPDATEDflag == false){
        return; //COnfig was not updated since last execution
    }
    // L_EN flag
    if(L_ENflag){
        powerOnLeftSensor();
    }else{
        powerOffLeftSensor();
    }
    
    //R_EN flag
    if(R_ENflag){
        powerOnRightSensor();
    }else{
        powerOffRightSensor();
    }
    
    //XTALK Flag
    if(XTALKflag){
        enableXTalk();
    }else{
        disableXTalk();
    }
    
    //CONT_MODE flag
    if(CONT_MODEflag){

    }else{

    }
    
    //CONV_FLAG
    if(CONVflag){

    }else{

    }

    //DURATION
    uint32_t totDuration = 1000*(20+DURATIONval*3);
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(RightSensor, totDuration);
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(LeftSensor, totDuration);
}

/*
 * Raises the interrupt signal
 */
void raiseInt(){
    //RB11
    LATBbits.LATB11 = 1;
}

/*
 * Reset the interrupt signal
 */
void resetInt(){
    LATBbits.LATB11 = 0;
}