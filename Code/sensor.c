#include <xc.h>
#include <stdbool.h>
#include "Api/inc/core/vl53l0x_api.h"
#include "config.h"
#include "sensor.h"

extern VL53L0X_DEV RightSensor, LeftSensor; /*Sensors handles*/


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
 * CONV_FINISHED: Set to 1 once mea
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
void applyConfigL(uint8_t config_l){
    // L_EN flag
    if(config_l &= L_EN != 0){
        powerOnLeftSensor();
    }else{
        powerOffLeftSensor();
    }
    
    //R_EN flag
    if(config_l &= R_EN != 0){
        powerOnRightSensor();
    }else{
        powerOffRightSensor();
    }
    
    //XTALK Flad
    if(config_l &= XTALK != 0){
        enableXTalk();
    }else{
        disableXTalk();
    }
    
    //CONT_MODE flag
    
    //CONV_FLAG
    
}

void applyConfigH(uint8_t config_l){
    
}

void enableXTalk(){
    VL53L0X_SetXTalkCompensationEnable(RightSensor, 1);
    VL53L0X_SetXTalkCompensationEnable(LeftSensor, 1);
}

void disableXTalk(){
    VL53L0X_SetXTalkCompensationEnable(RightSensor, 0);
    VL53L0X_SetXTalkCompensationEnable(LeftSensor, 0);    
}