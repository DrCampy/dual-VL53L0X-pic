#include <xc.h>
#include <stdbool.h>

#include "config.h"
#include "sensor.h"

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
 * L_EN    R_EN    XTALK    AUTO_INC    cont_mode    l_conv    r_conv    UNUSED
 * 
 * L_EN: Left sensor enabled (true/false)
 * R_EN: Right sensor enabled (true/false)

 * XTALK: Crosstalk compensation enabled (true/false)
 * Auto_INC: auto incrementation of I2C registers enabled (true/false)
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
    
    //XTALK Flag
    
    //CONT_MODE flag
    
    //CONV_FLAG
    
}

void applyConfigH(uint8_t config_l){
    
}