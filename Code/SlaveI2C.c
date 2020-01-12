/* 
 * File:   SlaveI2C.cpp
 * Author: Morgan Diepart
 * 
 * Created on 12 janvier 2020, 00:50
 */

#include "SlaveI2C.h"
#include <xc.h>
#include <stdbool.h>
typedef enum{
    IDLE,
    WAITING_REGISTER,
    SENDING_DATA,
    RECEIVING_DATA
} State;

bool autoIncrement;
State state = IDLE;

void I2CSlaveInit(uint8_t address){
    /* Sets address */
    I2C2ADDbits = (uint16_t)address;
    
    /* Enables module */
    I2C2CONLbits.I2CEN = 1;
}

void I2CSlaveExec(){
    
}