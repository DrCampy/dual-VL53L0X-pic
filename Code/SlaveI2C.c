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

int i2c_registers[11];
extern volatile bool i2c_slave_ready;
bool autoIncrement;
State state = IDLE;

void I2CSlaveInit(uint8_t address){
    /* Sets address */
    I2C2ADDbits = (uint16_t)address;
    
    /* Enables module */
    I2C2CONLbits.I2CEN = 1;
}

void I2CSlaveExec(){
    bool isAddress = !I2C2STATbits[5];
    bool isRx = I2C2STATbits[2];
    
    //Check for collision or so
    
    if(isAddress && isRx){
        //Do nothing. Next interrupt will give us some registers address
        state = WAITING_REGISTER;
    }else if(isAddress && !isRx){
        //Received our address. We have to send requested data.
        state = SENDING_DATA;
        
    }else if(!isAddress && isRx){
        //Receiving data. First onr is register, next one is data.
        //If auto increment is ON, we will first receive a register then datas.
        //If auto increment is OFF we will receive a register then a data then
        //a register again and so on.
        
        
    }else if(!isAddress && !isRx){
        
    }
    
    i2c_slave_ready = false;
    
}