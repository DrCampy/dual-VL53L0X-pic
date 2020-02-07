/* 
 * File:   SlaveI2C.cpp
 * Author: Morgan Diepart
 * 
 * Created on 12 janvier 2020, 00:50
 */

#include <xc.h>
#include <stdbool.h>
#include "SlaveI2C.h"

void I2CSlaveSetAddress(uint8_t address);

typedef enum{
    IDLE,
    WAITING_REGISTER,
    SENDING_DATA,
    RECEIVING_DATA
} State;

int i2cRegisters[I2C_NB_REGISTERS];
extern volatile bool i2c_slave_ready;
bool autoIncrement;
State state = IDLE;

void I2CSlaveInit(uint8_t address){
    /* Sets address */
    I2CSlaveSetAddress(address);
    
    /*Enables the I2C module to hols SCL low upon receiving a byte of either
     address (if matching) or data until SCLREL is set to 1. Also allows slave
     to automatically acknowledge it's address*/
    I2C2CONHbits.AHEN = 1;
    I2C2CONHbits.DHEN = 1;
    
    /* Enables module */
    I2C2CONLbits.I2CEN = 1;
}

//If master sends NACK after we sent data, the module will not generate interrupt.
//A stop condition is expected afterwards.

void I2CSlaveExec(){
    bool isAddress = !I2C2STATbits.D_NOT_A;
    bool isRx = I2C2STATbits.R_NOT_W;
    uint8_t workingRegister = 0x00;
    
    //Check for collision or so
    
    if(isAddress && isRx){
        //Do nothing. Next interrupt will give us some registers address
        state = WAITING_REGISTER; //update internal state
        
        //Acknowledge
        I2C2CONLbits.ACKDT = 0; //ACK
        I2C2CONLbits.SCLREL = 1; //Release SCL line
        
    }else if(isAddress && !isRx){
        //Received our address. We have to send requested data.
        state = SENDING_DATA; //update internal state
        I2C2TRN = i2cRegisters[workingRegister]; //Transmits the requested value
        
    }else if(!isAddress && isRx){
        //Receiving data. First one is register, next one is data.
        //If auto increment is ON, we will first receive a register then datas.
        //If auto increment is OFF we will receive a register then a data then
        //a register again and so on.
        
        //While receiving data, STREN should be 0 to automatically
        //wait and hold SCL. (Check SCLREL ?)
        if(state == WAITING_REGISTER){
            uint8_t temp = I2C2RCV;
            if(temp < 0 || temp > I2C_LAST_ADD){
                //Invalid register, NACK
                I2C2CONLbits.ACKDT = 1; //NACK
                I2C2CONLbits.SCLREL = 1; //Release SCL
            }else{
                workingRegister = I2C2RCV;
                state = RECEIVING_DATA;
            }
        }else{ //Receiving data
            //Check if we are in a writable register
            if(workingRegister <= I2C_ADDRESS){
                i2cRegisters[workingRegister] = I2C2RCV;
                if(workingRegister == I2C_CONFIG_L){
                    //Todo update according to registers
                }else if(workingRegister == I2C_CONFIG_H){
                    //Todo implements but unused for now...
                }else if(workingRegister == I2C_ADDRESS){
                    I2CSlaveSetAddress(i2cRegisters[I2C_ADDRESS]);
                }
                I2C2CONLbits.ACKDT = 0; //Ack
                
                //Auto increment
                if(autoIncrement == true){
                    workingRegister = (workingRegister+1)%I2C_ADDRESS;
                }
            }else{
                //Trying to write to read only register.
                I2C2CONLbits.ACKDT = 1; //Nack
            }
            I2C2CONLbits.SCLREL = 1; //Release scl
        }
        
        
    }else if(!isAddress && !isRx){
        //We just sent data. If autoIncrement enabled we should send next bit
        if(autoIncrement == true){
            //Check registers
            
            //If we are in distance registers =, automatically cycle between them
            if(workingRegister >= I2C_RIGHT_L &&
                    workingRegister <= I2C_MAX_L){
                workingRegister++;
                if(workingRegister > I2C_MAX_H){
                    workingRegister = I2C_RIGHT_L;
                }
                I2C2TRN = i2cRegisters[workingRegister];
            }
        }
        //TODO
                    
    }
    
    i2c_slave_ready = false;
    
}

void I2CSlaveSetAddress(uint8_t address){
    //Write address to register
    //Write address to I2C module
    i2cRegisters[I2C_ADDRESS] = address;
    I2C2ADD = address;
}

/*
 * Returns true if the register reg is a config register.
 * Otherwise it is a data register which belongs in another "cycle"
 * and is read-only.
 */
bool I2CSlaveIsConfigRegister(uint8_t reg){
    if(reg >= I2C_CONFIG_L && reg <= I2C_ADDRESS){
        return true;
    }else{
        return false;
    }
    
}