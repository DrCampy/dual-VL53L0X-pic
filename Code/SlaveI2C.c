/* 
 * File:   SlaveI2C.cpp
 * Author: Morgan Diepart
 * 
 * Created on 12 janvier 2020, 00:50
 */

#include <xc.h>
#include <stdbool.h>
#include "SlaveI2C.h"
#include "sensor.h"
#include "data_storage.h"

void    I2CSlaveSetAddress          (uint8_t address);
bool    I2CSlaveIsConfigRegister    (uint8_t reg);
uint8_t I2CSlaveGetByte             ();
void    I2CSlaveSendByte            (uint8_t data);
void    I2CSlaveAck                 ();
void    I2CSlaveNack                ();
int8_t  I2CSlaveIsLowRegister       (uint8_t reg);
uint8_t I2CSlaveNextRegister        (bool reg);
bool    I2CSlaveIsRegisterValid     (uint8_t reg);
uint8_t I2CSlaveGetRegister         (uint8_t address);
void    I2CSlaveSetRegister         (uint8_t address, uint8_t data);
void    I2CSlaveDistReadTrigger     ();


extern bool i2cSecondaryAddress;
//extern bool L_ENflag, R_ENflag, XTALKflag, AUTO_INCflag, CONT_MODEflag, 
//        CONVflag, CONV_FINISHEDflag;

typedef enum{
    IDLE,
    WAITING_REGISTER,
    SENDING_DATA,
    RECEIVING_DATA
} State;

//int     i2cRegisters[I2C_NB_REGISTERS];
bool    autoIncrement;
State   state = IDLE;

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
    
    if(isAddress){//Received our address
        if(isRx){
            //we are receiving (ADD<0> = 0)
            //Next interrupt will give us some registers address
            state = WAITING_REGISTER; //update internal state

            //Acknowledge
            I2CSlaveAck();
        }else{
            //we are transmitting (ADD<0> = 1)
            //We have to send requested data.
            state = SENDING_DATA; //update internal state
            I2CSlaveSendByte(I2CSlaveGetRegister(workingRegister)); //Transmits the requested value
            workingRegister = I2CSlaveNextRegister(workingRegister); //Selects next register.
        }
    }else{//Received / transmitted data
        if(isRx){
            /*Receiving (ADD<0> = 0)
             * First one is register, next one is data
             * If auto increment is ON, we will first receive a register then datas.
             * If auto increment is OFF we will receive a register then a data then
             * a register again and so on.
             */
            if(state == WAITING_REGISTER){
                uint8_t temp = I2CSlaveGetByte();
                if(I2CSlaveIsRegisterValid(temp)){
                    //We received the internal register the master wants to address.
                    workingRegister = temp;
                    state = RECEIVING_DATA;
                    I2CSlaveAck();
                }else{
                    I2CSlaveNack();
                }
            }else{ //Receiving data
                uint8_t data = I2CSlaveGetByte();
                I2CSlaveSetRegister(workingRegister, data);
                
                /*
                 * If autoIncrement enabled we will still be receiving data.
                 * If not we will be receiving the address of the next register
                 * to work with.
                 */
                if(AUTO_INCflag == true){
                    workingRegister = I2CSlaveNextRegister(workingRegister);
                }else{
                    state = WAITING_REGISTER;
                }
                I2CSlaveAck(); //Acknowledges
            }
        }else{
            //Transmitting (ADD<0> = 1)
            //First byte was sent immediately after we have been addressed.
            //We only have to continue the transmission.
            I2CSlaveSendByte(I2CSlaveGetRegister(workingRegister));
            
            //Selects next register
            workingRegister = I2CSlaveNextRegister(workingRegister); 
        }
    }    
}

void I2CSlaveSetAddress(uint8_t address){
    //Write address to I2C module
    I2C2ADD = address;
    
    //If we use the secondary i2c address mode we should not save it to
    // flash memory
    if(i2cSecondaryAddress == false){
        //Write address to flash memory
        writeI2CSlaveAddress(&address);
    }
    
    I2C_ADDRESSvalue = address;
}

/*
 * Returns true if the register reg is a config register.
 * Otherwise it is a data register which belongs in another "cycle"
 * and is read-only.
 */
bool I2CSlaveIsConfigRegister(uint8_t reg){
    switch(reg){
        case I2C_CONFIG_L:
        case I2C_CONFIG_H:
        case I2C_ADDRESS:
            return true;
            break;
        default:
            return false;
            break;
    }  
}

bool I2CSlaveIsRegisterWritable(uint8_t reg){
    return I2CSlaveIsConfigRegister(reg);
}

/*
 * Reads a byte from the I2C bus.
 */
uint8_t I2CSlaveGetByte(){
    return I2C2RCV;
}

/*
 * Sends the content of data on the I2C bus
 */
void I2CSlaveSendByte(uint8_t data){
    I2C2TRN = data;
}

/*
 * Send Ack bit to transmitter
 */
void I2CSlaveAck(){
    I2C2CONLbits.ACKDT = 0; //ACK
    I2C2CONLbits.SCLREL = 1; //Release SCL
}

/*
 * Sends Nack bit to transmitter.
 */
void I2CSlaveNack(){
    I2C2CONLbits.ACKDT = 1; //NACK
    I2C2CONLbits.SCLREL = 1; //Release SCL
}

/*
 * Returns the next register starting from reg.
 * Accounts for Config/Non-config registers and auto incrementation.
 */
uint8_t I2CSlaveNextRegister(bool reg){
    
    // If the register is a config register we loop accross them
    if(I2CSlaveIsConfigRegister(reg)){
        if(autoIncrement == false){
            return reg;
        }else{
            reg++;
            if(reg > I2C_ADDRESS){
                reg=I2C_CONFIG_L;
            }
        }
    }
    //If the register is a distance register we loop accross them
    else{
        if(autoIncrement == false){
            return reg;
        }else{
            reg++;
            if(reg > I2C_AVG){
                reg = I2C_RIGHT;
            }
            return reg;
        }
    }
    return 0x00;
}

/*
 * Returns true if reg is an existing register.
 * Returns false otherwise.
 */
bool I2CSlaveIsRegisterValid(uint8_t reg){
    switch(reg){
        case I2C_CONFIG_L:
        case I2C_CONFIG_H:
        case I2C_ADDRESS:
        case I2C_RIGHT:
        case I2C_LEFT:
        case I2C_MIN:
        case I2C_MAX:
        case I2C_AVG:
            return true;
            break;
        default:
            return false;
            break;
    }
}

void I2CSlaveSetRegister(uint8_t address, uint8_t data){
    switch(address){
        case I2C_CONFIG_L:
            setConfigL(data);
            break;
        case I2C_CONFIG_H:
            setConfigH(data);
            break;
        case I2C_ADDRESS:
            I2CSlaveSetAddress(data);
            break;
        default:
            /*Non-writable register*/
            break;
    }  
}

uint8_t I2CSlaveGetRegister(uint8_t address){
    switch(address){
        case I2C_CONFIG_L:
            return getConfigL();
            break;
        case I2C_CONFIG_H:
            return getConfigH();
            break;
        case I2C_ADDRESS:
            return I2C_ADDRESSvalue;
            break;
        case I2C_RIGHT:
            return rightDist;
            break;
        case I2C_LEFT:
            return leftDist;
            break;
        case I2C_MIN:
            return *minDist;
            break;
        case I2C_MAX:
            return *maxDist;
            break;
        case I2C_AVG:
            return avgDist;
            break;
        default:
            /*Unknown register*/
            break;
    }
    
    return 0;
}