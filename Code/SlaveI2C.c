/* 
 * File:   SlaveI2C.cpp
 * Author: Morgan Diepart
 * 
 * Created on 12 janvier 2020, 00:50
 */

#include <xc.h>
#include <stdbool.h>
#include "SlaveI2C.h"

void    I2CSlaveSetAddress          (uint8_t address);
bool    I2CSlaveIsConfigRegister    (uint8_t reg);
bool    I2CSlaveIsRegisterWritable  (uint8_t reg);
uint8_t I2CSlaveGetByte             ();
void    I2CSlaveSendByte            (uint8_t data);
void    I2CSlaveAck                 ();
void    I2CSlaveNack                ();
int8_t  I2CSlaveIsLowRegister       (uint8_t reg);
uint8_t I2CSlaveNextRegister        (bool reg);
bool    I2CSlaveIsRegisterValid     (uint8_t reg);
void    I2CSlaveApplyNewConfig      (uint8_t config_l, uint8_t config_h);

typedef enum{
    IDLE,
    WAITING_REGISTER,
    SENDING_DATA,
    RECEIVING_DATA
} State;

int     i2cRegisters[I2C_NB_REGISTERS];
bool    autoIncrement;
State   state = IDLE;
extern volatile bool i2c_slave_ready;

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
            I2CSlaveSendByte(i2cRegisters[workingRegister]); //Transmits the requested value
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
                if(workingRegister == I2C_CONFIG_L){
                    //Todo update according to registers
                }else if(workingRegister == I2C_CONFIG_H){
                    //TODO
                }else if(workingRegister == I2C_ADDRESS){
                    I2CSlaveSetAddress(i2cRegisters[I2C_ADDRESS]);
                    i2cRegisters[workingRegister] = data;
                }/*else we are in a read only register. Acknowledges but does
                  * nothing.
                  */
                
                /*
                 * If autoIncrement enabled we will still be receiving data.
                 * If not we will be receiving the address of the next register
                 * to work with.
                 */
                if(autoIncrement == true){
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
            I2CSlaveSendByte(i2cRegisters[workingRegister]);
            
            //Selects next register
            workingRegister = I2CSlaveNextRegister(workingRegister); 
        }
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
            if(I2CSlaveIsLowRegister(reg) == 1){
                return reg+1;
            }else if(I2CSlaveIsLowRegister(reg) == 0){
                return reg-1;
            }
        }else{
            reg++;
            if(reg > I2C_AVG_H){
                reg = I2C_RIGHT_L;
            }
            return reg;
        }
    }
    return 0x00;
}
    
/*
 * Checks if the register is a LOW part of an existing register.
 * 
 * Returns 1 if it is.
 * Returns 0 if it is not.
 * Returns -1 if the register is only one byte.
 */
int8_t I2CSlaveIsLowRegister(uint8_t reg){
    if(reg == I2C_CONFIG_L || reg == I2C_RIGHT_L ||
            reg == I2C_LEFT_L || reg == I2C_MIN_L ||
            reg == I2C_MAX_L || reg == I2C_AVG_L){
        return 1;
    }else if(reg == I2C_ADDRESS){
        return -1;
    }else{
        return 0;
    }
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
void I2CSlaveApplyNewConfig(uint8_t config_l, uint8_t config_h){
    
}

bool I2CSlaveIsRegisterValid(uint8_t reg){
    if(reg <= I2C_LAST_ADD){
        return true;
    }
    return false;
}