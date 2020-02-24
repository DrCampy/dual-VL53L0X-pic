/* 
 * File:   SlaveI2C.h
 * Author: Morgan Diepart
 *
 * Created on 12 janvier 2020, 00:50
 */

#ifndef SLAVEI2C_H
#define	SLAVEI2C_H

/*
 * To add a new register :
 * - Update I2CSlaveIsConfigRegister
 * - Update I2CSlaveIsRegisterValid
 * - Update I2CSlaveNextRegister
 * - Update I2CSlaveGetRegister
 * - Update I2CSlaveSetRegister (if writable register)
 */

//I2C registers
//Config 
//RW registers
#define     I2C_CONFIG_L            0x00
#define     I2C_CONFIG_H            0x01
#define     I2C_ADDRESS             0x02

//Distances
//RO registers
#define     I2C_RIGHT               0x10
#define     I2C_LEFT                0x11
#define     I2C_MIN                 0x12
#define     I2C_MAX                 0x13
#define     I2C_AVG                 0x14

void I2CSlaveInit(uint8_t address);
void I2CSlaveExec();

#endif	/* SLAVEI2C_H */
