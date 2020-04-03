/* 
 * File:   sensor.h
 * Author: Morgan Diepart
 *
 * Created on 15 f�vrier 2020, 01:21
 */

#ifndef SENSOR_H
#define	SENSOR_H

#ifdef	__cplusplus
extern "C" {
#endif
/*
#define L_EN            0b10000000
#define R_EN            0b01000000
#define XTALK           0b00100000
#define AUTO_INC        0b00010000
#define CONT_MODE       0b00001000
#define CONV            0b00000100
#define CONV_FINISHED   0b00000010
*/
#define INT_OFF         ((uint8_t)0)
#define INT_L_AND_R     ((uint8_t)1)
#define INT_L_OR_R      ((uint8_t)2)
   
#include "Api/inc/core/vl53l0x_api.h"
    
    /*Sensors handles*/
    extern VL53L0X_DEV RightSensor, LeftSensor; 

    /* Config Updated flag */
    extern bool CONFIG_UPDATEDflag;
    
    /* Config low register */   
    typedef struct tagCONFIG_LBITS {
        union {
            uint8_t value;
            struct {
                uint8_t :1;
                uint8_t CONV_FINISHED:1;
                uint8_t CONV:1;
                uint8_t CONT_MODE:1;
                uint8_t AUTO_INC:1;
                uint8_t XTALK:1;
                uint8_t R_EN:1;
                uint8_t L_EN:1;
            };
        };
    } CONFIG_LBITS;   
    extern CONFIG_LBITS CONFIG_Lbits;
    
    /* Config high register */
    typedef struct tagCONFIG_HBITS {
        union {
            uint8_t value;
            struct {
                uint8_t DURATION:6;
                uint8_t INT_MODE:2;
            };
        };
    } CONFIG_HBITS;   
    extern CONFIG_HBITS CONFIG_Hbits;
    
    /* Address register */
    extern uint8_t I2C_ADDRESSvalue;
    
    /* Distances */
    extern uint8_t leftDist;
    extern uint8_t rightDist;
    extern uint8_t avgDist;
    extern uint8_t minDist;
    extern uint8_t maxDist;
    
    void setConfigL(uint8_t config_l);
    void setConfigH(uint8_t config_l);
    
    void disableXTalk();
    void enableXTalk();
    
    uint8_t getConfigL();
    uint8_t getConfigH();

    void updateConfig();
    
    void raiseInt();
    void resetInt();
    
    void initVL53L0X();
    
    void measurementFinished();
    
#ifdef	__cplusplus
}
#endif

#endif	/* SENSOR_H */

