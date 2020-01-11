/* 
 * File:   config.h
 * Author: Morgan Diepart
 *
 * Created on 27 décembre 2019, 01:34
 */

#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

/*Main slave I2C address*/
#define PRIM_SLAVE_I2C_ADDR 0x42
    
/*Secondary slave I2C address*/
#define SEC_SLAVE_I2C_ADDR 0x44
    
/*Offset calibration distance in mm*/    
#define OFFSET_CAL_DISTANCE 100
    
/*XTalk calibration distance in mm*/
#define XTALK_CAL_DISTANCE 500
    
/*Define needed for the API to use I2C in 2.8V*/
#define USE_I2C_2V8
    
/* Defines bit position for used pins */
#define XSHUT_L 15
#define XSHUT_R 12
#define INT_L 14
#define INT_R 13
#define INT_OUT 11

#define DIP1pin RB5
#define DIP2pin RB6
#define DIP3pin RB7
    
/* EEPROM Data addresses */
#define ADD_RSC_R_L     0x00    /*refSPADCount      right low is    0x00 */
#define ADD_RSC_R_H     0x01    /*refSPADCount      right high is   0x01 */
#define ADD_IAS_R       0x02    /*isApertureSPAD    right is        0x02 */
#define ADD_RSC_L_L     0x03    /*refSPADCount      left low is     0x03 */
#define ADD_RSC_L_H     0x04    /*refSPADCount      left high is    0x04 */
#define ADD_IAS_L       0x05    /*isApertureSPAD    left is         0x05 */
#define ADD_OMM_R_L     0x06    /*offsetMicroMeter  right low is    0x06 */
#define ADD_OMM_R_H     0x07    /*offsetMicroMeter  right high is   0x07 */
#define ADD_OMM_L_L     0x08    /*offsetMicroMeter  left low is     0x08 */
#define ADD_OMM_L_H     0x09    /*offsetMicroMeter  left high is    0x09 */
#define ADD_XTCRMC_R_L  0x0A    /*xTalkCompensationRateMegaCps  right low is    0x0A */
#define ADD_XTCRMC_R_H  0x0B    /*xTalkCompensationRateMegaCps  right high is   0x0B */
#define ADD_XTCRMC_L_L  0x0C    /*xTalkCompensationRateMegaCps  left low is     0x0C */
#define ADD_XTCRMC_L_H  0x0D    /*xTalkCompensationRateMegaCps  left high is    0x0D */
    
#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

