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

/* Defines bit position for used pins */
#define XSHUT_L 15
#define XSHUT_R 12
#define INT_L 14
#define INT_R 13
#define INT_OUT 11

#define DIP1pin RB5
#define DIP2pin RB6
#define DIP3pin RB7

#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

