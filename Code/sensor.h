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

#define L_EN            0b10000000
#define R_EN            0b01000000
#define XTALK           0b00100000
#define CONT_MODE       0b00001000
#define CONV            0b00000100
#define CONV_FINISHED   0b00000010
    
    void powerOffRightSensor();
    void powerOnRightSensor();
    void powerOffLeftSensor();
    void powerOnLeftSensor();
    
    void applyConfigL(uint8_t config_l);
    void applyConfigH(uint8_t config_l);
    
    void disableXTalk();
    void enableXTalk();

#ifdef	__cplusplus
}
#endif

#endif	/* SENSOR_H */

