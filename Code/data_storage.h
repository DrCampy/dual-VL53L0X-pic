/* 
 * File:   data_storage.h
 * Author: Morgan Diepart
 *
 * Created on 14 février 2020, 22:11
 */

#ifndef DATA_STORAGE_H
#define	DATA_STORAGE_H

#include <xc.h>
#include <libpic30.h>
#include <stdint.h>        /* Includes uint16_t definition                    */
#include "config.h"
#include "Api/inc/platform/vl53l0x_types.h"

#ifdef	__cplusplus
extern "C" {
#endif

void writeRightSPADCalData(uint32_t* refSPADCount, uint8_t* isApertureSPAD);
void readRightSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD);
void writeLeftSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD);
void readLeftSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD);
void writeRightOffsetCalData(int32_t *offsetMicroMeter);
void readRightOffsetCalData(int32_t *offsetMicroMeter);
void writeLeftOffsetCalData(int32_t *offsetMicroMeter);
void readLeftOffsetCalData(int32_t *offsetMicroMeter);
void writeRightXTalkCalData(FixPoint1616_t *xTalkCompensationRateMegaCps);
void readRightXTalkCalData(FixPoint1616_t *xTalkCompensationRateMegaCps);
void writeLeftXTalkCalData(FixPoint1616_t *xTalkCompensationRateMegaCps);
void readLeftXTalkCalData(FixPoint1616_t *xTalkCompensationRateMegaCps);
void writeI2CSlaveAddress(uint8_t *address);
void readI2CSlaveAddress(uint8_t *address);

#ifdef	__cplusplus
}
#endif

#endif	/* DATA_STORAGE_H */

