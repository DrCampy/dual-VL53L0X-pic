#include "data_storage.h"
#include "config.h"
#include "DEEE/Include/DEE Emulation 16-bit/DEE Emulation 16-bit.h"
#include "Api/inc/platform/vl53l0x_types.h"
/****************/
/*     SPAD     */
/****************/
/* Write SPAD calibration data of right sensor to emulated EEPROM */
void writeRightSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD){
    uint16_t refSPADCountL = *refSPADCount;
    uint16_t refSPADCountH = *refSPADCount >> 16;
    DataEEWrite(refSPADCountL, ADD_RSC_R_L);
    DataEEWrite(refSPADCountH, ADD_RSC_R_H);
    DataEEWrite(*isApertureSPAD, ADD_IAS_R);
}

/* Read SPAD calibration data of right sensor from emulated EEPROM */
void readRightSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD){
    uint16_t refSPADCountL = DataEERead(ADD_RSC_R_L);
    uint16_t refSPADCountH = DataEERead(ADD_RSC_R_H);
    *refSPADCount = ((uint32_t)refSPADCountH << 16) + refSPADCountL;
    *isApertureSPAD = DataEERead(ADD_IAS_R);  
}

/* Write SPAD calibration data of left sensor to emulated EEPROM */
void writeLeftSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD){
    uint16_t refSPADCountL = *refSPADCount;
    uint16_t refSPADCountH = *refSPADCount >> 16;
    DataEEWrite(refSPADCountL, ADD_RSC_L_L);
    DataEEWrite(refSPADCountH, ADD_RSC_L_H);
    DataEEWrite(*isApertureSPAD, ADD_IAS_L);
}

/* Read SPAD calibration data of left sensor from emulated EEPROM */
void readLeftSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD){
    uint16_t refSPADCountL = DataEERead(ADD_RSC_L_L);
    uint16_t refSPADCountH = DataEERead(ADD_RSC_L_H);
    *refSPADCount = ((uint32_t)refSPADCountH << 16) + refSPADCountL;
    *isApertureSPAD = DataEERead(ADD_IAS_L); 
}

/****************/
/*    OFFSET    */
/****************/
/* Write offset calibration data of right sensor to emulated EEPROM */
void writeRightOffsetCalData(int32_t *offsetMicroMeter){
    uint16_t offsetMicroMeterL = *offsetMicroMeter;
    uint16_t offsetMicroMeterH = *offsetMicroMeter >> 16;
    DataEEWrite(offsetMicroMeterL, ADD_OMM_R_L);
    DataEEWrite(offsetMicroMeterH, ADD_OMM_R_H);
}

/* Read offset calibration data of right sensor from emulated EEPROM */
void readRightOffsetCalData(int32_t *offsetMicroMeter){
    uint16_t offsetMicroMeterL = DataEERead(ADD_OMM_R_L);
    uint16_t offsetMicroMeterH = DataEERead(ADD_OMM_R_H);
    *offsetMicroMeter = ((uint32_t)offsetMicroMeterH << 16) + offsetMicroMeterL;
}

/* Write offset calibration data of left sensor to emulated EEPROM */
void writeLeftOffsetCalData(int32_t *offsetMicroMeter){
    uint16_t offsetMicroMeterL = *offsetMicroMeter;
    uint16_t offsetMicroMeterH = *offsetMicroMeter >> 16;
    DataEEWrite(offsetMicroMeterL, ADD_OMM_L_L);
    DataEEWrite(offsetMicroMeterH, ADD_OMM_L_H);
}

/* Read offset calibration data of left sensor from emulated EEPROM */
void readLeftOffsetCalData(int32_t *offsetMicroMeter){
    uint16_t offsetMicroMeterL = DataEERead(ADD_OMM_L_L);
    uint16_t offsetMicroMeterH = DataEERead(ADD_OMM_L_H);
    *offsetMicroMeter = ((uint32_t)offsetMicroMeterH << 16) + offsetMicroMeterL;
}

/****************/
/*     XTALK    */
/****************/
/* Write crosstalk calibration data of right sensor to emulated EEPROM */
void writeRightXTalkCalData(FixPoint1616_t *xTalkCompensationRateMegaCps){
    uint16_t xTalkCompensationRateMegaCpsL = *xTalkCompensationRateMegaCps;
    uint16_t xTalkCompensationRateMegaCpsH = *xTalkCompensationRateMegaCps >> 16;
    DataEEWrite(xTalkCompensationRateMegaCpsL, ADD_XTCRMC_R_L);
    DataEEWrite(xTalkCompensationRateMegaCpsH, ADD_XTCRMC_R_H);
}

/* Read crosstalk calibration data of right sensor from emulated EEPROM */
void readRightXTalkCalData(FixPoint1616_t *xTalkCompensationRateMegaCps){
    uint16_t xTalkCompensationRateMegaCpsL = DataEERead(ADD_XTCRMC_R_L);
    uint16_t xTalkCompensationRateMegaCpsH = DataEERead(ADD_XTCRMC_R_H);
    *xTalkCompensationRateMegaCps = 
            ((FixPoint1616_t)xTalkCompensationRateMegaCpsH << 16)
            + xTalkCompensationRateMegaCpsL;
}

/* Write crosstalk calibration data of left sensor to emulated EEPROM */
void writeLeftXTalkCalData(FixPoint1616_t *xTalkCompensationRateMegaCps){
    uint16_t xTalkCompensationRateMegaCpsL = *xTalkCompensationRateMegaCps;
    uint16_t xTalkCompensationRateMegaCpsH = *xTalkCompensationRateMegaCps >> 16;
    DataEEWrite(xTalkCompensationRateMegaCpsL, ADD_XTCRMC_L_L);
    DataEEWrite(xTalkCompensationRateMegaCpsH, ADD_XTCRMC_L_H);
}

/* Read crosstalk calibration data of left sensor from emulated EEPROM */
void readLeftXTalkCalData(FixPoint1616_t *xTalkCompensationRateMegaCps){
    uint16_t xTalkCompensationRateMegaCpsL = DataEERead(ADD_XTCRMC_L_L);
    uint16_t xTalkCompensationRateMegaCpsH = DataEERead(ADD_XTCRMC_L_H);
    *xTalkCompensationRateMegaCps = 
            ((FixPoint1616_t)xTalkCompensationRateMegaCpsH << 16)
            + xTalkCompensationRateMegaCpsL;
}

/****************/
/*    ADDRESS   */
/****************/
/* Write I2C slave address to emulated EEPROM */
void writeI2CSlaveAddress(uint8_t *address){
    DataEEWrite(*address, ADD_I2C_ADD);
}

/* Read I2C slave address from emulated EEPROM */
void readI2CSlaveAddress(uint8_t *address){
    uint16_t address_16 = DataEERead(ADD_I2C_ADD);
    *address = (uint8_t) address_16;
}