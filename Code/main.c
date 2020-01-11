

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
#include <xc.h>            /* Device header file                              */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include <libpic30.h>
#include <p24FJ64GA702.h>  /* __delay_ms() function                           */
#include "config.h"        /* Configuration definitions                       */
#include "Api/inc/core/vl53l0x_api.h" /*VL53L0X Api                           */
#include "DEEE/Include/DEE Emulation 16-bit/DEE Emulation 16-bit.h"

/******************************************************************************/
/* Custom Functions, enums,...                                                */
/******************************************************************************/
typedef enum {RUN, RST, SPAD, OFFSET, XTALK} Mode;
void blinkStatusLed(uint8_t blinks, uint16_t blinkDuration,\
    uint16_t msBetweenBlinks);

void writeRightSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD);
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

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
volatile bool i2c_slave_ready = false;


/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    
    /* Configure the oscillator for the device */
    ConfigureOscillator();
    
    bool DIPS[3];
    DIPS[0] = PORTBbits.DIP1pin; /* Calibration Mode */
    DIPS[1] = PORTBbits.DIP2pin; /* LED Mode or Calibration Data Management */
    DIPS[3] = PORTBbits.DIP3pin; /* Slave I2C address */
    VL53L0X_Dev_t r, l;
    VL53L0X_DEV RightSensor = &r, LeftSensor = &l; /*Sensors handles*/
    VL53L0X_Error StatusL = VL53L0X_ERROR_NONE,\
                  StatusR = VL53L0X_ERROR_NONE; /*Sensors satuses */
    
    bool ledMode;
    uint8_t slaveI2CAddress;
    Mode currentMode = RUN;
    
    if(DIPS[0] == false){ /*We are in run mode*/
        currentMode = RUN;
        ledMode = DIPS[1];
        if(DIPS[2] == false){
            slaveI2CAddress = PRIM_SLAVE_I2C_ADDR;
        }else{
            slaveI2CAddress = SEC_SLAVE_I2C_ADDR;
        }
    }else if(DIPS[1] == false){ /*We are in a calibration mode*/
        if(DIPS[2] == false){
            currentMode = SPAD;
        }else{
            currentMode = OFFSET;
        }
    }else{
        if(DIPS[2] == false){
            currentMode = XTALK;
        }else{
            currentMode = RST;
        }
    }
    
    /*Configures LED pin. Led pin is RB4*/
    TRISBbits.TRISB4 = 0;
    
    /*Configure shutdown pins as outputs*/
    TRISB &= !((1 << XSHUT_L) + (1 << XSHUT_R));
    
    /* Initialize EEprom Emulator */
    DataEEInit();
    
    /*Wake up right sensor*/
    LATB = 1 << XSHUT_R;
    __delay_ms(2);
    
    /*VL53L0X_DataInit(VL53L0X_DEV Dev)*/
    LeftSensor->I2cDevAddr = 0x00; 
    /* Uses General call address so this device is always reconfigured
     * independently of it's previous configuration
     * Because we do not know if the address change is persistant.
     */
    VL53L0X_SetDeviceAddress(RightSensor, 0x54);
    LeftSensor->I2cDevAddr = 0x54;

    /*Turns on left sensor*/
    LATB |= 1 << XSHUT_L;
    __delay_ms(2);
    RightSensor->I2cDevAddr = 0x52;

    StatusR = VL53L0X_DataInit(RightSensor);
    StatusL = VL53L0X_DataInit(LeftSensor);
    
    /*Mainly used for debug, useless once in production ?*/
    if(!(StatusR == VL53L0X_ERROR_NONE && StatusL == VL53L0X_ERROR_NONE)){
        while(1){ /*TODO see with traps*/
            blinkStatusLed(2, 200, 300);
            __delay_ms(1000);
        }
    }
    
    StatusR = VL53L0X_StaticInit(RightSensor);
    StatusL = VL53L0X_StaticInit(LeftSensor);
    
    /*Mainly used for debug, useless once in production ?*/
    if(!(StatusR == VL53L0X_ERROR_NONE && StatusL == VL53L0X_ERROR_NONE)){
        while(1){
            blinkStatusLed(3, 200, 300);
            __delay_ms(1000);
        }
    }
    uint32_t refSPADCountR = 0, refSPADCountL = 0;
    uint8_t isApertureSPADR = 0, isApertureSPADL = 0;
    uint8_t vhvSettings, phaseCal; /*These variables are useless to us but required by the api*/
    int32_t offsetMicroMeterR = 0, offsetMicroMeterL = 0;
    FixPoint1616_t xTalkCompensationRateMegaCpsR = 0,\
                   xTalkCompensationRateMegaCpsL = 0;
    switch(currentMode){
        case SPAD: ;
            /*Lights LED to tell the user the calibration will begin*/
            blinkStatusLed(2, 1000, 500); /*2 blinks of 1s, 0.5s apart*/
            
            /*Keep Status LED on while measurement performed*/
            LATBbits.LATB4 = 1;
           
            /*Perform Calibration measurement*/
            VL53L0X_PerformRefSpadManagement(RightSensor, &refSPADCountR,\
                    &isApertureSPADR);
            VL53L0X_PerformRefSpadManagement(LeftSensor, &refSPADCountL,\
                    &isApertureSPADL);
            /*TODO check status and keep LED ON if status incorrect*/
            
            /*Stores data to memory*/
            writeRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            writeLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
                        
            /*Turns OFF LED*/
            LATBbits.LATB4 = 0;
            while(1){}
            break;
        case OFFSET: ;            
            /*Loads SPAD calibration data*/
            readRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            readLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
            VL53L0X_SetReferenceSpads(RightSensor, refSPADCountR,\
                    isApertureSPADR);
            VL53L0X_SetReferenceSpads(LeftSensor, refSPADCountL,\
                    isApertureSPADL);
            
            /*Performs Temperature ref calibration*/
            VL53L0X_PerformRefCalibration(RightSensor, &vhvSettings, &phaseCal);
            VL53L0X_PerformRefCalibration(LeftSensor, &vhvSettings, &phaseCal);
            
            /*Lights LED to tell the user the calibration will begin*/
            blinkStatusLed(3, 1000, 500); /*3 blinks of 1s, 0.5s apart*/
            
            /*Keep Status LED on while measurement performed*/
            LATBbits.LATB4 = 1;
            
            /*Perform Offset Calibration*/
            VL53L0X_PerformOffsetCalibration(RightSensor, OFFSET_CAL_DISTANCE,\
                    &offsetMicroMeterR);
            VL53L0X_PerformOffsetCalibration(LeftSensor, OFFSET_CAL_DISTANCE,\
                    &offsetMicroMeterL);
            /*TODO check status and keep LED ON if status incorrect*/

            /*Store data to memory*/
            writeRightOffsetCalData(&offsetMicroMeterR);
            writeLeftOffsetCalData(&offsetMicroMeterL);
            
            /*Turns OFF LED*/
            LATBbits.LATB4 = 0;
            while(1){}
            break;
            
        case XTALK: ;
            /*Loads SPAD calibration data*/
            readRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            readLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
            VL53L0X_SetReferenceSpads(RightSensor, refSPADCountR,\
                    isApertureSPADR);
            VL53L0X_SetReferenceSpads(LeftSensor, refSPADCountL,\
                    isApertureSPADL);
            
            /*Performs Temperature ref calibration*/
            VL53L0X_PerformRefCalibration(RightSensor, &vhvSettings, &phaseCal);
            VL53L0X_PerformRefCalibration(LeftSensor, &vhvSettings, &phaseCal);
            
            /*Loads offset cal data*/
            readRightOffsetCalData(&offsetMicroMeterR);
            readLeftOffsetCalData(&offsetMicroMeterL);
            VL53L0X_SetOffsetCalibrationDataMicroMeter(RightSensor,\
                    offsetMicroMeterR);
            VL53L0X_SetOffsetCalibrationDataMicroMeter(LeftSensor,\
                    offsetMicroMeterL);
            
            /*Lights LED to tell the user the calibration will begin*/
            blinkStatusLed(4, 1000, 500); /*4 blinks of 1s, 0.5s apart*/
            
            /*Keep Status LED on while measurement performed*/
            LATBbits.LATB4 = 1;
            
            /*Perform XTalk calibration*/
            VL53L0X_PerformXTalkCalibration(RightSensor, XTALK_CAL_DISTANCE,\
                                            &xTalkCompensationRateMegaCpsR);
            VL53L0X_PerformXTalkCalibration(LeftSensor, XTALK_CAL_DISTANCE,\
                                            &xTalkCompensationRateMegaCpsL);
            
            writeRightXTalkCalData(&xTalkCompensationRateMegaCpsR);
            writeLeftXTalkCalData(&xTalkCompensationRateMegaCpsL);
            
            /*Turns OFF LED*/
            LATBbits.LATB4 = 0;
            while(1){}
            break;
        case RST: ;            
            /*Lights LED to tell the user the reset will begin*/
            blinkStatusLed(5, 1000, 500); /*5 blinks of 1s, 0.5s apart*/
            
            /*Keep Status LED on while reset performed*/
            LATBbits.LATB4 = 1;
            
            /*Read all cal data from the device*/
            VL53L0X_GetReferenceSpads(RightSensor, &refSPADCountR, &isApertureSPADR);
            VL53L0X_GetReferenceSpads(LeftSensor, &refSPADCountL, &isApertureSPADL);

            VL53L0X_GetOffsetCalibrationDataMicroMeter(RightSensor, &offsetMicroMeterR);
            VL53L0X_GetOffsetCalibrationDataMicroMeter(LeftSensor, &offsetMicroMeterL);
            
            VL53L0X_GetXTalkCompensationRateMegaCps(RightSensor, &xTalkCompensationRateMegaCpsR);
            VL53L0X_GetXTalkCompensationRateMegaCps(LeftSensor, &xTalkCompensationRateMegaCpsL);
            
            /*Store all cal data to the pic memory*/
            writeRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            writeLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
            writeRightOffsetCalData(&offsetMicroMeterR);
            writeLeftOffsetCalData(&offsetMicroMeterL);
            writeRightXTalkCalData(&xTalkCompensationRateMegaCpsR);
            writeLeftXTalkCalData(&xTalkCompensationRateMegaCpsL);
            
            /*Turns OFF LED*/
            LATBbits.LATB4 = 0;
            while(1){}
            break;
            
        case RUN: ;
            while(1){
            
                if(i2c_slave_ready == true){
                    
                    /*Manage slave I2C*/
                    
                    i2c_slave_ready = false;
                } 
            }
    }


    return 0; /*Even if we will never return...*/
}

void blinkStatusLed(uint8_t blinks, uint16_t blinkDuration,\
        uint16_t msBetweenBlinks){
    uint8_t i;
    for(i = 0; i < blinks; i++){
        LATBbits.LATB4 = 1;
        __delay_ms(blinkDuration);
        LATBbits.LATB4 = 0;
        __delay_ms(msBetweenBlinks);
    }
}
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