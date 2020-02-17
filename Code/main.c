

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
#include <xc.h>            /* Device header file                              */
#include <stdbool.h>       /* Includes true/false definition                  */
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "config.h"        /* Configuration definitions                       */
#include <libpic30.h>
#include "Api/inc/core/vl53l0x_api.h" /*VL53L0X Api                           */
#include "data_storage.h"
#include "DEEE/Include/DEE Emulation 16-bit/DEE Emulation 16-bit.h"
#include "sensor.h"
#include "SlaveI2C.h"

/******************************************************************************/
/* Custom Functions, enums,...                                                */
/******************************************************************************/
typedef enum {RUN, RST, SPAD_CAL, OFFSET_CAL, XTALK_CAL} Mode;
void blinkStatusLed(uint8_t blinks, uint16_t blinkDuration,\
    uint16_t msBetweenBlinks);


/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
volatile bool i2c_slave_ready = false;
bool i2cSecondaryAddress = false;
VL53L0X_DEV RightSensor, LeftSensor; /*Sensors handles*/

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    
    /* Configure the oscillator for the device */
    ConfigureOscillator();
    
    /* Initialize EEprom Emulator */
    DataEEInit();
    
    bool DIPS[3];
    DIPS[0] = PORTBbits.DIP1pin; /* Calibration Mode */
    DIPS[1] = PORTBbits.DIP2pin; /* LED Mode or Calibration Data Management */
    DIPS[2] = PORTBbits.DIP3pin; /* Slave I2C address */
    VL53L0X_Dev_t r, l;
    RightSensor = &r; LeftSensor = &l; /*Sensors handles*/
    VL53L0X_Error StatusL = VL53L0X_ERROR_NONE,\
                  StatusR = VL53L0X_ERROR_NONE; /*Sensors satuses */
    uint8_t slaveI2CAddress;
    Mode currentMode = RUN;
    
    /* 
     * Uses General call address so devices are always reconfigured
     * independently of their previous configuration
     * If their address was changed by a previous config it may
     * otherwise cause problems.
     */
    LeftSensor->I2cDevAddr = 0x00; 
    RightSensor->I2cDevAddr = 0x00;
    
    bool ledMode;

    
    if(DIPS[0] == false){ /*We are in run mode*/
        currentMode = RUN;
        ledMode = DIPS[1];
        if(DIPS[2] == false){
            readI2CSlaveAddress(&slaveI2CAddress);
            if(GetaddrNotFound()){
                //Address not found in memory. Using default
                slaveI2CAddress = PRIM_SLAVE_I2C_ADDR;
            }
        }else{
            slaveI2CAddress = SEC_SLAVE_I2C_ADDR;
            i2cSecondaryAddress = true;
        }
    }else if(DIPS[1] == false){ /*We are in a calibration mode*/
        if(DIPS[2] == false){
            currentMode = SPAD_CAL;
        }else{
            currentMode = OFFSET_CAL;
        }
    }else{
        if(DIPS[2] == false){
            currentMode = XTALK_CAL;
        }else{
            currentMode = RST;
        }
    }

    /*Configures LED pin. Led pin is RB4*/
    TRISBbits.TRISB4 = 0;
    
    /*Configure shutdown pins as outputs*/
    TRISB &= !((1 << XSHUT_L) + (1 << XSHUT_R));
    
    /*Configure right sensor address*/
    powerOnRightSensor(); /*Wake up right sensor*/
    __delay_ms(2); /* sensor needs 2 ms to wake up */
    VL53L0X_SetDeviceAddress(RightSensor, 0x54);
    RightSensor->I2cDevAddr = 0x54;
    powerOffRightSensor();

    /*Configure left sensor address*/
    powerOnLeftSensor(); /*Wake up left sensor*/
    __delay_ms(2); /* sensor needs 2 ms to wake up */
    VL53L0X_SetDeviceAddress(LeftSensor, 0x52);
    LeftSensor->I2cDevAddr = 0x52;
    
    powerOnRightSensor();
    
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
    
    /* Calibration data variables */
    uint32_t refSPADCountR = 0, refSPADCountL = 0;
    uint8_t isApertureSPADR = 0, isApertureSPADL = 0;
    uint8_t vhvSettings, phaseCal; /*These variables are useless to us but required by the api*/
    int32_t offsetMicroMeterR = 0, offsetMicroMeterL = 0;
    FixPoint1616_t xTalkCompensationRateMegaCpsR = 0,\
                   xTalkCompensationRateMegaCpsL = 0;
    switch(currentMode){
        case SPAD_CAL: ;
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
        case OFFSET_CAL: ;            
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
            
        case XTALK_CAL: ;
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
            
        case RUN:
            I2CSlaveInit(slaveI2CAddress);
            
            /*Main loop*/
            while(1){
                if(i2c_slave_ready == true){
                    /*Manage slave I2C*/    
                    I2CSlaveExec();
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
