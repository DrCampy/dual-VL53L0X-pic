/* Defines bit position for used pins */
#define XSHUT_L 15
#define XSHUT_R 12
#define INT_L 14
#define INT_R 13
#define INT_OUT 11

#define DIP1pin RB5
#define DIP2pin RB6
#define DIP3pin RB7

/*Define needed for the API to use I2C in 2.8V*/
#define USE_I2C_2V8

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
#include <xc.h>            /* Device header file                              */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include <libpic30.h>
#include <p24FJ64GA702.h>
#include <p24FJ64GA702.h>      /* __delay_ms function                             */
#include "Api/inc/core/vl53l0x_api.h" /*VL53L0X Api                           */


/******************************************************************************/
/* Custom Functions, enums,...                                                */
/******************************************************************************/
enum Mode {RUN, RST, SPAD, OFFSET, XTALK};
void blinkStatusLed(uint8_t blinks,
        uint16_t blinkDuration, uint16_t msBetweenBlinks);


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
    DIPS[0] = PORTB.DIP1pin; /* Calibration Mode */
    DIPS[1] = PORTB.DIP2pin; /* LED Mode or Calibration Data Management */
    DIPS[3] = PORTB.DIP3pin; /* Slave I2C address */
    VL53L0X_Dev_t r, l;
    VL53L0X_DEV RightSensor = &r, LeftSensor = &l; /*Sensors handles*/
    VL53L0X_Error StatusL = VL53L0X_ERROR_NONE, StatusR = VL53L0X_ERROR_NONE; /*Sensors satuses */
    
    bool ledMode;
    uint8_t slaveI2CAddress;
    Mode currentMode;
    
    if(DIPS[0] == false){ /*We are in run mode*/
        currentMode = Mode.RUN;
        ledMode = DIPS[1];
        if(DIPS[2] == false){
            slaveI2CAddress = 0x42;
        }else{
            slaveI2CAddress = 0x44;
        }
    }else if(DIPS[1] == false){ /*We are in a calibration mode*/
        if(DIPS[2] == false){
            currentMode = Mode.SPAD;
        }else{
            currentMode = Mode.OFFSET;
        }
    }else{
        if(DIPS[2] == false){
            currentMode = Mode.XTALK;
        }else{
            currentMode = Mode.RST;
        }
    }
    
    /*Configures LED pin. Led pin is RB4*/
    TRISBbits.TRISB4 = 0;
    
    /*Configure shutdown pins as outputs*/
    TRISB &= !((1 << XSHUT_L) + (1 << XSHUT_R));
    
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
        while(true){ /*TODO see with traps*/
            blinkStatusLed(2, 200, 300);
            __delay_ms(1000);
        }
    }
    
    StatusR = VL53L0X_StaticInit(RightSensor);
    StatusL = VL53L0X_StaticInit(LeftSensor);
    
    /*Mainly used for debug, useless once in production ?*/
    if(!(StatusR == VL53L0X_ERROR_NONE && StatusL == VL53L0X_ERROR_NONE)){
        while(true){
            blinkStatusLed(3, 200, 300);
            __delay_ms(1000);
        }
    }
    uint32_t refSPADCountR = 0, refSPADCountL = 0;
    uint8_t isApertureSPADR = 0, isApertureSPADL = 0;
    switch(currentMode){
        case SPAD:
            /*Lights LED to tell the user the calibration will begin*/
            blinkStatusLed(2, 1000, 500); /*2 blinks of 1s, 0.5s apart*/
            
            /*Keep Status LED on while measurement performed*/
            LATBbits.LATB4 = 1;
           
            /*Perform Calibration measurement*/
            VL53L0X_PerformReferenceSpadManagement(RightSensor, &refSPADCountR, &isApertureSPADR);
            VL53L0X_PerformReferenceSpadManagement(LeftSensor, &refSPADCountL, &isApertureSPADL);
            /*TODO check status and keep LED ON if status incorrect*/
            
            /*Stores data to memory*/
            writeRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            writeLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
                        
            /*Turns OFF LED*/
            LATBbits.LATB4 = 0;
            while(true){}
            break;
        case OFFSET:
            int32_t offsetMicroMeterR;
            int32_t offsetMicroMeterL;
            /*Loads SPAD calibration data*/
            readRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            readLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
            VL53L0X_SetReferenceSpads(RightSensor, refSPADCountR, isApertureSPADR);
            VL53L0X_SetReferenceSpads(LeftSensor, refSPADCountL, isApertureSPADL);
            
            /*Performs Temperature ref calibration*/
            VL53L0X_PerformRefCalibration(RightSensor,/*Unused*/,/*Unused*/);
            VL53L0X_PerformRefCalibration(LeftSensor,/*Unused*/,/*Unused*/);
            
            /*Lights LED to tell the user the calibration will begin*/
            blinkStatusLed(3, 1000, 500); /*3 blinks of 1s, 0.5s apart*/
            
            /*Keep Status LED on while measurement performed*/
            LATBbits.LATB4 = 1;
            
            /*Perform Offset Calibration*/
            VL53L0X_PerformOffsetCalibration(RightSensor, 100, &offsetMicroMeterR);
            VL53L0X_PerformOffsetCalibration(LeftSensor, 100, &offsetMicroMeterL);
            /*TODO check status and keep LED ON if status incorrect*/

            /*Store data to memory*/
            writeRightOffsetCalData(&offsetMicroMeterR);
            writeLeftOffsetCalData(&offsetMicroMeterL);
            
            /*Turns OFF LED*/
            LATBbits.LATB4 = 0;
            while(true){}
            break;
            
        case XTALK:
            
        case RST:
    }

    /*
     * .X_Shut_R high V
     * (wait) V
     * .Change adress of right sensor V
     * .X_Shut_L high V
     * (wait) V
     * Init
     * Check if we have to do  SPAD calibration otherwise loads it
     * Temp calibration
     * Check if we need to do offset or xTalk calibration otherwise load them
     * 
    */

    while(1)
    {
        if(i2c_slave_ready == true){
            /*Manage slave I2C*/
            i2c_slave_ready = false;
        }
    }
}

void blinkStatusLed(uint8_t blinks, uint16_t blinkDuration, uint16_t msBetweenBlinks){
    for(uint8_t i = 0; i < blinks; i++){
        LATBbits.LATB4 = 1;
        __delay_ms(blinkDuration);
        LATBbits.LATB4 = 0;
        __delay_ms(msBetweenBlinks);
    }
}

void writeRightSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD){
    /*TODO*/
}

void readRightSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD){
    /*TODO*/
}

void writeLeftSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD){
    /*TODO*/
}

void readLeftSPADCalData(uint32_t *refSPADCount, uint8_t *isApertureSPAD){
    /*TODO*/
}

void writeRightOffsetCalData(int32_t *offsetMicroMeter){
    /*TODO*/
}

void readRightOffsetCalData(int32_t *offsetMicroMeter){
    /*TODO*/
}

void writeLeftOffsetCalData(int32_t *offsetMicroMeter){
    /*TODO*/
}

void readLeftOffsetCalData(int32_t *offsetMicroMeter){
    /*TODO*/
}
