

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


/*
 * Uncomment to enable debug mode.
 * 
 * In debug mode the device will check for errors after each call to a function
 * from the VL53L0X api and stop to blink if an error happened.
 */
#define DEBUG

//#define USE_I2C_1V8 //Uncomment to use 1.8V I2C

#ifndef USE_I2C_2V8 //Must be defined at compilation level.
#ifndef USE_I2C_1V8
#error "Voltage not defined !"
#endif
#endif
/******************************************************************************/
/* Custom Functions, enums,...                                                */
/******************************************************************************/
typedef enum {RUN, RST, SPAD_CAL, OFFSET_CAL, XTALK_CAL} Mode;
void blinkStatusLed(uint8_t blinks, uint16_t blinkDuration,
    uint16_t msBetweenBlinks);
void updateSpecialMeasurements();

void checkError(uint8_t nb_flashes);
void ledOn();
void ledOff();
void loadCalData();

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
volatile bool i2c_slave_ready = false;
volatile bool isLeftReady = false, isRightReady = false;
volatile bool isRightRunning = false, isLeftRunning = false;
bool i2cSecondaryAddress = false;


extern VL53L0X_DEV RightSensor, LeftSensor; /*Sensors handles*/

VL53L0X_Error StatusL = VL53L0X_ERROR_NONE,
              StatusR = VL53L0X_ERROR_NONE; /*Sensors satuses */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    /* Configure the oscillator */
    ConfigureOscillator();          

    /* Declarations */
    bool DIPS[3];
    bool ledMode;
    uint8_t slaveI2CAddress;
    Mode currentMode = RUN;
    
    /* Initializations */
    RCON = 0x2000;
    DataEEInit();                   /* Initialize EEprom Emulator */
    initVL53L0X();                  /* Initialize sensors sructures */
    configureInterrupts();
    DIPS[0] = PORTBbits.DIP1pin;    /* Calibration Mode */
    DIPS[1] = PORTBbits.DIP2pin;    /* LED Mode or Cal Data Management */
    DIPS[2] = PORTBbits.DIP3pin;    /* Slave I2C address */  
    ANSBbits.ANSB15 = 0;            /* Configure XSHUT_L as digital */
    ODCBbits.ODCB15 = 1;            /* Configure XSHUT_L as open-drain */
    TRISBbits.TRISB15 = 0;          /* Configure XSHUT_L as output */
    ANSBbits.ANSB12 = 0;            /* Configure XSHUT_R as digital */
    ODCBbits.ODCB12 = 1;            /* Configure XSHUT_R as open-drain */
    TRISBbits.TRISB12 = 0;          /* Configure XSHUT_R as output */
    TRISBbits.TRISB4 = 0;           /* Configures LED pin as output */
    
    /* Detect in which mode we are */
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
    }else{ /* DIPS[1] == true*/
        if(DIPS[2] == false){
            currentMode = XTALK_CAL;
        }else{
            currentMode = RST;
        }
    }

    /* Configure right sensor address */
    LATBbits.LATB15 = 0; /* Put left sensor to sleep */
    LATBbits.LATB12 = 1; /* Wake up right sensor */
    __delay_ms(2); /* sensor needs 2 ms to wake up */
    VL53L0X_SetDeviceAddress(RightSensor, 0x54);
    LATBbits.LATB15 = 1; /* Wakes up left sensor */
    __delay_ms(2); /* sensor needs 2 ms to wake up */
    
    StatusR = VL53L0X_DataInit(RightSensor);
    StatusL = VL53L0X_DataInit(LeftSensor);
    
    checkError(2);
    
    StatusR = VL53L0X_StaticInit(RightSensor);
    StatusL = VL53L0X_StaticInit(LeftSensor);
    
    checkError(3);
    
    /* Calibration data variables */
    uint32_t refSPADCountR = 0, refSPADCountL = 0;
    uint8_t isApertureSPADR = 0, isApertureSPADL = 0;
    uint8_t vhvSettings, phaseCal; /* These variables are useless to us
                                    * but required by the api*/
    int32_t offsetMicroMeterR = 0, offsetMicroMeterL = 0;
    FixPoint1616_t xTalkCompensationRateMegaCpsR = 0,
                   xTalkCompensationRateMegaCpsL = 0;
    switch(currentMode){
        case SPAD_CAL: ;
            /* Lights LED to tell the user the calibration will begin */
            blinkStatusLed(2, 500, 500); /* 2 blinks of 0.5s, 0.5s apart */
            
            /* Keep Status LED on while calibration performed */
            ledOn();
           
            /* Perform Calibration measurement */
            StatusR = VL53L0X_PerformRefSpadManagement(RightSensor,
                        &refSPADCountR, &isApertureSPADR);
            StatusL = VL53L0X_PerformRefSpadManagement(LeftSensor,
                        &refSPADCountL, &isApertureSPADL);
            checkError(5);
            
            /* Stores data to memory */
            writeRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            writeLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
                        
            /* Turns OFF LED */
            ledOff();
            while(1){}
            break;
        case OFFSET_CAL: ;            
            /* Loads SPAD calibration data */
            readRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            readLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
            
            StatusR = VL53L0X_SetReferenceSpads(RightSensor, refSPADCountR,
                        isApertureSPADR);
            StatusL = VL53L0X_SetReferenceSpads(LeftSensor, refSPADCountL,
                        isApertureSPADL);
            checkError(5);
            
            /* Performs Temperature ref calibration */
            StatusR = VL53L0X_PerformRefCalibration(RightSensor, &vhvSettings,
                        &phaseCal);
            StatusL = VL53L0X_PerformRefCalibration(LeftSensor, &vhvSettings,
                        &phaseCal);
            checkError(6);
            
            /* Lights LED to tell the user the calibration will begin */
            blinkStatusLed(3, 500, 500); /* 3 blinks of 0.5s, 0.5s apart */
            
            /* Keep Status LED on while calibration performed */
            ledOn();
            
            /* Perform Offset Calibration */
            StatusR = VL53L0X_PerformOffsetCalibration(RightSensor,
                        OFFSET_CAL_DISTANCE, &offsetMicroMeterR);
            StatusL = VL53L0X_PerformOffsetCalibration(LeftSensor,
                        OFFSET_CAL_DISTANCE, &offsetMicroMeterL);
            checkError(7);

            /* Store data to memory */
            writeRightOffsetCalData(&offsetMicroMeterR);
            writeLeftOffsetCalData(&offsetMicroMeterL);
            
            /* Turns OFF LED */
            ledOff();
            while(1){}
            break;
            
        case XTALK_CAL: ;
            /* Loads SPAD calibration data */
            readRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            readLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
            StatusR = VL53L0X_SetReferenceSpads(RightSensor, refSPADCountR,
                        isApertureSPADR);
            StatusL = VL53L0X_SetReferenceSpads(LeftSensor, refSPADCountL,
                        isApertureSPADL);
            checkError(5);
            
            /* Performs Temperature ref calibration */
            StatusR = VL53L0X_PerformRefCalibration(RightSensor, &vhvSettings,
                        &phaseCal);
            StatusL = VL53L0X_PerformRefCalibration(LeftSensor, &vhvSettings,
                        &phaseCal);
            checkError(6);
            /* Loads offset cal data */
            readRightOffsetCalData(&offsetMicroMeterR);
            readLeftOffsetCalData(&offsetMicroMeterL);
            StatusR = VL53L0X_SetOffsetCalibrationDataMicroMeter(RightSensor,
                        offsetMicroMeterR);
            StatusL = VL53L0X_SetOffsetCalibrationDataMicroMeter(LeftSensor,
                        offsetMicroMeterL);
            checkError(7);
            /* Lights LED to tell the user the calibration will begin */
            blinkStatusLed(4, 500, 500); /* 4 blinks of 0.5s, 0.5s apart */
            
            /* Keep Status LED on while measurement performed */
            LATBbits.LATB4 = 1;
            
            /* Perform XTalk calibration */
            StatusR = VL53L0X_PerformXTalkCalibration(RightSensor,
                        XTALK_CAL_DISTANCE, &xTalkCompensationRateMegaCpsR);
            StatusL = VL53L0X_PerformXTalkCalibration(LeftSensor,
                        XTALK_CAL_DISTANCE, &xTalkCompensationRateMegaCpsL);
            checkError(8);
            
            writeRightXTalkCalData(&xTalkCompensationRateMegaCpsR);
            writeLeftXTalkCalData(&xTalkCompensationRateMegaCpsL);
            
            /* Turns OFF LED */
            ledOff();
            while(1){}
            break;
        case RST: ;            
            /* Lights LED to tell the user the reset will begin */
            blinkStatusLed(5, 500, 500); /* 5 blinks of 0.5s, 0.5s apart */
            
            /* Keep Status LED on while reset performed */
            ledOn();
            
            /* Read all cal data from the device */
            StatusR = VL53L0X_GetReferenceSpads(RightSensor, &refSPADCountR,
                        &isApertureSPADR);
            StatusL = VL53L0X_GetReferenceSpads(LeftSensor, &refSPADCountL,
                        &isApertureSPADL);
            checkError(5);
            
            StatusR = VL53L0X_GetOffsetCalibrationDataMicroMeter(RightSensor,
                        &offsetMicroMeterR);
            StatusL = VL53L0X_GetOffsetCalibrationDataMicroMeter(LeftSensor,
                        &offsetMicroMeterL);
            checkError(7);
            
            StatusR = VL53L0X_GetXTalkCompensationRateMegaCps(RightSensor,
                        &xTalkCompensationRateMegaCpsR);
            StatusL = VL53L0X_GetXTalkCompensationRateMegaCps(LeftSensor,
                        &xTalkCompensationRateMegaCpsL);
            checkError(8);
            
            /* Store all cal data to the pic memory */
            writeRightSPADCalData(&refSPADCountR, &isApertureSPADR);
            writeLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
            writeRightOffsetCalData(&offsetMicroMeterR);
            writeLeftOffsetCalData(&offsetMicroMeterL);
            writeRightXTalkCalData(&xTalkCompensationRateMegaCpsR);
            writeLeftXTalkCalData(&xTalkCompensationRateMegaCpsL);
            
            /* Turns OFF LED */
            ledOff();
            while(1){}
            break;
            
        case RUN: ;
            VL53L0X_RangingMeasurementData_t RightMeasurement, LeftMeasurement;
            bool leftUpdated = false, rightUpdated = false;
            
            /* Load calibration data + perform reference calibration */
            loadCalData();    

            /* Configures devices */
            StatusR &= VL53L0X_SetDeviceMode(RightSensor,
                        VL53L0X_DEVICEMODE_SINGLE_RANGING);
            StatusL &= VL53L0X_SetDeviceMode(LeftSensor,
                        VL53L0X_DEVICEMODE_SINGLE_RANGING);
            StatusR &= VL53L0X_SetGpioConfig(RightSensor, 0,
                    VL53L0X_DEVICEMODE_SINGLE_RANGING,
                    VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                    VL53L0X_INTERRUPTPOLARITY_LOW);
            StatusL &= VL53L0X_SetGpioConfig(LeftSensor, 0,
                    VL53L0X_DEVICEMODE_SINGLE_RANGING,
                    VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                    VL53L0X_INTERRUPTPOLARITY_LOW);
            StatusR &= VL53L0X_SetInterruptThresholds(RightSensor,
                    VL53L0X_DEVICEMODE_SINGLE_RANGING,
                    (FixPoint1616_t)0.0, (FixPoint1616_t)50);
            StatusL &= VL53L0X_SetInterruptThresholds(LeftSensor,
                    VL53L0X_DEVICEMODE_SINGLE_RANGING,
                    (FixPoint1616_t)0.0, (FixPoint1616_t)50);
            I2CSlaveInit(slaveI2CAddress);
            
            //Enable interrupts for slave I2C and both sensors
            IFS1bits.INT2IF = 0;
            IEC1bits.INT2IE = 1;
            IFS3bits.INT3IF = 0;
            IEC3bits.INT3IE = 1;
            checkError(2);
            
            /*Main loop*/
            while(1){                
                // Start a measurement
                if(CONFIG_Lbits.CONV){
                    //Right Sensor
                    if(CONFIG_Lbits.R_EN && !isRightRunning && !isRightReady && !isLeftRunning && !rightUpdated){
                        isRightRunning = true;
                        StatusR &= VL53L0X_StartMeasurement(RightSensor);
                    }
                    
                    //Left Sensor
                    if(CONFIG_Lbits.L_EN && !isLeftRunning && !isLeftReady && !isRightRunning && !leftUpdated){
                        isLeftRunning = true;
                        StatusL &= VL53L0X_StartMeasurement(LeftSensor);
                    }
                }
                
                // Recovers measurements (right)
                if(isRightReady){
                    //Get measurement data from right sensor
                    StatusR &= VL53L0X_GetRangingMeasurementData(RightSensor,
                                &RightMeasurement);
                    StatusR &= VL53L0X_ClearInterruptMask(RightSensor, 0 /*unused*/);
                    if(RightMeasurement.RangeStatus == 0){
                        rightDist = RightMeasurement.RangeMilliMeter/10;
                        rightUpdated = true;    
                    }else if(RightMeasurement.RangeStatus == 4){
                        //Out of range
                        rightDist = 255;
                        rightUpdated = true;
                    }

                    isRightReady = false;
                }
                
                // Recovers measurements (left)
                if(isLeftReady){
                    //Get measurement data from left sensor
                    StatusL &= VL53L0X_GetRangingMeasurementData(LeftSensor,
                                &LeftMeasurement);
                    StatusL &= VL53L0X_ClearInterruptMask(LeftSensor, 0 /*unused*/);
                    
                    /* We only accept the measurement if it was successful or 
                     * out of range*/
                    if(LeftMeasurement.RangeStatus == 0){
                        //Valid measurement
                        leftDist = LeftMeasurement.RangeMilliMeter/10;
                        leftUpdated = true;
                    }else if(LeftMeasurement.RangeStatus == 4){
                        //Out of range
                        leftDist = 255;
                        leftUpdated = true;
                    }
                    isLeftReady = false;
                }
                
                updateSpecialMeasurements();
                
                //Updates interrupt state, CONV and CONFIG_FINISHED flags
                bool rightCond = rightUpdated || !CONFIG_Lbits.R_EN;
                bool leftCond = leftUpdated || !CONFIG_Lbits.L_EN;
                if(rightCond && leftCond){
                    //Raise interrupt if they are not disabled.
                    if(CONFIG_Hbits.INT_MODE != INT_OFF){
                        raiseInt();
                    }
                    
                    //full measurement performed
                    measurementFinished();
                    
                    //Reset status 
                    rightUpdated = false;
                    leftUpdated = false;
                }else if(rightUpdated || leftUpdated){
                    //Raise interrupt if we have a new measurement and INT is
                    //configured so
                    if(CONFIG_Hbits.INT_MODE == INT_L_OR_R){
                        raiseInt();
                    }
                }
                // Interrupt and CONV_FINISHED flag gets reset whenever a
                // distance register is read.
                
                //Manages I2C
                if(i2c_slave_ready == true){
                    i2c_slave_ready = false;
                    I2CSlaveExec();
                } 
                
                // Applies the config that may have been updated by I2C
                updateConfig();
                
                //If an error happened during the main loop, stop and blink LED.
                checkError(1);
            }
        default:;
            /*Unknown mode*/
    }
    return 0; /*Even if we will never return...*/
}

void blinkStatusLed(uint8_t blinks, uint16_t blinkDuration,\
        uint16_t msBetweenBlinks){
    uint8_t i;
    for(i = 0; i < blinks; i++){
        ledOn();
        __delay_ms(blinkDuration);
        ledOff();
        __delay_ms(msBetweenBlinks);
    }
}

void updateSpecialMeasurements(){
    if(!CONFIG_Lbits.L_EN & !CONFIG_Lbits.R_EN){ //Right and left disabled
        minDist = maxDist = rightDist = leftDist = 255;
    }else if(!CONFIG_Lbits.L_EN){ //Left disabled
        minDist = maxDist = rightDist;
        leftDist = 255;
    }else if(!CONFIG_Lbits.R_EN){ //Right disabled
        minDist = maxDist = leftDist;
        rightDist = 255;
    }else{ //Both sensors enabled
        // Update min and max
        if(rightDist < leftDist){
            minDist = rightDist;
            maxDist = leftDist;
        }else{
            minDist = leftDist;
            maxDist = rightDist;
        }
    }
    avgDist = (rightDist + leftDist)/2;
}

void checkError(uint8_t nb_flashes){
#ifdef DEBUG
    if(!(StatusR == VL53L0X_ERROR_NONE && StatusL == VL53L0X_ERROR_NONE)){
        //Indefinitely blinks LED
        while(1){
           blinkStatusLed(nb_flashes, 200, 300);
           __delay_ms(1000);
        }   
    }
#endif
}

// Turns on LED
void ledOn(){
    LATBbits.LATB4 = 1;
}

//Turns off LED
void ledOff(){
    LATBbits.LATB4 = 0;
}

/*
 * Reads calibration data from PIC memory and loads it to devices.
 * In case data is not available in memory, no data is loaded to the devices
 * and they will use their factory settings.
 */
void loadCalData(){    
    
    /* Calibration data variables */
    uint32_t refSPADCountR = 0, refSPADCountL = 0;
    uint8_t isApertureSPADR = 0, isApertureSPADL = 0;
    uint8_t vhvSettings, phaseCal; /* These variables are useless to us
                                    * but required by the api*/
    int32_t offsetMicroMeterR = 0, offsetMicroMeterL = 0;
    FixPoint1616_t xTalkCompensationRateMegaCpsR = 0,
                   xTalkCompensationRateMegaCpsL = 0;
    
    //Spad calibration data
    readRightSPADCalData(&refSPADCountR, &isApertureSPADR);
    readLeftSPADCalData(&refSPADCountL, &isApertureSPADL);
    
    //Checks if data exists in memory
    if(!GetaddrNotFound()){
        StatusR = VL53L0X_SetReferenceSpads(RightSensor, refSPADCountR,
                    isApertureSPADR);
        StatusL = VL53L0X_SetReferenceSpads(LeftSensor, refSPADCountL,
                    isApertureSPADL);
        checkError(5);
    }
    
    //Perform temperature reference calibration.
    StatusR = VL53L0X_PerformRefCalibration(RightSensor, &vhvSettings,
                &phaseCal);
    StatusL = VL53L0X_PerformRefCalibration(LeftSensor, &vhvSettings,
                &phaseCal);
    checkError(6);
    
    //Offset Calibration data
    readRightOffsetCalData(&offsetMicroMeterR);
    readLeftOffsetCalData(&offsetMicroMeterL);
    
    //Checks if data exists in memory
    if(!GetaddrNotFound()){
        StatusR = VL53L0X_SetOffsetCalibrationDataMicroMeter(RightSensor,
                    offsetMicroMeterR);
        StatusL = VL53L0X_SetOffsetCalibrationDataMicroMeter(LeftSensor,
                    offsetMicroMeterL);
        checkError(7);
    }
    
    //Crosstalk calibration data
    readRightXTalkCalData(&xTalkCompensationRateMegaCpsR);
    readLeftXTalkCalData(&xTalkCompensationRateMegaCpsL);
    
    //Checks if data exists in memory
    if(!GetaddrNotFound()){
        StatusR = VL53L0X_SetXTalkCompensationRateMegaCps(RightSensor,
                    xTalkCompensationRateMegaCpsR);
        StatusL = VL53L0X_SetXTalkCompensationRateMegaCps(LeftSensor,
                    xTalkCompensationRateMegaCpsL);
        checkError(8);
    }
}