/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/*!
 * \file   VL53L0X_platform.c
 * \brief  Code function defintions for Doppler Testchip Platform Layer
 *
 */

/* sprintf(), vsnprintf(), printf()*/
//#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "vl53l0x_platform_log.h"
#include "../../../system.h"
#include <xc.h>

#define F16MHZ 16000000L
#define F8MHZ 8000000L

/*
#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...) trace_print_module_function(TRACE_MODULE_PLATFORM, level, TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) trace_print_module_function(TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif*/

/*char  debug_string[VL53L0X_MAX_STRING_LENGTH_PLT];*/

uint8_t cached_page = 0;
bool isCommInit = false;

#define MIN_COMMS_VERSION_MAJOR     1
#define MIN_COMMS_VERSION_MINOR     8
#define MIN_COMMS_VERSION_BUILD     1
#define MIN_COMMS_VERSION_REVISION  0

#define STATUS_OK              0x00
#define STATUS_FAIL            0x01

bool _check_min_version(void)
{
    return true;
}

/*Uses MSSP1 in I2C mode. Speed is 100 kbit*/
int VL53L0X_i2c_init(void)
{
    
#if(FCY == F16MHZ)
    I2C1BRG = 0x4E; /* If Fosc = 32MHz*/
#elif(FCY == F8MHZ)
    I2C1BRG = 0x26; /*If Fosc = 16MHz*/
#else
#error I2C not setup for selected FCY frequency.
#endif
    I2C1CONHbits.PCIE = 1;
    I2C1CONHbits.SCIE = 1;
    
    I2C1CONLbits.I2CEN = 1; /*enables module*/

    /* Workarround for I2C Sillicon bug.
     * After a reset of the device, if a start condition is placed on the bus,
     * a bus write collision may occur instead of a start condition. (I2C1 only)
     * Workarround : 
     * Drive SCL1 low
     * Set SDA1 as output
     * Drive SDA1 low
     * Drive SDA1 high
     */
    TRISBbits.TRISB8 = 0; //SCL1 output
    TRISBbits.TRISB9 = 0; //SDA1 output
    LATBbits.LATB8 = 0; //SCL low
    LATBbits.LATB9 = 0; //SDA low
    LATBbits.LATB8 = 1; //SDA high
    LATBbits.LATB9 = 1; //SCL high
    TRISBbits.TRISB8 = 1; //SCL1 input
    TRISBbits.TRISB9 = 1; //SDA1 input
            
    /* Enables interrupts for I2C1 */
    IEC1bits.MI2C1IE = 1;
    
    isCommInit = true;
    
    return STATUS_OK;
}
int32_t VL53L0X_comms_close(void)
{
    I2C1CONLbits.I2CEN = 0;
    isCommInit = false;
    return STATUS_OK;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t reg, uint8_t *pdata, int32_t count)
{
    /* Ensures comm port have been initialized before proceeding */
    if(!isCommInit){
        VL53L0X_i2c_init();
    }
    
    //Check I2C1STATbits.P
    //I2C1STATbits.P == 0
    /*1. Start*/
    I2C1CONLbits.SEN = 1;

    /*while(IFS1bits.MI2C1IF == false){
        Idle();
    }*/
    while(I2C1STATbits.TRSTAT == 1);
    
    /*2. Send address*/
    I2C1TRN = address;
    while(I2C1STATbits.TRSTAT == 1);

    //while(IFS1bits.MI2C1IF == 0){} /*Wait for interrupt*/

    /*3. Check for ack*/
    if(I2C1STATbits.ACKSTAT != 0){
        /*Error*/
        I2C1CONLbits.PEN = 1;
        return STATUS_FAIL;
    }
    /*4. Send Index*/
    I2C1TRN = reg;
    while(I2C1STATbits.TRSTAT == 1);

    //while(IFS1bits.MI2C1IF == 0){} /*Wait for interrupt*/

    /*5. Check for ack*/
    if(I2C1STATbits.ACKSTAT != 0){
        /*Error*/
        I2C1CONLbits.PEN = 1;
        return STATUS_FAIL;
    }

    uint8_t i = 0;
    for(; i < count; i++){
        /*6. send data*/
        I2C1TRN = pdata[i];
        while(I2C1STATbits.TRSTAT == 1);

        //while(IFS1bits.MI2C1IF == 0){} /*Wait for interrupt*/

        /*Check for ack*/
        if(I2C1STATbits.ACKSTAT != 0){
            /*Error*/
            I2C1CONLbits.PEN = 1;
            return STATUS_FAIL;
        }
    }

    /*8. Send stop*/
    I2C1CONLbits.PEN = 1;

    return STATUS_OK;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    /*1. Start*/
    I2C1CONLbits.SEN = 1;

    /*2. Send address */
    I2C1TRN = address;
    while(IFS1bits.MI2C1IF == 0){} /*Wait for interrupt*/

    /*3. Check for ack*/
    if(I2C1STATbits.ACKSTAT != 0){
        /*Error*/
        I2C1CONLbits.PEN = 1;
        return STATUS_FAIL;
    }
    /*4. Send Index*/
    I2C1TRN = index;
    while(IFS1bits.MI2C1IF == 0){} /*Wait for interrupt*/

    /*5. Check for ack*/
    if(I2C1STATbits.ACKSTAT != 0){
        /*Error*/
        I2C1CONLbits.PEN = 1;
        return STATUS_FAIL;
    }

    /*6. Enters receiving mode*/
    I2C1CONLbits.RCEN = 1;

    /*7. Send repeated start*/
    I2C1CONLbits.RSEN = 1;

    /*8. Send address + 1 for reading*/
    I2C1CONL = address+1;

    while(IFS1bits.MI2C1IF == 0){} /*Wait for interrupt*/

    /*5. Check for ack*/
    if(I2C1STATbits.ACKSTAT != 0){
        /*Error*/
        I2C1CONLbits.PEN = 1;
        I2C1CONLbits.RCEN = 0;
        return STATUS_FAIL;
    }

    uint8_t i = 0;
    for(; i < count; i++){
        /*9. gets data*/
        //while(IFS1bits.SSP1IF == 0){} /*Wait for interrupt*/
        pdata[i] = I2C1RCV;
        /*Sends ack*/
        I2C1CONLbits.ACKEN = 1;
    }

    /*8. Send stop*/
    I2C1CONLbits.PEN = 1;
    I2C1CONLbits.RCEN = 0; /*leaves receive mode*/

    return STATUS_OK;
}


int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = STATUS_OK;
    const int32_t cbyte_count = 1;

    status = VL53L0X_write_multi(address, index, &data, cbyte_count);

    return status;

}


int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = STATUS_OK;

    uint8_t  buffer[BYTES_PER_WORD];

    /* Split 16-bit word into MS and LS uint8_t*/
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    if(index%2 == 1)
    {
        status = VL53L0X_write_multi(address, index, &buffer[0], 1);
        status = VL53L0X_write_multi(address, index + 1, &buffer[1], 1);
        /* serial comms cannot handle word writes to non 2-byte aligned registers.*/
    }
    else
    {
        status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);
    }

    return status;

}


int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = STATUS_OK;
    uint8_t  buffer[BYTES_PER_DWORD];

    /* Split 32-bit word into MS ... LS bytes*/
    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);

    return status;

}


int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int32_t status = STATUS_OK;
    int32_t cbyte_count = 1;

    status = VL53L0X_read_multi(address, index, pdata, cbyte_count);

    return status;
}


int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t  status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_WORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);
	*pdata = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return status;

}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_DWORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return status;

}



/* 16 bit address functions*/

/*
int32_t VL53L0X_write_multi16(uint8_t address, uint16_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;

    return status;
}

int32_t VL53L0X_read_multi16(uint8_t address, uint16_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;

    return status;
}



int32_t VL53L0X_write_byte16(uint8_t address, uint16_t index, uint8_t data)
{
    int32_t status = STATUS_OK;
    const int32_t cbyte_count = 1;
    
    status = VL53L0X_write_multi16(address, index, &data, cbyte_count);
    
    return status;
}
*/

/*int32_t VL53L0X_write_word16(uint8_t address, uint16_t index, uint16_t data)
{
    int32_t status = STATUS_OK;

    uint8_t  buffer[BYTES_PER_WORD];
*/
    /* Split 16-bit word into MS and LS uint8_t*/
/*    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    if(index%2 == 1)
    {
        status = VL53L0X_write_multi16(address, index, &buffer[0], 1);
        status = VL53L0X_write_multi16(address, index + 1, &buffer[1], 1);*/
        /* serial comms cannot handle word writes to non 2-byte aligned registers.*/
/*    }
    else
    {
        status = VL53L0X_write_multi16(address, index, buffer, BYTES_PER_WORD);
    }

    return status;

}
*/

/*int32_t VL53L0X_write_dword16(uint8_t address, uint16_t index, uint32_t data)
{
    int32_t status = STATUS_OK;
    uint8_t  buffer[BYTES_PER_DWORD];
*/
    /* Split 32-bit word into MS ... LS bytes*/
 /*   buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) > 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) > 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    status = VL53L0X_write_multi16(address, index, buffer, BYTES_PER_DWORD);

    return status;

}


int32_t VL53L0X_read_byte16(uint8_t address, uint16_t index, uint8_t *pdata)
{
    int32_t status = STATUS_OK;
    int32_t cbyte_count = 1;

    status = VL53L0X_read_multi16(address, index, pdata, cbyte_count);

    return status;
}


int32_t VL53L0X_read_word16(uint8_t address, uint16_t index, uint16_t *pdata)
{
    int32_t  status = STATUS_OK;
    uint8_t  buffer[BYTES_PER_WORD];

    status = VL53L0X_read_multi16(address, index, buffer, BYTES_PER_WORD);
    *pdata = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return status;
}

int32_t VL53L0X_read_dword16(uint8_t address, uint16_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;
    uint8_t  buffer[BYTES_PER_DWORD];

    status = VL53L0X_read_multi16(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return status;

}*/



/*
int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
    float wait_ms = (float)wait_us/1000.0f;
    __delay_ms(wait_ms);
    return STATUS_OK;
}


int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    __delay_ms(wait_ms);
    return STATUS_OK;
}
*/


int32_t VL53L0X_set_gpio(uint8_t level)
{
    /*status = VL53L0X_set_gpio_sv(level);*/

    return STATUS_OK;

}


int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
    return STATUS_OK;
}


int32_t VL53L0X_release_gpio(void)
{
    return STATUS_OK;

}

int32_t VL53L0X_cycle_power(void)
{
    return STATUS_OK;
}


int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz){
       *ptimer_freq_hz = 0;
       return STATUS_FAIL;
}


int32_t VL53L0X_get_timer_value(int32_t *ptimer_count){
       *ptimer_count = 0;
       return STATUS_FAIL;
}
