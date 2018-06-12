// MIT License

// Copyright (c) 2018 Abel Greenwald

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This library was written based on AKM's datasheet for the AKM8963 3-Axis
// Electronic Compass.  References to this document can be found in this 
// software's repo or here:
// https://www.akm.com/akm/en/file/datasheet/AK8963C.pdf

#ifndef AK8963_H
#define AK8963_H

#include "mbed.h"

// Registers (pg. 26)
#define AK8963_REG_WIA    0x00
#define AK8963_REG_INFO   0x01
#define AK8963_REG_ST1    0x02
#define AK8963_REG_HXL    0x03
#define AK8963_REG_HXH    0x04
#define AK8963_REG_HYL    0x05
#define AK8963_REG_HYH    0x06
#define AK8963_REG_HZL    0x07
#define AK8963_REG_HZH    0x08
#define AK8963_REG_ST2    0x09
#define AK8963_REG_CNTL1  0x0A
#define AK8963_REG_CNTL2  0x0B
#define AK8963_REG_ASTC   0x0C
#define AK8963_REG_TS1    0x0D
#define AK8963_REG_TS2    0x0E
#define AK8963_REG_I2CDIS 0x0F
#define AK8963_REG_ASAX   0x10
#define AK8963_REG_ASAY   0x11
#define AK8963_REG_ASAZ   0x12

// Status 1 (pg. 28)
#define AK8963_STAT_DRDY 0x01
#define AK8963_STAT_DOR  0x02

// Status 2 (pg. 30)
#define AK8963_STAT_HOFL 0x08
#define AK8963_STAT_BITM 0x16

// Control 1 Register (pg. 30)
#define AK8963_MODE_PDN      0x00
#define AK8963_MODE_SNG      0x01
#define AK8963_MODE_CONT1    0x02
#define AK8963_MODE_TRG      0x04
#define AK8963_MODE_CONT2    0x06
#define AK8963_MODE_TEST     0x08
#define AK8963_MODE_FUSE_ROM 0x0F
#define AK8963_MODE_BIT_14   0x00
#define AK8963_MODE_BIT_16   0x10

// Control 2 (pg. 31)
#define AK8963_REG_SRST_NORM 0x00
#define AK8963_REG_SRST_RST  0x01

// Other
#define AK8963_REG_WIA_ID         0x48 // Device ID of AKM8963
#define AK8963_LSB_UT_14          0.6  // uT/LSB typ. 14 bit
#define AK8963_LSB_UT_16          0.15 // uT/LSB typ. 16 bit
#define AK8963_WAIT_POWER_DOWN_US 100  // Time until power down
#define AK8963_SET_I2C_DISABLE    0x1B // Page 31

class AK8963 {

private:

    // Stores the hard coded sensitivity adjustment values used for conversion
    // from 1 least signifigant bit (LSB) to microteslas (uT)
    typedef struct {
        uint8_t ASAX;
        uint8_t ASAY;
        uint8_t ASAZ;
    } SensitivityAdjustment;

    // Stores either 14 or 16 bit precision read from ST2 register.
    uint8_t _precision;
    
    // Stores the operation mode state read from CNTL1 register.
    uint8_t _operationMode;
    
    // Holds the address of an SPI object.  Used to send data to the wire.
    SPI *_connection;
    
    // Holds the address of a DigitalOut object.  Used to enable or disable 
    // slave for reading and writing (r/w).
    DigitalOut *_SS;
    
    // Local instance of sensivitity struct used for fast lookups when doing
    // math.
    SensitivityAdjustment _sensitivityAdjustment;
    
    // Completes the algorithm required to convert LSB to uT.
    float _in_2_uT(int16_t H, uint8_t axis);
    
    // Disables I2C on the chip as this is an SPI library.
    void disableI2C();
    
    // Executes reads from the chip.
    uint8_t read(uint8_t regAddr);
    
    // Executes writes on the chip.
    void write(uint8_t regAddr, uint8_t regVal);

    
public:

    // Default constructor.
    AK8963();
    
    // Constructor.  Object needs an SPI object and a slave select pin object.
    AK8963(SPI *conn, DigitalOut *SS);

    // Evaluates the connection by reading the WIA register and comparing with
    // static value.  See pg 28.
    bool checkConnection(void);
    
    // Evaluates whether or not the chip has declared data ready in the Status 1 
    // Register.  See page 28.
    bool checkDataReady(void);
    
    // Evaluates whether or not a reading has been skipped by reading the Status
    // 1 register.  See page 28.
    bool checkOverrun(void);
    
    // Evaluates whether or not a magnetic reading overflow has occured.  See pg
    // 30.  *NOTE* this register should be read after every reading is taken.
    // read page 30 for details.
    bool checkOverflow(void);

    // Retrieve X coordinate reading in uT
    float getX(void);

    // Retrieve Y coordinate reading in uT
    float getY(void);

    // Retrieve Z coordinate reading it uT
    float getZ(void);

    // Set or change the SPI connection object
    void setSPIconn(SPI *conn);
    
    // Set or change the Digitalout slave select object
    void setSScon(DigitalOut *SS);

    // retrieve the sensitivity adjectment values stored in the ASA* registers.
    // See page 32.
    void setSensitivityAdjustment(void);

    // Sets operation modes in the CNTL1 Register.  See page 30.
    void setOperationMode(uint8_t mode);
    
    // Sets the precision to either 14 bit or 16 bit values.  See page 30.
    void setPrecision(uint8_t mode);

    // Soft resets all registers by writing to the CNTL2 register.  See page 31
    // and all registers "reset state" value throughout the document.
    void reset();    

};

#endif
