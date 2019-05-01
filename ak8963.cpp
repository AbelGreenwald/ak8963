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

#include "ak8963.h"

// If you want to set everything up yourself later, use this one.
AK8963::AK8963() {
}

// Constructor that assigns conn and ss for use by other functions.  Also,
// defaults to 16 bit precision and "Continuous 1 (8 Hz readings)" mode.
AK8963::AK8963(SPI *conn, DigitalOut *SS) {
    _connection = conn;
    _SS = SS;
    reset();
    _precision = AK8963_MODE_BIT_16;
    disableI2C();
    setOperationMode(AK8963_MODE_CONT1);
    setSensitivityAdjustment();
}

// Comments in header file
bool AK8963::checkConnection() {
    if (AK8963_REG_WIA_ID == read(AK8963_REG_WIA)) {
        return true;
    }
    else {
        return false;
    }
}

// Comments in header file
bool AK8963::checkDataReady() {
    if (AK8963_STAT_DRDY == (read(AK8963_REG_ST1) & AK8963_STAT_DRDY)) {
        return true;
    }
    else {
        return false;
    }
}

// Comments in header file
bool AK8963::checkOverrun() {
    if (AK8963_STAT_DOR == (read(AK8963_REG_ST1) & AK8963_STAT_DOR)) {
        return true;
    }
    else {
        return false;
    }
}

// Comments in header file
bool AK8963::checkOverflow() {
    if (AK8963_STAT_HOFL == (read(AK8963_REG_ST2) & AK8963_STAT_HOFL)) {
        return true;
    }
    else {
        return false;
    }
}

// Data is 2 bytes wide, combine the high value and low value into 16 bit 
// variable and call function to get uT's from reading
float AK8963::getX() {
    uint8_t lsb = read(AK8963_REG_HXL);
    uint8_t msb = read(AK8963_REG_HXH);
    uint16_t val = lsb | (msb << 0x8);
    return (_in_2_uT(val, 0x0));
}

// Data is 2 bytes wide, combine the high value and low value into 16 bit 
// variable and call function to get uT's from reading
float AK8963::getY() {
    uint8_t lsb = read(AK8963_REG_HYL);
    uint8_t msb = read(AK8963_REG_HYH);
    uint16_t val = lsb | (msb << 0x8);
    return (_in_2_uT(val, 0x1));
}

// Data is 2 bytes wide, combine the high value and low value into 16 bit 
// variable and call function to get uT's from reading
float AK8963::getZ() {
    uint8_t lsb = read(AK8963_REG_HZL);
    uint8_t msb = read(AK8963_REG_HZH);
    uint16_t val = lsb | (msb << 0x8);
    return (_in_2_uT(val, 0x2));
}

// Sets SPI connection Object manually
void AK8963::setSPIconn(SPI *conn) {
    _connection = conn;
}

// Sets Signal Select Pin manually
void AK8963::setSScon(DigitalOut *SS) {
    _SS = SS;
}

// Sets the operation mode.  See Control 1 operating modes defined in header
// library.  Note changes to the Control 1 register must start from "Power Down"
// mode then wait awhile to for them to take effect. See pg 13.
void AK8963::setOperationMode(uint8_t mode) {
    write(AK8963_REG_CNTL1, AK8963_MODE_PDN);
    _operationMode = mode;
    uint8_t regVal = _operationMode | _precision;
    write(AK8963_REG_CNTL1, regVal);
    wait_us(AK8963_WAIT_POWER_DOWN_US);
}

// Sets either 14 or 16 bit precision.  See Control 1 operating modes defined in
// header library.
void AK8963::setPrecision(uint8_t prec) {
    write(AK8963_REG_CNTL1, AK8963_MODE_PDN);
    _precision = prec;
    uint8_t regVal = _operationMode | _precision;
    write(AK8963_REG_CNTL1, regVal);
    wait_us(AK8963_WAIT_POWER_DOWN_US);
}

// Comments in header file
void AK8963::setSensitivityAdjustment() {
    setOperationMode(AK8963_MODE_FUSE_ROM);
    _sensitivityAdjustment.ASAX = read(AK8963_REG_ASAX);
    _sensitivityAdjustment.ASAY = read(AK8963_REG_ASAY);
    _sensitivityAdjustment.ASAZ = read(AK8963_REG_ASAZ);
    setOperationMode(_operationMode);
}

// Comments in header file
void AK8963::reset() {
    write(AK8963_REG_CNTL2, AK8963_REG_SRST_RST);
    wait_us(AK8963_WAIT_POWER_DOWN_US);
}

// Selects device by setting SS "low", then writes register address, and finally 
// the contents to be written.  Lastly, deselects device by setting SS high.  
// Note that the most signifigant bit in the initial write operation indicates a
// subsiquent read or write (implicit 0 for write in this case).  
// See http://www.ti.com/lit/ug/sprugp2a/sprugp2a.pdf to learn way too much about
// the SPI protocol.
void AK8963::write(uint8_t regAddr, uint8_t regVal) {
    _SS->write(0);
    _connection->write(regAddr);
    _connection->write(regVal);
    _SS->write(1);
}

// Selects device by setting SS "low", then writes register address and a 1 in
// the most signifigant bit (see below).  Lastly, writes a 0 to trigger a read
// event.  Very lastly, deselects device by setting SS high.
// Note that the most signifigant bit in the initial write operation indicates a
// subsiquent read or write (explicit 1 for read in this case).  
// See http://www.ti.com/lit/ug/sprugp2a/sprugp2a.pdf way too much about
// the SPI protocol.
uint8_t AK8963::read(uint8_t regAddr) {
    _SS->write(0);
    _connection->write(regAddr | 0x80);
    uint8_t data = _connection->write(0x00);
    _SS->write(1);
    return data;
}

// Comments in header file
void AK8963::disableI2C() {
    write(AK8963_REG_I2CDIS,AK8963_SET_I2C_DISABLE);
    wait_us(AK8963_WAIT_POWER_DOWN_US);
}

// This function converts reading data to uT, the unit of magnetic induction.
// It does so by first determining which axis's adjustment value to use (see
// header file), then executes an equation which can be found on pg 32 of the
// datasheet.  Lastly, the function returns the adjusted value for either 14 bit
// or 16 bit mode.
float AK8963::_in_2_uT(int16_t H, uint8_t axis) {
    float Hadj;
    int16_t ASA;
    switch (axis) {
        case (0x0):
            ASA = _sensitivityAdjustment.ASAX;
        case (0x1):
            ASA = _sensitivityAdjustment.ASAY;
        case (0x2):
            ASA = _sensitivityAdjustment.ASAZ;
    }
    Hadj = (float)(H * (((ASA - 128)/256.0)+1));
    if (_precision == AK8963_MODE_BIT_16) {
        return (Hadj * (float)AK8963_LSB_UT_16);
    }
    else {
        return (Hadj * (float)AK8963_LSB_UT_14);
    }
}
