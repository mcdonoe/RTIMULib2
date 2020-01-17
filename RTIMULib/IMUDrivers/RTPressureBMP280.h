////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _RTPRESSUREBMP280_H_
#define _RTPRESSUREBMP280_H_

#include "RTPressure.h"

// NOTE These values are here for reference, they are actually combined together and used
// via the Combined control words below... See the BMP280 Register map in the data sheet for
// morre information
// Control Modes

#define BMP280_MODE_FORCED       0x2
#define BMP280_MODE_NORMAL       0x3
#define BMP280_NORMAL_STANDBY_TIME    0x7  // 125us

// Filter
#define BMP280_FILTER_COEF_4  0x4

//  State definitions

#define BMP280_STATE_IDLE               0
#define BMP280_STATE_TEMPERATURE        1
#define BMP280_STATE_PRESSURE           2

//  Conversion reg defs

#define BMP280_ULTRA_LOWPOWER_T            0x1
#define BMP280_STANDARD_RESOLUTION_P            0x3

// Combined control words
// The config word has STANDBY TIME in bits [5:7], FILTER_COEF_4 in [2:4] and zeros in [0:1]
#define BMP280_CONFIG_WORD  0xA0

// Measurement control word has ULTRA_LOWPOWER_T temp measurment in bits [5:7], 
// STANDARD_RESOLUTION_P in [2:4] and NORMAL MODE in [0:1]
#define BMP280_MEASUREMENT_CONTROL_WORD  0x27


class RTIMUSettings;

class RTPressureBMP280 : public RTPressure
{
public:
    RTPressureBMP280(RTIMUSettings *settings);
    ~RTPressureBMP280();

    virtual const char *pressureName() { return "BMP280"; }
    virtual int pressureType() { return RTPRESSURE_TYPE_BMP180; }
    virtual bool pressureInit();
    virtual bool pressureRead(RTIMU_DATA& data);

private:
    void pressureBackground();
    void setTestData();

    unsigned char m_pressureAddr;                           // I2C address
    RTFLOAT m_pressure;                                     // the current pressure
    RTFLOAT m_temperature;                                  // the current temperature

    // This is the calibration data read from the sensor
    // As per BMP280 data sheet

    uint16_t dig_T1_;
    int16_t dig_T2_;
    int16_t dig_T3_;
    uint16_t dig_P1_;
    int16_t dig_P2_;
    int16_t dig_P3_;
    int16_t dig_P4_;
    int16_t dig_P5_;
    int16_t dig_P6_;
    int16_t dig_P7_;
    int16_t dig_P8_;
    int16_t dig_P9_;

    int32_t fineVal_;

    int m_state;
    int m_oss;

    uint16_t m_rawPressure;
    uint16_t m_rawTemperature;

    bool m_validReadings;
};

#endif // _RTPRESSUREBMP280_H_

