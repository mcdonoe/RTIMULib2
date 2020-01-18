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

#include "RTPressureBMP280.h"

RTPressureBMP280::RTPressureBMP280(RTIMUSettings *settings) : RTPressure(settings)
{
    m_validReadings = false;
    fineVal_ = 0;
}

RTPressureBMP280::~RTPressureBMP280()
{
}

bool RTPressureBMP280::pressureInit()
{
    unsigned char result;
    unsigned char data[24];

    m_pressureAddr = m_settings->m_I2CPressureAddress;

    // check ID of chip

    if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_ID, 1, &result, "Failed to read BMP280 id"))
        return false;

    if (result != BMP280_DEVICE_ID) {
        HAL_ERROR1("Incorrect BMP280 id %d\n", result);
        return false;
    }

    // Configure the BMP280
    if (!m_settings->HALWrite(m_pressureAddr, BMP280_REG_CONFIG, BMP280_CONFIG_WORD, "Failed to Configure the BMP280!"))
    {
        return false;
    }

    // get calibration data
    if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_CAL_CONSTANTS_START, 24, data, "Failed to read BMP280 calibration data"))
        return false;

    // Temp Coefficients
    dig_T1_ = ((uint8_t)data[1] << 8) + (uint8_t)data[0];
    dig_T2_ = ((uint8_t)data[3] << 8) + (uint8_t)data[2];
    dig_T3_ = ((uint8_t)data[5] << 8) + (uint8_t)data[4];

    // Pressure coefficients
    dig_P1_ = ((uint8_t)data[7] << 8) + (uint8_t)data[6];
    dig_P2_ = ((uint8_t)data[9] << 8) + (uint8_t)data[8];
    dig_P3_ = ((uint8_t)data[11] << 8) + (uint8_t)data[10];
    dig_P4_ = ((uint8_t)data[13] << 8) + (uint8_t)data[12];
    dig_P5_ = ((uint8_t)data[15] << 8) + (uint8_t)data[14];
    dig_P6_ = ((uint8_t)data[17] << 8) + (uint8_t)data[16];
    dig_P7_ = ((uint8_t)data[19] << 8) + (uint8_t)data[18];
    dig_P8_ = ((uint8_t)data[21] << 8) + (uint8_t)data[20];
    dig_P9_ = ((uint8_t)data[23] << 8) + (uint8_t)data[22];


    // Kick off the measurement cycle
    if (!m_settings->HALWrite(m_pressureAddr, BMP280_REG_MEAS_CTRL, BMP280_MEASUREMENT_CONTROL_WORD, "Failed to Start Measurement Cycle!"))
    {
        return false;
    }

    HAL_INFO("BMP280 Init Complete!\n")

    return true;
}

bool RTPressureBMP280::pressureRead(RTIMU_DATA& data)
{

    data.pressureValid = false;
    data.temperatureValid = false;
    data.temperature = 0;
    data.pressure = 0;

    int32_t adc_T, adc_P;
    float var1, var2;
    uint8_t sensorData[3];

    if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_TEMP_MSB, 3, sensorData, "Failed to read BMP280 Temp Register")) 
    {
        HAL_ERROR("BMP280: Temp. Read Error\n")
        // return here, as fineVal_ won't be computed if this fails.
        // Pressure computations would be invalid
        return false;
    }
    else
    {

        // Align the data pulled from the REGISTERS
        adc_T = sensorData[0];
        adc_T <<=8;
        adc_T |= sensorData[1];
        adc_T <<=8;
        adc_T |= sensorData[2]; 

        adc_T >>= 4;

        // Temperature computations
        var1 = ((adc_T) / 16384.0 - (dig_T1_) / 1024.0) * (dig_T2_);
        var2 = (((adc_T) / 131072.0 - (dig_T1_) / 8192.0) * 
            ((adc_T)/131072.0 - (dig_T1_)/8192.0)) * (dig_T3_);

        fineVal_ = var1 + var2;
        float cTemp = fineVal_ / 5120.0;

        data.pressureValid = false;
        data.temperatureValid = true;
        data.temperature = cTemp;
        data.pressure = 0;
    }

    if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_PRESS_MSB, 3, sensorData, "Failed to read BMP280 Pressure Register"))
    {
        HAL_ERROR("BMP280: Pressure Read Error\n");
    }
    else
    {
    
        // Align the data pulled from the REGISTERS
        adc_P = sensorData[0];
        adc_P <<=8;
        adc_P |= sensorData[1];
        adc_P <<=8;
        adc_P |= sensorData[2]; 

        adc_P >>= 4;
    
        // Pressure offset calculations
        var1 = (fineVal_ / 2.0) - 64000.0;
        var2 = var1 * var1 * (dig_P6_) / 32768.0;
        var2 = var2 + var1 * (dig_P5_) * 2.0;
        var2 = (var2 / 4.0) + ((dig_P4_) * 65536.0);
        var1 = ((dig_P3_) * var1 * var1 / 524288.0 + ( dig_P2_) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * (dig_P1_);
        float p = 1048576.0 - adc_P;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = (dig_P9_) * p * p / 2147483648.0;
        var2 = p * (dig_P8_) / 32768.0;
        float pressure = (p + (var1 + var2 + (dig_P7_)) / 16.0) / 100;

        data.pressureValid = true;
        data.pressure = pressure;

    }    
    return true;
}


void RTPressureBMP280::pressureBackground()
{

}

void RTPressureBMP280::setTestData()
{
}
