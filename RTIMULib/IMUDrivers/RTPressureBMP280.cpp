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
    dig_T1_ = ((uint8_t)data[0] << 8) + (uint8_t)data[1];
    dig_T2_ = ((uint8_t)data[2] << 8) + (uint8_t)data[3];
    dig_T3_ = ((uint8_t)data[4] << 8) + (uint8_t)data[5];

    // Pressure coefficients
    dig_P1_ = ((uint8_t)data[6] << 8) + (uint8_t)data[7];
    dig_P2_ = ((uint8_t)data[8] << 8) + (uint8_t)data[9];
    dig_P3_ = ((uint8_t)data[10] << 8) + (uint8_t)data[11];
    dig_P4_ = ((uint8_t)data[12] << 8) + (uint8_t)data[13];
    dig_P5_ = ((uint8_t)data[14] << 8) + (uint8_t)data[15];
    dig_P6_ = ((uint8_t)data[16] << 8) + (uint8_t)data[17];
    dig_P7_ = ((uint8_t)data[18] << 8) + (uint8_t)data[19];
    dig_P8_ = ((uint8_t)data[20] << 8) + (uint8_t)data[21];
    dig_P9_ = ((uint8_t)data[22] << 8) + (uint8_t)data[23];


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

    int32_t var1, var2, adc_T;
    uint8_t sensorData[3];

    if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_TEMP_MSB, 3, sensorData, "Failed to read BMP280 Temp Register")) 
    {
        return false;
        HAL_ERROR("BMP280: Temp. Read Error\n")
    }

    // Align the data
    printf("S1: %x S2: %x S3: %x\n", sensorData[0], sensorData[1], sensorData[2]);
    adc_T = sensorData[0];
    adc_T <<=8;
    adc_T |= sensorData[1];
    adc_T <<=8;
    adc_T |= sensorData[2]; 


    adc_T >>= 4;

    printf("adc_T: %x\n", adc_T);       

    var1 = ((adc_T) / 16384.0 - (dig_T1_) / 1024.0) * (dig_T2_);
    var2 = (((adc_T) / 131072.0 - (dig_T1_) / 8192.0) * 
        ((adc_T)/131072.0 - (dig_T1_)/8192.0)) * (dig_T3_);

    printf("Var1 is: %d, Var2 is: %d\n", var1, var2);

    fineVal_ = var1 + var2;
    float cTemp = fineVal_ / 5120.0;
    float fTemp = cTemp * 1.8 + 32;

    printf("TEMP IS: %d\n", fTemp);
/*     data.pressureValid = false;
    data.temperatureValid = false;
    data.temperature = 0;
    data.pressure = 0;

    if (m_state == BMP180_STATE_IDLE) {
        // start a temperature conversion
        if (!m_settings->HALWrite(m_pressureAddr, BMP180_REG_SCO, BMP180_SCO_TEMPCONV, "Failed to start temperature conversion")) {
            return false;
        } else {
            m_state = BMP180_STATE_TEMPERATURE;
        }
    }

    pressureBackground();

    if (m_validReadings) {
        data.pressureValid = true;
        data.temperatureValid = true;
        data.temperature = m_temperature;
        data.pressure = m_pressure;
        // printf("P: %f, T: %f\n", m_pressure, m_temperature);
    } */
    return true;
}


void RTPressureBMP280::pressureBackground()
{
  /*   uint8_t data[2];

    switch (m_state) {
        case BMP180_STATE_IDLE:
        break;

        case BMP180_STATE_TEMPERATURE:
        if (!m_settings->HALRead(m_pressureAddr, BMP180_REG_SCO, 1, data, "Failed to read BMP180 temp conv status")) {
            break;
        }
        if ((data[0] & 0x20) == 0x20)
            break;                                      // conversion not finished
        if (!m_settings->HALRead(m_pressureAddr, BMP180_REG_RESULT, 2, data, "Failed to read BMP180 temp conv result")) {
            m_state = BMP180_STATE_IDLE;
            break;
        }
        m_rawTemperature = (((uint16_t)data[0]) << 8) + (uint16_t)data[1];

        data[0] = 0x34 + (m_oss << 6);
        if (!m_settings->HALWrite(m_pressureAddr, BMP180_REG_SCO, 1, data, "Failed to start pressure conversion")) {
            m_state = BMP180_STATE_IDLE;
            break;
        }
        m_state = BMP180_STATE_PRESSURE;
        break;

        case BMP180_STATE_PRESSURE:
        if (!m_settings->HALRead(m_pressureAddr, BMP180_REG_SCO, 1, data, "Failed to read BMP180 pressure conv status")) {
            break;
        }
        if ((data[0] & 0x20) == 0x20)
            break;                                      // conversion not finished
        if (!m_settings->HALRead(m_pressureAddr, BMP180_REG_RESULT, 2, data, "Failed to read BMP180 temp conv result")) {
            m_state = BMP180_STATE_IDLE;
            break;
        }
        m_rawPressure = (((uint16_t)data[0]) << 8) + (uint16_t)data[1];

        if (!m_settings->HALRead(m_pressureAddr, BMP180_REG_XLSB, 1, data, "Failed to read BMP180 XLSB")) {
            m_state = BMP180_STATE_IDLE;
            break;
        }

        // call this function for testing only
        // should give T = 150 (15.0C) and pressure 6996 (699.6hPa)

        // setTestData();

        int32_t pressure = ((((uint32_t)(m_rawPressure)) << 8) + (uint32_t)(data[0])) >> (8 - m_oss);

        m_state = BMP180_STATE_IDLE;

        // calculate compensated temperature

        int32_t X1 = (((int32_t)m_rawTemperature - m_AC6) * m_AC5) / 32768;

        if ((X1 + m_MD) == 0) {
            break;
        }

        int32_t X2 = (m_MC * 2048)  / (X1 + m_MD);
        int32_t B5 = X1 + X2;
        m_temperature = (RTFLOAT)((B5 + 8) / 16) / (RTFLOAT)10;

        // calculate compensated pressure

        int32_t B6 = B5 - 4000;
        //          printf("B6 = %d\n", B6);
        X1 = (m_B2 * ((B6 * B6) / 4096)) / 2048;
        //          printf("X1 = %d\n", X1);
        X2 = (m_AC2 * B6) / 2048;
        //          printf("X2 = %d\n", X2);
        int32_t X3 = X1 + X2;
        //          printf("X3 = %d\n", X3);
        int32_t B3 = (((m_AC1 * 4 + X3) << m_oss) + 2) / 4;
        //          printf("B3 = %d\n", B3);
        X1 = (m_AC3 * B6) / 8192;
        //          printf("X1 = %d\n", X1);
        X2 = (m_B1 * ((B6 * B6) / 4096)) / 65536;
        //          printf("X2 = %d\n", X2);
        X3 = ((X1 + X2) + 2) / 4;
        //          printf("X3 = %d\n", X3);
        int32_t B4 = (m_AC4 * (unsigned long)(X3 + 32768)) / 32768;
        //          printf("B4 = %d\n", B4);
        uint32_t B7 = ((unsigned long)pressure - B3) * (50000 >> m_oss);
        //          printf("B7 = %d\n", B7);

        int32_t p;
        if (B7 < 0x80000000)
        p = (B7 * 2) / B4;
            else
        p = (B7 / B4) * 2;

        //          printf("p = %d\n", p);
        X1 = (p / 256) * (p / 256);
        //          printf("X1 = %d\n", X1);
        X1 = (X1 * 3038) / 65536;
        //          printf("X1 = %d\n", X1);
        X2 = (-7357 * p) / 65536;
        //          printf("X2 = %d\n", X2);
        m_pressure = (RTFLOAT)(p + (X1 + X2 + 3791) / 16) / (RTFLOAT)100;      // the extra 100 factor is to get 1hPa units

        m_validReadings = true;

        // printf("UP = %d, P = %f, UT = %d, T = %f\n", m_rawPressure, m_pressure, m_rawTemperature, m_temperature);
        break;
    } */
}

void RTPressureBMP280::setTestData()
{
/*     m_AC1 = 408;
    m_AC2 = -72;
    m_AC3 = -14383;
    m_AC4 = 32741;
    m_AC5 = 32757;
    m_AC6 = 23153;
    m_B1 = 6190;
    m_B2 = 4;
    m_MB = -32767;
    m_MC = -8711;
    m_MD = 2868;

    m_rawTemperature = 27898;
    m_rawPressure = 23843;
 */}
