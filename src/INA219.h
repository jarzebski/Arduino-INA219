/*

The MIT License (MIT)

Copyright (c) 2014-2023 Korneliusz Jarzębski

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#ifndef INA219_h
#define INA219_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define INA219_ADDRESS              (0x40)

#define INA219_CMD_READ             (0x01)

#define INA219_REG_CONFIG           (0x00)
#define INA219_REG_SHUNTVOLTAGE     (0x01)
#define INA219_REG_BUSVOLTAGE       (0x02)
#define INA219_REG_POWER            (0x03)
#define INA219_REG_CURRENT          (0x04)
#define INA219_REG_CALIBRATION      (0x05)

typedef enum
{
    INA219_RANGE_16V            = 0b0,
    INA219_RANGE_32V            = 0b1
} ina219_range_t;

typedef enum
{
    INA219_GAIN_40MV            = 0b00,
    INA219_GAIN_80MV            = 0b01,
    INA219_GAIN_160MV           = 0b10,
    INA219_GAIN_320MV           = 0b11
} ina219_gain_t;

typedef enum
{
    INA219_BUS_RES_9BIT         = 0b0000,
    INA219_BUS_RES_10BIT        = 0b0001,
    INA219_BUS_RES_11BIT        = 0b0010,
    INA219_BUS_RES_12BIT        = 0b0011
} ina219_busRes_t;

typedef enum
{
    INA219_SHUNT_RES_9BIT_1S    = 0b0000,
    INA219_SHUNT_RES_10BIT_1S   = 0b0001,
    INA219_SHUNT_RES_11BIT_1S   = 0b0010,
    INA219_SHUNT_RES_12BIT_1S   = 0b0011,
    INA219_SHUNT_RES_12BIT_2S   = 0b1001,
    INA219_SHUNT_RES_12BIT_4S   = 0b1010,
    INA219_SHUNT_RES_12BIT_8S   = 0b1011,
    INA219_SHUNT_RES_12BIT_16S  = 0b1100,
    INA219_SHUNT_RES_12BIT_32S  = 0b1101,
    INA219_SHUNT_RES_12BIT_64S  = 0b1110,
    INA219_SHUNT_RES_12BIT_128S = 0b1111
} ina219_shuntRes_t;

typedef enum
{
    INA219_MODE_POWER_DOWN      = 0b000,
    INA219_MODE_SHUNT_TRIG      = 0b001,
    INA219_MODE_BUS_TRIG        = 0b010,
    INA219_MODE_SHUNT_BUS_TRIG  = 0b011,
    INA219_MODE_ADC_OFF         = 0b100,
    INA219_MODE_SHUNT_CONT      = 0b101,
    INA219_MODE_BUS_CONT        = 0b110,
    INA219_MODE_SHUNT_BUS_CONT  = 0b111,
} ina219_mode_t;


class INA219
{
    public:

	bool begin(uint8_t address = INA219_ADDRESS);
	bool configure(ina219_range_t range = INA219_RANGE_32V, ina219_gain_t gain = INA219_GAIN_320MV, ina219_busRes_t busRes = INA219_BUS_RES_12BIT, ina219_shuntRes_t shuntRes = INA219_SHUNT_RES_12BIT_1S, ina219_mode_t mode = INA219_MODE_SHUNT_BUS_CONT);
	bool calibrate(float rShuntValue = 0.1, float iMaxExcepted = 2);

	ina219_range_t getRange(void);
	ina219_gain_t getGain(void);
	ina219_busRes_t getBusRes(void);
	ina219_shuntRes_t getShuntRes(void);
	ina219_mode_t getMode(void);

	float readShuntCurrent(void);
	float readShuntVoltage(void);
	float readBusPower(void);
	float readBusVoltage(void);

	float getMaxPossibleCurrent(void);
	float getMaxCurrent(void);
	float getMaxShuntVoltage(void);
	float getMaxPower(void);

    private:

	int8_t inaAddress;
	float currentLSB, powerLSB;
	float vShuntMax, vBusMax, rShunt;

	void writeRegister16(uint8_t reg, uint16_t val);
	int16_t readRegister16(uint8_t reg);
};

#endif