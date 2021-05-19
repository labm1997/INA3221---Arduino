#pragma once

#include <Arduino.h>
#include <Wire.h>

// Device address
#define INA3221_ADDRESS (0b1000000)

// Registers
#define INA3221_CONFIG_REGISTER (0x00)
#define INA3221_CH1_SHUNT_VOLTAGE_REGISTER (0x01)
#define INA3221_CH1_BUS_VOLTAGE_REGISTER (0x02)
#define INA3221_CH2_SHUNT_VOLTAGE_REGISTER (0x03)
#define INA3221_CH2_BUS_VOLTAGE_REGISTER (0x04)
#define INA3221_CH3_SHUNT_VOLTAGE_REGISTER (0x05)
#define INA3221_CH3_BUS_VOLTAGE_REGISTER (0x06)
#define INA3221_CH1_CRIT_LIMIT_REGISTER (0x07)
#define INA3221_CH1_WARN_LIMIT_REGISTER (0x08)
#define INA3221_CH2_CRIT_LIMIT_REGISTER (0x09)
#define INA3221_CH2_WARN_LIMIT_REGISTER (0x0a)
#define INA3221_CH3_CRIT_LIMIT_REGISTER (0x0b)
#define INA3221_CH3_WARN_LIMIT_REGISTER (0x0c)
#define INA3221_SHUNT_VOLTAGE_SUM_REGISTER (0x0d)
#define INA3221_SHUNT_VOLTAGE_SUM_LIMIT_REGISTER (0x0e)
#define INA3221_MASK_ENABLE_REGISTER (0x0f)
#define INA3221_POWER_VALID_UPPER_LIMIT_REGISTER (0x10)
#define INA3221_POWER_VALID_LOWER_LIMIT_REGISTER (0x11)
#define INA3221_MANUFACTURER_ID_REGISTER (0xfe)
#define INA3221_DIE_ID_REGISTER (0xff)

// Macros
#define INA3221_READBIT(data, bit) ((data & (1 << bit)) >> bit)

// Enums
typedef enum {
    INA3221_CH1 = 1,
    INA3221_CH2 = 2,
    INA3221_CH3 = 3,
} INA3221_CHANNELS;

typedef enum {
    INA3221_AVERAGING_0 = 0b000,
    INA3221_AVERAGING_4 = 0b001,
    INA3221_AVERAGING_16 = 0b010,
    INA3221_AVERAGING_64 = 0b011,
    INA3221_AVERAGING_128 = 0b100,
    INA3221_AVERAGING_256 = 0b101,
    INA3221_AVERAGING_512 = 0b110,
    INA3221_AVERAGING_1024 = 0b111,
} INA3221_AVERAGING_MODE;

typedef enum {
    INA3221_CONVERSION_TIME_140u = 0b000,
    INA3221_CONVERSION_TIME_204u = 0b001,
    INA3221_CONVERSION_TIME_332u = 0b010,
    INA3221_CONVERSION_TIME_588u = 0b011,
    INA3221_CONVERSION_TIME_1100u = 0b100,
    INA3221_CONVERSION_TIME_2116u = 0b101,
    INA3221_CONVERSION_TIME_4156u = 0b110,
    INA3221_CONVERSION_TIME_8244u = 0b111,
} INA3221_CONVERSION_TIME;

typedef enum {
    INA3221_POWER_DOWN = 0b000,
    INA3221_SHUNT_SINGLE_SHOT = 0b001,
    INA3221_BUS_SINGLE_SHOT = 0b010,
    INA3221_SHUNT_BUS_SINGLE_SHOT = 0b011,
    INA3221_SHUNT_CONTINUOUS = 0b101,
    INA3221_BUS_CONTINUOUS = 0b110,
    INA3221_SHUNT_BUS_CONTINUOUS = 0b111,
} INA3221_OPERATING_MODE;

typedef struct {
    bool ready;
    bool conversionReadyFlag;
    bool timingControlAlertFlag;
    bool powerValidAlertFlag;
    bool ch1WarningAlertFlag;
    bool ch2WarningAlertFlag;
    bool ch3WarningAlertFlag;
    bool summationAlertFlag;
    bool ch1CriticalAlertFlag;
    bool ch2CriticalAlertFlag;
    bool ch3CriticalAlertFlag;
    bool criticalAlertLatchEnable;
    bool warningAlertLatchEnable;
    bool ch1SummationControlEnable;
    bool ch2SummationControlEnable;
    bool ch3SummationControlEnable;
} maskRegister_t;

// Functions
uint16_t readRegister(uint8_t address);
void     INA3221_init();
void     INA3221_reset();
void     INA3221_enableChannel(INA3221_CHANNELS channel);
void     INA3221_disableChannel(INA3221_CHANNELS channel);
void     INA3221_setAveragingMode(INA3221_AVERAGING_MODE mode);
void     INA3221_setBusConversionTime(INA3221_CONVERSION_TIME conversionTime);
void     INA3221_setMode(INA3221_OPERATING_MODE mode);
float    INA3221_readShunt(INA3221_CHANNELS channel);
float    INA3221_readBus(INA3221_CHANNELS channel);
void     INA3221_setChannelCriticalAlertLimit(INA3221_CHANNELS channels, float shuntVoltage);
void     INA3221_setChannelWarningAlertLimit(INA3221_CHANNELS channels, float shuntVoltage);
float    INA3221_readShuntSum();
void     INA3221_setReadShuntSumLimit(float sumVoltage);
void     INA3221_updateMaskRegister();
void     INA3221_enableChannelSumMode(INA3221_CHANNELS channel);
void     INA3221_disableChannelSumMode(INA3221_CHANNELS channel);
void     INA3221_enableWarningAlertLatch();
void     INA3221_disableWarningAlertLatch();
void     INA3221_enableCriticalAlertLatch();
void     INA3221_disableCriticalAlertLatch();
bool     INA3221_getCriticalChannelsFlag(bool *ch1, bool *ch2, bool *ch3);
bool     INA3221_getSummationAlertFlag(bool *summationAlertFlag);
bool     INA3221_getCWarningChannelsFlag(bool *ch1, bool *ch2, bool *ch3);
bool     INA3221_getPowerValidAlertFlag(bool *powerValidAlertFlag);
bool     INA3221_getTimingControlAlertFlag(bool *timingControlAlertFlag);
bool     INA3221_getConversionReadyFlag(bool *conversionReadyFlag);
void     INA3221_setPowerValidUpperLimit(float busVoltage);
void     INA3221_setPowerValidLowerLimit(float busVoltage);
uint16_t INA3221_readManufacturerId();
uint16_t INA3221_readDieId();