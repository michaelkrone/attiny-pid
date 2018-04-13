#ifndef COMMON_I2C_DATA_H
#define COMMON_I2C_DATA_H

#include <stdint.h>

// I2C data
typedef struct COMMON_I2C_DATA
{
	// I2C command
	volatile uint8_t command;
	// parameter value set by the I2C master
	volatile int16_t parameterValue;
} i2cData_t;

#endif