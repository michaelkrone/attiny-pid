#ifndef PID_I2C_H
#define PID_I2C_H

#include <stdint.h>

#ifndef PID_I2C_COMMAND_ADDRESS
static const uint8_t PID_I2C_COMMAND_ADDRESS = 0x01;
#endif

// available I2C commands
enum PID_I2C_COMMAND
{
	// one byte set parameter commands
	PID_I2C_COMMAND_ENABLE = PID_I2C_COMMAND_ADDRESS,
	PID_I2C_COMMAND_DISABLE,
	PID_I2C_COMMAND_RESET,
	// multi byte set parameter commands
	PID_I2C_COMMAND_SET_VALUE,
	PID_I2C_COMMAND_SET_K_P,
	PID_I2C_COMMAND_SET_K_I,
	PID_I2C_COMMAND_SET_K_D
};

#endif
