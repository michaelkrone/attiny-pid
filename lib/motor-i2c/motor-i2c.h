#ifndef MOTOR_I2C_H
#define MOTOR_I2C_H

#include <stdint.h>

#ifndef MOTOR_I2C_COMMAND_ADDRESS
static const uint8_t MOTOR_I2C_COMMAND_ADDRESS = 0x0F;
#endif

// available I2C commands
enum MOTOR_I2C_COMMAND
{
	// 1 byte commands
	MOTOR_I2C_COMMAND_STOP = MOTOR_I2C_COMMAND_ADDRESS,
	MOTOR_I2C_COMMAND_HALT
};

#endif
