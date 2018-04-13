#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <TinyWireS.h>
#include "i2c-data.h"

template <typename T>
unsigned int i2cWrite(const T &value)
{
	uint8_t *p = (uint8_t *)&value;
	unsigned int i;
	for (i = 0; i < sizeof value; i++)
	{
		TinyWireS.send(*p++);
	}
	return sizeof(value);
}

template <typename T>
unsigned int i2cRead(T &value)
{
	uint8_t *p = (uint8_t *)&value;
	unsigned int i;
	for (i = 0; i < sizeof value; i++)
	{
		*p++ = TinyWireS.receive();
	}
	return i;
}

#endif
