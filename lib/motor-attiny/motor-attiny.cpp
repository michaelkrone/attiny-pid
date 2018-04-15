#include "./motor-attiny.h"

/* Initialisation of motor controller parameters.*/
void motor_Init(uint8_t upPin, uint8_t downPin, struct MOTOR_DATA *data)
{
	// reset values struct for convinience
	data->upPin = upPin;
	data->downPin = downPin;
	pinMode(data->upPin, OUTPUT);
	pinMode(data->downPin, OUTPUT);
	// force stoping the motor and set structs up/down values
	motor_set_upPin(LOW, data, TRUE);
	motor_set_downPin(LOW, data, TRUE);
}

/* Control motor up.*/
void motor_up(uint8_t speed, struct MOTOR_DATA *data)
{
	motor_set_downPin(LOW, data);
	motor_set_upPin(speed, data);
}

/* Control motor down.*/
void motor_down(uint8_t speed, struct MOTOR_DATA *data)
{
	motor_set_upPin(LOW, data);
	motor_set_downPin(speed, data);
}

/* Stop motor.*/
void motor_stop(struct MOTOR_DATA *data)
{
	motor_set_upPin(LOW, data);
	motor_set_downPin(LOW, data);
}

/* Halt motor.*/
void motor_halt(struct MOTOR_DATA *data)
{
	motor_set_upPin(ANALOG_WRITE_MAX, data);
	motor_set_downPin(ANALOG_WRITE_MAX, data);
}

inline void motor_set_upPin(uint8_t value, struct MOTOR_DATA *data, bool force)
{
	if (force == TRUE || value != data->upValue)
	{
		analogWrite(data->upPin, value);
		data->upValue = value;
	}
}

inline void motor_set_downPin(uint8_t value, struct MOTOR_DATA *data, bool force)
{
	if (force == TRUE || value != data->downValue)
	{
		analogWrite(data->downPin, value);
		data->downValue = value;
	}
}