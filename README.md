# ATTiny85 PID Controller

This library provides an I2C controllable ATTiny85 PID contrller.

Olimex85 Pinout:

```
SCL = 2
SCD = 0
MOTOR_UP=1
MOTOR_DOWN=4
SIGNAL_READ=3
```

The following I2C commands are supported:

```
	PID_I2C_COMMAND_ENABLE
	PID_I2C_COMMAND_DISABLE
	PID_I2C_COMMAND_RESET
	PID_I2C_COMMAND_SET_VALUE
	PID_I2C_COMMAND_SET_K_P
	PID_I2C_COMMAND_SET_K_I
	PID_I2C_COMMAND_SET_K_D
	MOTOR_I2C_COMMAND_STOP
	MOTOR_I2C_COMMAND_HALT
```
