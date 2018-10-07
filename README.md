# ATTiny85 PID Controller

This library provides an I2C controllable ATTiny85 PID contrller.

Olimexino85 Pinout:

```
SCL = 2
SDA = 0
MOTOR_UP = 1 (PB1)
MOTOR_DOWN = 4 (PB4)
SIGNAL_READ = 3 (A3)
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

Reading the measurement value:

```
	// Read from Slave
	Wire.requestFrom(target, (size_t)(sizeof i2cData.parameterValue));

	if (Wire.getError())
	{
		Serial.print("FAIL\n");
	}
	else
	{
		// If no error then read Rx data into buffer and print
		uint8_t len = io::i2cRead(i2cData.parameterValue, true);
		// Serial.printf("read bytes: %d, value: %d\n", len, i2cData.parameterValue);
	}
```

Example for sending a byte command:

```
	Wire.beginTransmission(0x4);		  // Slave address
	io::i2cWrite(PID_I2C_COMMAND_ENABLE); // Write string to I2C Tx buffer (incl. string null at end)
	Wire.endTransmission();

	if (Wire.getError())
		...
```

Example for sending a command with a parameter:

```
	i2cData.parameterValue = 10;
	Wire.beginTransmission(0x4);		   // Slave address
	io::i2cWrite(PID_I2C_COMMAND_SET_K_P);
	io::i2cWrite(i2cData.parameterValue);
	Wire.endTransmission();

	if (Wire.getError())
		...
```
