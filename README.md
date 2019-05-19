# ATTiny85 PID Controller

This library provides an I2C controllable ATTiny85 PID controller.

### Two pin control
| Function | Name | ATTiny85 | Olimexino85 |
|----------|------|-------------|----------|
| SCL | PB2 | 7 | 2 |
| SDA | PB0 | 5 | 0 |
| MOTOR_UP | PB1[^1] | 6 | 1 |
| MOTOR_DOWN | PB4[^2] | 3 | 4 |
| SIGNAL_READ | A3 | 2 | 3 |

[^1]: PWM on timer 0

[^2]: PWM on timer 1

### Three pin control
| Function | Name | ATTiny85 | Olimexino85 |
|----------|------|----------|-------------|
| SCL | PB2 | 7 | 2 |
| SDA | PB0 | 5 | 0 |
| MOTOR_UP | PB5/#RESET | 1 | #RST |
| MOTOR_DOWN | PB4 | 3 | 4 |
| MOTOR_SPEED | PB1* | 6 | 1 |
| SIGNAL_READ | A3 | 2 | 3 |

The following I2C commands are supported:

```
	MOTOR_I2C_COMMAND_STOP
	MOTOR_I2C_COMMAND_HALT
	PID_I2C_COMMAND_ENABLE
	PID_I2C_COMMAND_DISABLE
	PID_I2C_COMMAND_RESET
	PID_I2C_COMMAND_SET_VALUE
	PID_I2C_COMMAND_SET_K_P
	PID_I2C_COMMAND_SET_K_I
	PID_I2C_COMMAND_SET_K_D
	PID_I2C_COMMAND_SET_DEAD_BAND
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
