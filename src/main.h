#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define I2C_ATTINY
#define I2C_SLAVE_ADDRESS 0x4 // the 7-bit I2C slave address

// PIN_PID_READ default = A3
#define PIN_MOTOR_UP PB1
#define PIN_MOTOR_DOWN PB4

/* Sampling Time Interval
 * Specify the desired PID sample time interval
 * With a 8-bit counter (255 cylces to overflow), the time interval value is calculated as follows:
 * PID_TIME_INTERVAL = ( desired interval [sec] ) * ( frequency [Hz] ) / 255
 */
#define PID_TIME_INTERVAL 50

/* P, I and D parameter values
 * The K_P_DEFAULT, K_I_DEFAULT and K_D_DEFAULT values (P, I and D gains)
 * need to be modified to adapt to the application at hand
 */
#define K_P_DEFAULT 1.00
#define K_I_DEFAULT 0.00
#define K_D_DEFAULT 0.00
// #define PID_SCALING_FACTOR 128

#define ANALOG_READ_MIN 0
#define ANALOG_READ_MAX 1023
#define ANALOG_WRITE_MIN 0
#define ANALOG_WRITE_MAX 255

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE (16)
#endif

#include "../lib/pid-attiny/pid-attiny.h"
#include "../lib/motor-attiny/motor-attiny.h"
#include "../lib/i2c-attiny/i2c.h"
#include "../lib/pid-i2c/pid-i2c.h"
#include "../lib/motor-i2c/motor-i2c.h"

// Boolean values
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

typedef struct GLOBAL_FLAGS
{
	// TRUE when PID control loop should run one time (update measure value)
	volatile uint8_t pidTimer : 1;
	// TRUE when PID is enabled
	volatile uint8_t pidEnabled : 1;
	// TRUE if an I2C Action is triggered
	volatile uint8_t i2cAction : 1;
	// TRUE if an I2C read request is triggered
	volatile uint8_t sendValue : 1;
	// cache read pid value mode
	volatile uint8_t readMode : 2;
	uint8_t dummy : 2;
	// I2C parameter value and command
	i2cData_t i2cData;
} globalFlags_t;

#endif
