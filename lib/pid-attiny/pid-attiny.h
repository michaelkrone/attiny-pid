#ifndef PID_H
#define PID_H

#ifndef PID_SCALING_FACTOR
#define PID_SCALING_FACTOR 128
#endif

#include <stdint.h>

/* Maximum values
 *
 * Needed to avoid sign/overflow problems
 */
// Maximum value of variables
#define MAX_INT INT16_MAX
#define MAX_LONG INT32_MAX
#define MAX_I_TERM (MAX_LONG / 2)

/* PID Status
 *
 * Setpoints and data used by the PID control algorithm
 */
typedef struct PID_DATA
{
	// Last process value, used to find derivative of process value.
	int16_t lastProcessValue;
	// Summation of errors, used for integrate calculations
	int32_t sumError;
	// The Proportional tuning constant, multiplied with PID_SCALING_FACTOR
	int16_t P_Factor;
	// The Integral tuning constant, multiplied with PID_SCALING_FACTOR
	int16_t I_Factor;
	// The Derivative tuning constant, multiplied with PID_SCALING_FACTOR
	int16_t D_Factor;
	// Maximum allowed error, avoid overflow
	int16_t maxError;
	// Maximum allowed sumerror, avoid overflow
	int32_t maxSumError;
} pidData_t;

typedef struct PID_VALUES
{
	// the set point value
	volatile int16_t referenceValue;
	// the current state
	volatile int16_t measurementValue;
	// the plant to set
	volatile int16_t plantValue;
} pidValues_t;

void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, struct PID_DATA *pid, struct PID_VALUES *values);
void pid_Controller(struct PID_VALUES *pid_values, struct PID_DATA *pid_st);
void pid_Reset_Integrator(pidData_t *pid_st);
void pid_Set_P(int16_t p, pidData_t *pid_st);
void pid_Set_I(int16_t i, pidData_t *pid_st);
void pid_Set_D(int16_t d, pidData_t *pid_st);

#endif
