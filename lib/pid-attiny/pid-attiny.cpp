#include "./pid-attiny.h"

/* Initialisation of PID controller parameters.
 *
 *  Initialise the variables used by the PID algorithm.
 *  \param p_factor  Proportional term.
 *  \param i_factor  Integral term.
 *  \param d_factor  Derivate term.
 *  \param pid  Struct with PID status.
 */
void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, struct PID_DATA *pid, struct PID_VALUES *values)
{
	// reset values struct for convinience
	values->referenceValue = 0;
	values->measurementValue = 0;
	values->plantValue = 0;
	pid->lastProcessValue = 0;
	pid_Reset_Integrator(pid);
	pid_Set_P(p_factor, pid);
	pid_Set_I(i_factor, pid);
	pid_Set_D(d_factor, pid);
}

/* PID control algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  \param setPoint  Desired value.
 *  \param processValue  Measured value.
 *  \param pid_st  PID status struct.
 */
void pid_Controller(struct PID_VALUES *pid_values, struct PID_DATA *pid_st)
{
	// copy volatile vars for processing
	int16_t referenceValue = pid_values->referenceValue;
	int16_t measurementValue = pid_values->measurementValue;
	int16_t error, p_term, d_term;
	int32_t i_term, ret, temp;

	// Calculate Pterm and limit error overflow
	error = referenceValue - measurementValue;
	if (error > pid_st->maxError)
	{
		p_term = MAX_INT;
	}
	else if (error < -pid_st->maxError)
	{
		p_term = -MAX_INT;
	}
	else
	{
		p_term = pid_st->P_Factor * error;
	}

	// Calculate Iterm and limit integral runaway
	temp = pid_st->sumError + error;
	if (temp > pid_st->maxSumError)
	{
		i_term = MAX_I_TERM;
		pid_st->sumError = pid_st->maxSumError;
	}
	else if (temp < -pid_st->maxSumError)
	{
		i_term = -MAX_I_TERM;
		pid_st->sumError = -pid_st->maxSumError;
	}
	else
	{
		pid_st->sumError = temp;
		i_term = pid_st->I_Factor * pid_st->sumError;
	}

	// Calculate Dterm
	d_term = pid_st->D_Factor * (pid_st->lastProcessValue - measurementValue);
	pid_st->lastProcessValue = measurementValue;

	ret = (p_term + i_term + d_term) / PID_SCALING_FACTOR;
	if (ret > MAX_INT)
	{
		ret = MAX_INT;
	}
	else if (ret < -MAX_INT)
	{
		ret = -MAX_INT;
	}

	pid_values->plantValue = ((int16_t)ret);
}

/* Resets the integrator.
 *
 *  Calling this function will reset the integrator in the PID regulator.
 */
void pid_Reset_Integrator(pidData_t *pid_st)
{
	pid_st->sumError = 0;
}

/* Allows setting PID Factors */
void pid_Set_P(int16_t p, pidData_t *pid_st)
{
	pid_st->P_Factor = p * PID_SCALING_FACTOR;
	// Limits to avoid overflow
	pid_st->maxError = MAX_INT / (pid_st->P_Factor + 1);
	pid_Reset_Integrator(pid_st);
}

void pid_Set_I(int16_t i, pidData_t *pid_st)
{
	pid_st->I_Factor = i * PID_SCALING_FACTOR;
	// Limits to avoid overflow
	pid_st->maxSumError = MAX_I_TERM / (pid_st->I_Factor + 1);
	pid_Reset_Integrator(pid_st);
}

void pid_Set_D(int16_t d, pidData_t *pid_st)
{
	pid_st->D_Factor = d * PID_SCALING_FACTOR;
	pid_Reset_Integrator(pid_st);
}
