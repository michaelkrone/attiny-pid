#include <avr/io.h>
#include <WProgram.h>
#include "main.h"

// Parameters for PID regulator
pidData_t pidData;
// PID in and output values
pidValues_t pidValues;
// motor driver config
motorConfig_t motorData;
// Global parameter flags, disable read timer, pid timer, pid enabled and i2c action
globalFlags_t gFlags = {FALSE, FALSE, FALSE, FALSE};
// var for copiying volatile parameter values
volatile int16_t parameterCopy;

/* Overflow Interrupt Handler called if TCNT0 switches form
 * 255 to 0, approx. every 2ms
 */
#ifndef TIMER0_OVF_vect
#define TIMER0_OVF_vect TIMER0_OVF0_vect
#endif

/* Set control input to system
 *
 * Set the output from the controller as input
 * to system.
 */
void setInput(void)
{
    int16_t plantValue, measurementValue;
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        plantValue = pidValues.plantValue;
        measurementValue = pidValues.measurementValue;
    }

    int16_t plant = map(plantValue, -MAX_INT, MAX_INT, ANALOG_WRITE_MIN, ANALOG_WRITE_MAX);
    if (plantValue > 0 && measurementValue < ANALOG_READ_MAX)
    {
        motor_up(plant, &motorData);
    }
    else if (plantValue < 0 && measurementValue > ANALOG_READ_MIN)
    {
        motor_down(plant, &motorData);
    }
    else
    {
        motor_stop(&motorData);
    }
}

/* Reat system input value atomically */
inline uint16_t readSignal(void)
{

    ADCSRA |= (1 << ADSC); // start ADC measurement
    while (ADCSRA & (1 << ADSC))
        ; // wait till conversion complete

    // for 10-bit resolution:
    uint8_t adcLoByte = ADCL; // get the sample value from ADCL
    // map(adcRaw, ANALOG_READ_MIN, ANALOG_READ_MAX, -MAX_INT, MAX_INT);
    return (ADCH << 8 | adcLoByte); // add lobyte and hibyte
}

/* Timer interrupt to control the sampling interval
 */
ISR(TIMER0_OVF_vect)
{
    volatile static uint16_t doPid = 0;
    if (gFlags.pidEnabled == FALSE)
    {
        doPid = 0;
    }
    else
    {
        if (doPid < PID_TIME_INTERVAL)
        {
            doPid++;
        }
        else
        {
            gFlags.pidTimer = TRUE;
            doPid = 0;
        }
    }
}

/**
 * The I2C data received handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
 */
void receiveEvent(uint8_t byteCount)
{
    // Sanity-check
    if (byteCount < 1 || byteCount > TWI_RX_BUFFER_SIZE)
    {
        return;
    }

    // read command
    uint8_t len = i2cRead(gFlags.i2cData.command);
    if (len > 0)
    {
        byteCount -= len;
        gFlags.i2cAction = TRUE;

        // check if a pareter exists for the command
        if (byteCount >= (sizeof gFlags.i2cData.parameterValue))
        {
            i2cRead(gFlags.i2cData.parameterValue);
        }
    }
}

/**
 * The I2C data request handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
 */
void requestEvent(void)
{
    int16_t value = readSignal();
    i2cWrite(value);
}

inline void readI2CParameter(void)
{
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        parameterCopy = gFlags.i2cData.parameterValue;
    }
}

/* Init of PID controller demo
 */
void InitPID(void)
{

    pid_Init(K_P_DEFAULT, K_I_DEFAULT, K_D_DEFAULT, &pidData, &pidValues);

    ADMUX =
        (0 << ADLAR) | // do not left shift result (for 10-bit values)
        (0 << REFS1) | // Sets ref. voltage to VCC, bit 1
        (0 << REFS0) | // Sets ref. voltage to VCC, bit 0
        (1 << MUX0) |  //combined with next line…
        (1 << MUX1);   // sets ADC3 (A3/PB3) as analog input channel

    ADCSRA =
        (1 << ADEN) |  // Enable ADC
        (1 << ADPS2) | // set prescaler to 128, bit 2
        (1 << ADPS1) | // set prescaler to 128, bit 1
        (0 << ADPS0);  // set prescaler to 128, bit 0

    // Set up timer, enable timer/counter 0 overflow interrupt
    TCCR0A = (1 << CS00);
    TIMSK |= (1 << TOIE0);
    TCNT0 = 0;
}

void InitMotor(void)
{
    motor_Init(PIN_MOTOR_UP, PIN_MOTOR_DOWN, &motorData);
}

void InitI2C(void)
{
    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onReceive(receiveEvent);
    TinyWireS.onRequest(requestEvent);
}

/*  PID controller
 */
void setup(void)
{
    InitPID();
    InitMotor();
    InitI2C();
    // enable interrupts
    sei();
}

void loop(void)
{
    TinyWireS_stop_check();

    // Run PID calculations once every PID timer timeout
    if (gFlags.pidEnabled == TRUE && gFlags.pidTimer == TRUE)
    {
        ATOMIC_BLOCK(ATOMIC_FORCEON)
        {
            pidValues.measurementValue = readSignal();
        }
        pid_Controller(&pidValues, &pidData);
        setInput();
        gFlags.pidTimer = FALSE;
    }

    // Process I2C Commands
    if (gFlags.i2cAction)
    {
        switch (gFlags.i2cData.command)
        {

        case PID_I2C_COMMAND_ENABLE:
            pid_Reset_Integrator(&pidData);
            gFlags.pidEnabled = TRUE;
            break;

        case PID_I2C_COMMAND_DISABLE:
        case MOTOR_I2C_COMMAND_STOP:
            ATOMIC_BLOCK(ATOMIC_FORCEON)
            {
                gFlags.pidEnabled = FALSE;
                motor_stop(&motorData);
            }
            break;

        case PID_I2C_COMMAND_RESET:
            pid_Reset_Integrator(&pidData);
            break;

        case PID_I2C_COMMAND_SET_VALUE:
            readI2CParameter();
            if (parameterCopy > ANALOG_READ_MAX)
            {
                pidValues.referenceValue = ANALOG_READ_MAX;
            }
            else if (parameterCopy < ANALOG_READ_MIN)
            {
                pidValues.referenceValue = ANALOG_READ_MIN;
            }
            else
            {
                pidValues.referenceValue = parameterCopy;
            }
            pid_Reset_Integrator(&pidData);
            break;

        case PID_I2C_COMMAND_SET_K_P:
            readI2CParameter();
            pid_Set_P(parameterCopy, &pidData);
            break;

        case PID_I2C_COMMAND_SET_K_I:
            readI2CParameter();
            pid_Set_I(parameterCopy, &pidData);
            break;

        case PID_I2C_COMMAND_SET_K_D:
            readI2CParameter();
            pid_Set_D(parameterCopy, &pidData);
            break;

        case MOTOR_I2C_COMMAND_HALT:
            ATOMIC_BLOCK(ATOMIC_FORCEON)
            {
                gFlags.pidEnabled = FALSE;
                motor_halt(&motorData);
            }
            break;
        }

        gFlags.i2cAction = FALSE;
    }
}
