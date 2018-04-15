#ifndef MOTOR_ATTINY_H
#define MOTOR_ATTINY_H

#include <stdint.h>
#include <wiring.h>

#ifndef ANALOG_WRITE_MIN
#define ANALOG_WRITE_MIN 0
#endif

#ifndef ANALOG_WRITE_MAX
#define ANALOG_WRITE_MAX 255
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef struct MOTOR_DATA
{
	// the up control pin
	uint8_t upPin;
	// the down control pin
	uint8_t downPin;
	// cache up value
	uint8_t upValue;
	// cache down value
	uint8_t downValue;
} motorConfig_t;

void motor_Init(uint8_t upPin, uint8_t downPin, struct MOTOR_DATA *data);
void motor_up(uint8_t speed, struct MOTOR_DATA *data);
void motor_down(uint8_t speed, struct MOTOR_DATA *data);
void motor_set_upPin(uint8_t value, struct MOTOR_DATA *data, bool force = FALSE);
void motor_set_downPin(uint8_t value, struct MOTOR_DATA *data, bool force = FALSE);
void motor_stop(struct MOTOR_DATA *data);
void motor_halt(struct MOTOR_DATA *data);

#endif
