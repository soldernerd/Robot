/* 
 * File:   motor.h
 * Author: Luke
 *
 * Created on 31. August 2017, 00:14
 */

#include <xc.h>
#include <stdint.h>

#ifndef MOTOR_H
#define	MOTOR_H

typedef enum
{
    MOTOR_A,
    MOTOR_B
} motor_t;

typedef enum
{
    DIRECTION_FORWARD,
    DIRECTION_BACKWARD
} direction_t;

//Function prototypes

void motor_set_direction(motor_t mot, direction_t dir);
void motor_set_power(uint8_t power);

#endif	/* MOTOR_H */

