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

typedef enum
{
    STEPSIZE_FULL,
    STEPSIZE_HALF,
    STEPSIZE_QUARTER,
    STEPSIZE_SIXTEENTH
} stepsize_t;

typedef enum
{
    SLEEPMODE_MOTORS_OFF,
    SLEEPMODE_MOTORS_ON
} sleepmode_t;

typedef enum
{
    RESET_OFF,
    RESET_ON
} reset_t;

typedef enum
{
    RUNMODE_OFF,
    RUNMODE_ON
} runmode_t;


//Function prototypes

void motor_set_direction(motor_t mot, direction_t dir);
void motor_set_power(uint8_t power);
void motor_set_microstepping(stepsize_t stepsize);
void motor_set_sleep(sleepmode_t mode);
void motor_set_reset(reset_t reset);
void motor_run(motor_t motor, runmode_t mode);

#endif	/* MOTOR_H */

