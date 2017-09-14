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
    ENABLE_LOW,
    ENABLE_HIGH
} enable_t;

typedef enum
{
    SLEEPMODE_LOW,
    SLEEPMODE_HIGH
} sleepmode_t;

typedef enum
{
    RESET_LOW,
    RESET_HIGH
} reset_t;

typedef enum
{
    RUNMODE_OFF,
    RUNMODE_ON
} runmode_t;

typedef enum
{
    SPEED_1 = 1,
    SPEED_2 = 2,
    SPEED_3 = 3,
    SPEED_4 = 4,
    SPEED_5 = 5
} speed_t;

//Function prototypes
void motor_a_isr(void);
void motor_b_isr(void);
void motor_set_direction(motor_t mot, direction_t dir);
void motor_set_power(uint8_t power);
void motor_set_microstepping(stepsize_t stepsize);
void motor_set_enable(enable_t mode);
void motor_set_sleep(sleepmode_t mode);
void motor_set_reset(reset_t reset);
void motor_set_speed(motor_t motor, speed_t speed);
void motor_run(motor_t motor, runmode_t mode);
void motor_set_number_of_steps(motor_t motor, uint16_t steps);

#endif	/* MOTOR_H */

