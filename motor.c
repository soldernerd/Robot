/*
 * File:   motor.c
 * Author: luke
 *
 * Created on 31. August 2017, 00:14
 */


#include <xc.h>
#include "motor.h"
#include "common.h"
    
void motor_set_direction(motor_t mot, direction_t dir)
{
    if(mot==MOTOR_A)
    {
        if(dir==DIRECTION_FORWARD)
        {
            DIR_A_VARIABLE &= ~DIR_A_MASK;
        }
        if(dir==DIRECTION_BACKWARD)
        {
            DIR_A_VARIABLE |= DIR_A_MASK;
        }
        DIR_A_PORT = DIR_A_VARIABLE;
    }
    if(mot==MOTOR_B)
    {
        if(dir==DIRECTION_FORWARD)
        {
            DIR_B_VARIABLE |= DIR_B_MASK; 
        }
        if(dir==DIRECTION_BACKWARD)
        {
            DIR_B_VARIABLE &= ~DIR_B_MASK; 
        }
        DIR_B_PORT = DIR_B_VARIABLE;
    }
}

void motor_set_power(uint8_t power)
{
    DACCON1 = power & 0b00011111;
}

void motor_set_microstepping(stepsize_t stepsize)
{
    switch(stepsize)
    {
        case STEPSIZE_FULL:
            MS1_VARIABLE &= ~MS1_MASK;
            MS2_VARIABLE &= ~MS2_MASK; 
            break;
        case STEPSIZE_HALF:
            MS1_VARIABLE |= MS1_MASK;
            MS2_VARIABLE &= ~MS2_MASK; 
            break;
        case STEPSIZE_QUARTER:
            MS1_VARIABLE &= ~MS1_MASK;
            MS2_VARIABLE |= MS2_MASK; 
            break;
        case STEPSIZE_SIXTEENTH:
            MS1_VARIABLE &= ~MS1_MASK;
            MS2_VARIABLE &= ~MS2_MASK; 
            break;
    }
    MS1_PORT = MS1_VARIABLE;
    MS2_PORT = MS2_VARIABLE;
}

void motor_set_sleep(sleepmode_t mode)
{
    switch(mode)
    {
        case SLEEPMODE_MOTORS_OFF:
            SLEEP_VARIABLE |= SLEEP_MASK; 
            break;
        case SLEEPMODE_MOTORS_ON:
            SLEEP_VARIABLE &= ~SLEEP_MASK; 
            break;
    }
    SLEEP_PORT = SLEEP_VARIABLE;
}

void motor_set_reset(reset_t reset)
{
    switch(reset)
    {
        case RESET_OFF:
            RESET_VARIABLE |= RESET_MASK;
            break;
        case RESET_ON:
            RESET_VARIABLE &= ~RESET_MASK;
            break;
    }
    RESET_PORT = RESET_VARIABLE;   
}

void motor_run(motor_t motor, runmode_t mode)
{
    switch(motor)
    {
        case MOTOR_A:
            switch(mode)
            {
                case RUNMODE_OFF:
                    T2CONbits.TMR2ON = 0; //Turn timer off
                    break;
                case RUNMODE_ON:
                    T2CONbits.TMR2ON = 1; //Turn timer on
                    break;
            }
            break;
        case MOTOR_B:
            switch(mode)
            {
                case RUNMODE_OFF:
                    T4CONbits.TMR4ON = 0; //Turn timer off
                    break;
                case RUNMODE_ON:
                    T4CONbits.TMR4ON = 1; //Turn timer on
                    break;
            }
            break;
    }
}