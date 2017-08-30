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

