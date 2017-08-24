/*
 * File:   common.c
 * Author: luke
 *
 * Created on 24. August 2017, 23:38
 */

#include <xc.h>
#include <stdint.h>
#include "common.h"


void aux1_on(void)
{
    AUX1_VARIABLE |= AUX1_MASK;
    AUX1_PORT = AUX1_VARIABLE;
}

void aux1_off(void)
{
    AUX1_VARIABLE &= ~AUX1_MASK;
    AUX1_PORT = AUX1_VARIABLE;
}

uint8_t aux1_is_on(void)
{
    return AUX1_VARIABLE & AUX1_MASK;
}

void aux2_on(void)
{
    AUX2_VARIABLE |= AUX2_MASK;
    AUX2_PORT = AUX2_VARIABLE;
}

void aux2_off(void)
{
    AUX2_VARIABLE &= ~AUX2_MASK;
    AUX2_PORT = AUX2_VARIABLE;
}

uint8_t aux2_is_on(void)
{
    return AUX2_VARIABLE & AUX2_MASK;
}

void aux3_on(void)
{
    AUX3_VARIABLE |= AUX3_MASK;
    AUX3_PORT = AUX3_VARIABLE;
}

void aux3_off(void)
{
    AUX3_VARIABLE &= ~AUX3_MASK;
    AUX3_PORT = AUX3_VARIABLE;
}

uint8_t aux3_is_on(void)
{
    return AUX3_VARIABLE & AUX3_MASK;
}

void aux4_on(void)
{
    AUX4_VARIABLE |= AUX4_MASK;
    AUX4_PORT = AUX4_VARIABLE;
}

uint8_t aux4_is_on(void)
{
    return AUX4_VARIABLE & AUX4_MASK;
}

void aux4_off(void)
{
    AUX4_VARIABLE &= ~AUX4_MASK;
    AUX4_PORT = AUX4_VARIABLE;
}
