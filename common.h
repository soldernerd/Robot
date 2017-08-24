/* 
 * File:   common.h
 * Author: Luke
 *
 * Created on 24. August 2017, 23:38
 */

#ifndef COMMON_H
#define	COMMON_H

//Definitions
#define _XTAL_FREQ 32000000

#define RESET_TRIS TRISCbits.TRISC5
#define RESET_MASK 0b00100000
#define RESET_PORT PORTC
#define RESET_VARIABLE portC

#define SLEEP_TRIS TRISAbits.TRISA1
#define SLEEP_MASK 0b00000010
#define SLEEP_PORT PORTA
#define SLEEP_VARIABLE portA

#define REF_TRIS TRISAbits.TRISA2
#define REF_MASK 0b00000100
#define REF_PORT PORTA
#define REF_VARIABLE portA

#define MS1_TRIS TRISCbits.TRISC7
#define MS1_MASK 0b10000000
#define MS1_PORT PORTC
#define MS1_VARIABLE portC

#define MS2_TRIS TRISCbits.TRISC6
#define MS2_MASK 0b01000000
#define MS2_PORT PORTC
#define MS2_VARIABLE portC

#define DIR_A_TRIS TRISCbits.TRISC0
#define DIR_A_MASK 0b00000001
#define DIR_A_PORT PORTC
#define DIR_A_VARIABLE portC

#define STEP_A_TRIS TRISCbits.TRISC2
#define STEP_A_MASK 0b00000100
#define STEP_A_PORT PORTC
#define STEP_A_VARIABLE portC

#define DIR_B_TRIS TRISAbits.TRISA6
#define DIR_B_MASK 0b01000000
#define DIR_B_PORT PORTA
#define DIR_B_VARIABLE portA

#define STEP_B_TRIS TRISCbits.TRISC1
#define STEP_B_MASK 0b00000010
#define STEP_B_PORT PORTC
#define STEP_B_VARIABLE portC

#define AUX1_TRIS TRISAbits.TRISA4
#define AUX1_MASK 0b00010000
#define AUX1_PORT PORTA
#define AUX1_VARIABLE portA

#define AUX2_TRIS TRISBbits.TRISB5
#define AUX2_MASK 0b00100000
#define AUX2_PORT PORTB
#define AUX2_VARIABLE portB

#define AUX3_TRIS TRISBbits.TRISB4
#define AUX3_MASK 0b00010000
#define AUX3_PORT PORTB
#define AUX3_VARIABLE portB

#define AUX4_TRIS TRISBbits.TRISB0
#define AUX4_MASK 0b00000001
#define AUX4_PORT PORTB
#define AUX4_VARIABLE portB

#define SCL_TRIS TRISCbits.TRISC3
#define SDA_TRIS TRISCbits.TRISC4
#define VSENSE_TRIS TRISBbits.TRISB3

//Global variables defined in robot.c
extern volatile uint8_t portA;
extern volatile uint8_t portB;
extern volatile uint8_t portC;


//Function prototypes

void aux1_on(void);
void aux1_off(void);
uint8_t aux1_is_on(void);
void aux2_on(void);
void aux2_off(void);
uint8_t aux2_is_on(void);
void aux3_on(void);
void aux3_off(void);
uint8_t aux3_is_on(void);
void aux4_on(void);
uint8_t aux4_is_on(void);
void aux4_off(void);

#endif	/* COMMON_H */

