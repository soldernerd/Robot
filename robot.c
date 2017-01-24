/*
 * File:   robot.c
 * Author: Luke
 *
 * Created on 22. Januar 2017, 16:57
 */

#include <xc.h>
#include <stdint.h>

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = RA5     // Voltage Regulator Capacitor Enable (VCAP functionality is enabled on RA5)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

#define _XTAL_FREQ 1000000

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


//Global variables
uint8_t portA = 0x00;
uint8_t portB = 0x00;
uint8_t portC = 0x00;


void setup(void)
{
    RESET_TRIS = 0;
    SLEEP_TRIS = 0;
    REF_TRIS = 0;
    MS2_TRIS = 0;
    MS1_TRIS = 0;
    
    DIR_A_TRIS = 0;
    STEP_A_TRIS = 0;
    DIR_B_TRIS = 0;
    STEP_B_TRIS = 0;

    AUX1_TRIS = 0;
    AUX2_TRIS = 0;
    AUX3_TRIS = 0;
    AUX4_TRIS = 0;
    
    VSENSE_TRIS = 1;
    SCL_TRIS = 1;
    SDA_TRIS = 1;
    
    //Full Step Mode
    MS1_VARIABLE &= ~MS1_MASK;
    MS1_PORT = MS1_VARIABLE;
    MS2_VARIABLE &= ~MS2_MASK;
    MS2_PORT = MS2_VARIABLE;
    
    //Quarter Step Mode
    MS1_VARIABLE &= ~MS1_MASK;
    MS1_PORT = MS1_VARIABLE;
    MS2_VARIABLE |= MS2_MASK;
    MS2_PORT = MS2_VARIABLE;
    
    RESET_VARIABLE |= RESET_MASK;
    RESET_PORT = RESET_VARIABLE;
    SLEEP_VARIABLE |= SLEEP_MASK;
    SLEEP_PORT = SLEEP_VARIABLE;
    
    DACCON0bits.DACEN = 1;
    DACCON0bits.DACLPS = 0;
    DACCON0bits.DACOE = 1;
    DACCON0bits.DACPSS = 0b00;
    DACCON0bits.DACNSS = 0;
    DACCON1 = 1; //output level
    
    
    //REF_VARIABLE |= REF_MASK;
    //REF_PORT = REF_VARIABLE;
}

void main(void)
{
    uint16_t stepcount = 0;
    setup();
    
    while(1)
    {
        STEP_A_VARIABLE ^= STEP_A_MASK;
        STEP_A_PORT = STEP_A_VARIABLE;
        STEP_B_VARIABLE ^= STEP_B_MASK;
        STEP_B_PORT = STEP_B_VARIABLE;
        __delay_ms(1);
        ++stepcount;
        if(stepcount==1600)
        {
            stepcount=0;
            DIR_A_VARIABLE ^= DIR_A_MASK;
            DIR_A_PORT = DIR_A_VARIABLE;
            DIR_B_VARIABLE ^= DIR_B_MASK;
            DIR_B_PORT = DIR_B_VARIABLE;
        }
    }
    
    return;
}
