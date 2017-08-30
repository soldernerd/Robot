/*
 * File:   robot.c
 * Author: Luke
 *
 * Created on 22. Januar 2017, 16:57
 */

#include <xc.h>
#include <stdint.h>
#include "common.h"
#include "motor.h"
#include "i2c.h"

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


//Global variables
volatile uint8_t portA = 0x00;
volatile uint8_t portB = 0x00;
volatile uint8_t portC = 0x00;

static void reset_on();
static void reset_off();
static void motors_on();
static void motors_off();
  
void interrupt _isr(void)
{
    i2c_isr();
}

static void motors_on(void)
{
    SLEEP_VARIABLE &= ~SLEEP_MASK;
    SLEEP_PORT = SLEEP_VARIABLE;
}

static void motors_off(void)
{
    SLEEP_VARIABLE |= SLEEP_MASK;
    SLEEP_PORT = SLEEP_VARIABLE;
}

static void reset_on(void)
{
    RESET_VARIABLE &= ~RESET_MASK;
    RESET_PORT = RESET_VARIABLE;
}

static void reset_off(void)
{
    RESET_VARIABLE |= RESET_MASK;
    RESET_PORT = RESET_VARIABLE;
}


void setup(void)
{
  //Set up 32MHz internal oscillator
  //Pages 74, 83
  OSCCON = 0b11110000;
  
  /*
  //Configure interrupts
  IOCBP = ENCODER1_PB | ENCODER2_PB; //Interrupt-on-Change on positive edge
  IOCBN = ENCODER1_A | ENCODER1_B | ENCODER2_A | ENCODER2_B; //Interrupt-on-Change on negative edge
  INTCON &= (~INTERRUPT_ON_CHANGE_FLAG); //Clear interrupt flag prior to enable
  IOCBF = 0; // Clear interrupt-on-change flag
  INTCON |= INTERRUPT_ON_CHANGE_ENABLE; // Interrupt on change enable bit
  INTCON |= GLOBAL_INTERRUPT_ENABLE; // Global interrupt enable bit
  */
    
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
    //SLEEP_VARIABLE |= SLEEP_MASK;
    //SLEEP_PORT = SLEEP_VARIABLE;
    
    DACCON0bits.DACEN = 1;
    DACCON0bits.DACLPS = 0;
    DACCON0bits.DACOE = 1;
    DACCON0bits.DACPSS = 0b00;
    DACCON0bits.DACNSS = 0;
    DACCON1 = 2; //output level
    
    motors_on();
    //reset_on();
    REF_VARIABLE |= REF_MASK;
    REF_PORT = REF_VARIABLE;

    //Green
    aux1_off();
    //Red
    aux2_off();
    //Blue
    aux3_off();
    //Buzzer
    aux4_off();
    
    //Enable I2C
    i2c_slave_init();
}

void main(void)
{
    uint8_t bytes_received;
    uint8_t* rx_buffer;
    uint8_t cntr;
    uint16_t stepcount = 0;
    uint16_t ledcount = 0;
    setup();
    rx_buffer = i2c_get_rx_handle();
    
    motor_set_power(5);
    motor_set_direction(MOTOR_A, DIRECTION_FORWARD);
    motor_set_direction(MOTOR_B, DIRECTION_FORWARD);
    
    while(1)
    {
        //__delay_ms(1);
        
        STEP_A_VARIABLE |= STEP_A_MASK;
        STEP_A_PORT = STEP_A_VARIABLE;
        STEP_B_VARIABLE |= STEP_B_MASK;
        STEP_B_PORT = STEP_B_VARIABLE;
        __delay_ms(2);
        STEP_A_VARIABLE &= ~STEP_A_MASK;
        STEP_A_PORT = STEP_A_VARIABLE;
        STEP_B_VARIABLE &= ~STEP_B_MASK;
        STEP_B_PORT = STEP_B_VARIABLE;
        __delay_ms(2);
        
        bytes_received = i2c_data_received();
        if(bytes_received)
        {
            switch(rx_buffer[1])
            {
                case I2C_COMMAND_GREEN_OFF:
                    aux1_off();
                    break;
                case I2C_COMMAND_GREEN_ON:
                    aux1_on();
                    break;
                case I2C_COMMAND_RED_OFF:
                    aux2_off();
                    break;
                case I2C_COMMAND_RED_ON:
                    aux2_on();
                    break;
                case I2C_COMMAND_BLUE_OFF:
                    aux3_off();
                    break;
                case I2C_COMMAND_BLUE_ON:
                    aux3_on();
                    break;
                case I2C_COMMAND_BUZZER_OFF:
                    aux4_off();
                    break;
                case I2C_COMMAND_BUZZER_ON:
                    aux4_on();
                    break;
            }
        }
    }
    return;
}
