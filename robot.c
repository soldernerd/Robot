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
  
void interrupt _isr(void)
{
    if(PIR1bits.SSPIF)
    {
        i2c_isr();
    }
    if(PIR1bits.TMR2IF)
    {
        motor_a_isr();
    }
    if(PIR3bits.TMR4IF)
    {
        motor_b_isr();
    }
}

void setup(void)
{
    //Set up 32MHz internal oscillator
    //Pages 74, 83
    OSCCON = 0b11110000;
    
    RESET_TRIS = 0;
    ENABLE_TRIS = 0;
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
    
    DACCON0bits.DACEN = 1;
    DACCON0bits.DACLPS = 0;
    DACCON0bits.DACOE = 1;
    DACCON0bits.DACPSS = 0b00;
    DACCON0bits.DACNSS = 0;
    DACCON1 = 2; //output level

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
    uint16_t tmp;
    uint16_t motors_idle_count;

    setup();
    rx_buffer = i2c_get_rx_handle();
    
    //Set up PWM CCP1
    //PR2 = 0xFF;
    CCP1CONbits.P1M = 0b00;
    CCP1CONbits.DC1B = 0b00;
    CCP1CONbits.CCP1M = 0b1100;
    CCPTMRS0bits.C1TSEL = 0b00; //Use timer 2
    //T2CONbits.T2OUTPS = 0b1001; //Postscaler=10
    T2CONbits.T2CKPS = 0b10; //Pre-Scaler = 16
    T2CONbits.TMR2ON = 1; //Turn timer on
    T2CONbits.TMR2ON =0; //Turn timer off
    //Clear interrupt flag and enable interrupts
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 1;
    
    //Set up PWM CCP2
    //PR4 = 0xFF;
    CCP2CONbits.P2M = 0b00;
    CCP2CONbits.DC2B = 0b00;
    CCP2CONbits.CCP2M = 0b1100;
    CCPTMRS0bits.C2TSEL = 0b01; //Use timer 4
    //T4CONbits.T4OUTPS = 0b1111; //Postscaler=10
    T4CONbits.T4CKPS = 0b11; //Pre-Scaler = 16
    T4CONbits.TMR4ON = 1; //Turn timer on
    T4CONbits.TMR4ON = 0; //Turn timer off
    //Clear interrupt flag and enable interrupts
    PIR3bits.TMR4IF = 0;
    PIE3bits.TMR4IE = 1;
    
    motor_set_power(3);
    motor_set_direction(MOTOR_A, DIRECTION_FORWARD);
    motor_set_direction(MOTOR_B, DIRECTION_FORWARD);
    motor_set_enable(ENABLE_HIGH);
    motor_set_sleep(SLEEPMODE_HIGH);
    motor_set_reset(RESET_HIGH);
    motor_set_microstepping(STEPSIZE_SIXTEENTH);
    motor_set_speed(MOTOR_A, SPEED_1);
    motor_set_speed(MOTOR_B, SPEED_1);
    
    while(1)
    {
        __delay_ms(1);
        
        //Check if motors should be disabled
        if((!T2CONbits.TMR2ON) || (!T4CONbits.TMR4ON))
        {
            ++motors_idle_count;
            if(motors_idle_count==250)
            {
               motor_set_enable(ENABLE_HIGH); 
            }
        }
        else
        {
            motors_idle_count = 0;
        }
         
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
                case I2C_COMMAND_ENABLE_LOW:
                    motor_set_enable(ENABLE_LOW);
                    break;
                case I2C_COMMAND_ENABLE_HIGH:
                    motor_set_enable(ENABLE_HIGH);
                    break;    
                case I2C_COMMAND_RESET_LOW:
                    motor_set_reset(RESET_LOW);
                    break;
                case I2C_COMMAND_RESET_HIGH:
                    motor_set_reset(RESET_HIGH);
                    break;
                case I2C_COMMAND_SLEEP_LOW:
                    motor_set_sleep(SLEEPMODE_LOW);
                    break;
                case I2C_COMMAND_SLEEP_HIGH:
                    motor_set_sleep(SLEEPMODE_HIGH);
                    break;
                case I2C_COMMAND_MOTOR_A_OFF:
                    motor_run(MOTOR_A, RUNMODE_OFF);
                    break;
                case I2C_COMMAND_MOTOR_A_ON:
                    motor_run(MOTOR_A, RUNMODE_ON);
                    break;
                case I2C_COMMAND_MOTOR_B_OFF:
                    motor_run(MOTOR_B, RUNMODE_OFF);
                    break;
                case I2C_COMMAND_MOTOR_B_ON:
                    motor_run(MOTOR_B, RUNMODE_ON);
                    break;
                case I2C_COMMAND_MOTOR_A_FORWARD:
                    motor_set_direction(MOTOR_A, DIRECTION_FORWARD);
                    break;    
                case I2C_COMMAND_MOTOR_A_BACKWARD:
                    motor_set_direction(MOTOR_A, DIRECTION_BACKWARD);
                    break;
                case I2C_COMMAND_MOTOR_B_FORWARD:
                    motor_set_direction(MOTOR_B, DIRECTION_FORWARD);
                    break;     
                case I2C_COMMAND_MOTOR_B_BACKWARD:
                    motor_set_direction(MOTOR_B, DIRECTION_BACKWARD);
                    break;
                case I2C_COMMAND_MICROSTEP_FULL:
                    motor_set_microstepping(STEPSIZE_FULL);
                    break;
                case I2C_COMMAND_MICROSTEP_HALF:
                    motor_set_microstepping(STEPSIZE_HALF);
                    break;
                case I2C_COMMAND_MICROSTEP_QUARTER:
                    motor_set_microstepping(STEPSIZE_QUARTER);
                    break;
                case I2C_COMMAND_MICROSTEP_SIXTEENTH:
                    motor_set_microstepping(STEPSIZE_SIXTEENTH);
                    break;
                case I2C_COMMAND_SPEED_A:
                    motor_set_speed(MOTOR_A, (speed_t) rx_buffer[2]);
                    break;
                case I2C_COMMAND_SPEED_B:
                    motor_set_speed(MOTOR_B, (speed_t) rx_buffer[2]);
                    break;
                case I2C_COMMAND_POWER:
                    motor_set_power(rx_buffer[2]);
                    break;
                case I2C_COMMAND_DRIVE:
                    tmp = rx_buffer[2];
                    tmp <<= 8;
                    tmp |= rx_buffer[3];
                    motor_set_number_of_steps(MOTOR_A, tmp);
                    tmp = rx_buffer[4];
                    tmp <<= 8;
                    tmp |= rx_buffer[5];
                    motor_set_number_of_steps(MOTOR_B, tmp);
                    motor_run(MOTOR_A, RUNMODE_ON);
                    motor_run(MOTOR_B, RUNMODE_ON);
                    break;
            }
        }
    }
    return;
}
