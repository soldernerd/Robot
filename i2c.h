/* 
 * File:   i2c.h
 * Author: Luke
 *
 * Created on 4. M�rz 2017, 14:29
 */

#ifndef I2C_H
#define	I2C_H

//Slave address and mask
#define I2C_SLAVE_ADDRESS 0b11100000 /*0x70*/
#define I2C_SLAVE_ADDRESS_MASK 0b11111111 /*Do not mask any bits*/

//Struct
#define I2C_SEND_BUFFER_SIZE 16
#define I2C_RECEIVE_BUFFER_SIZE 16
typedef struct i2c_
{
  volatile uint8_t address;
  volatile uint8_t dummy;
  volatile uint8_t data_received;
  volatile uint8_t bytes_received;
  volatile uint8_t data_sent;
  volatile uint8_t bytes_sent;
  volatile uint8_t *ptr;
  uint8_t send_buffer[I2C_SEND_BUFFER_SIZE];
  uint8_t receive_buffer[I2C_RECEIVE_BUFFER_SIZE];
} i2c_struct;

/* SEND BUFFER (i.e Master Read Data)
 * Byte 0: Status
 * Byte 1: White
 * Byte 2: Off
 * Byte 3: Brightness
 * Byte 4: Color
 * Bytes 5-6: Red
 * Bytes 7-8: Green
 * Bytes 9-10: Blue
 */


/******************************************************************************
 * Constants                                                                  *
 ******************************************************************************/
#define GLOBAL_INTERRUPT_ENABLE 0b10000000
#define INTERRUPT_ON_CHANGE_ENABLE 0b00001000
#define INTERRUPT_ON_CHANGE_FLAG 0b00000001

#define ENCODER1_A 0b00010000
#define ENCODER1_B 0b00100000
#define ENCODER1_PB 0b00001000
#define ENCODER2_A 0b00000010
#define ENCODER2_B 0b00000100
#define ENCODER2_PB 0b00000001

#define NUMBER_OF_BRIGHTNESS_LEVELS 32
#define NUMBER_OF_COLORS 24

#define RGB_NEUTRAL_RED 255
#define RGB_NEUTRAL_GREEN 90
#define RGB_NEUTRAL_BLUE 50

typedef enum
{
    //Aux outputs
    I2C_COMMAND_GREEN_OFF = 0x50,
    I2C_COMMAND_GREEN_ON = 0x51,
    I2C_COMMAND_RED_OFF = 0x52,
    I2C_COMMAND_RED_ON = 0x53,
    I2C_COMMAND_BLUE_OFF = 0x54,
    I2C_COMMAND_BLUE_ON = 0x55,
    I2C_COMMAND_BUZZER_OFF = 0x56,
    I2C_COMMAND_BUZZER_ON = 0x57,
    //Motor control
    I2C_COMMAND_ENABLE_LOW = 0x58,   
    I2C_COMMAND_ENABLE_HIGH = 0x59,         
    I2C_COMMAND_RESET_LOW = 0x60,   
    I2C_COMMAND_RESET_HIGH = 0x61, 
    I2C_COMMAND_SLEEP_LOW = 0x62, 
    I2C_COMMAND_SLEEP_HIGH = 0x63,
    I2C_COMMAND_MOTOR_A_OFF = 0x64,   
    I2C_COMMAND_MOTOR_A_ON = 0x65, 
    I2C_COMMAND_MOTOR_B_OFF = 0x66, 
    I2C_COMMAND_MOTOR_B_ON = 0x67,
    I2C_COMMAND_MICROSTEP_FULL = 0x68,
    I2C_COMMAND_MICROSTEP_HALF = 0x69, 
    I2C_COMMAND_MICROSTEP_QUARTER = 0x6A, 
    I2C_COMMAND_MICROSTEP_SIXTEENTH = 0x6B,
    I2C_COMMAND_MOTOR_A_FORWARD = 0x6C,      
    I2C_COMMAND_MOTOR_A_BACKWARD = 0x6D,
    I2C_COMMAND_MOTOR_B_FORWARD = 0x6E,      
    I2C_COMMAND_MOTOR_B_BACKWARD = 0x6F, 
    //2 byte commands
    I2C_COMMAND_SPEED_A = 0x10,
    I2C_COMMAND_SPEED_B = 0x11,
    I2C_COMMAND_POWER = 0x12,
    //5 byte command
    I2C_COMMAND_DRIVE = 0x22        
} i2c_commands_t;


/******************************************************************************
 * Function prototype                                                         *
 ******************************************************************************/
void i2c_isr(void);
void i2c_slave_init(void);
void i2c_fill_send_buffer(void);
uint8_t i2c_data_received(void);
uint8_t i2c_data_sent(void);
uint8_t* i2c_get_rx_handle(void);
uint8_t* i2c_get_tx_handle(void);



#endif	/* I2C_H */

