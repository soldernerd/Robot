/* 
 * File:   i2c.h
 * Author: Luke
 *
 * Created on 4. März 2017, 14:29
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

#define I2C_COMMAND_GREEN_OFF 0x50
#define I2C_COMMAND_GREEN_ON 0x51
#define I2C_COMMAND_RED_OFF 0x52
#define I2C_COMMAND_RED_ON 0x53
#define I2C_COMMAND_BLUE_OFF 0x54
#define I2C_COMMAND_BLUE_ON 0x55
#define I2C_COMMAND_BUZZER_OFF 0x56
#define I2C_COMMAND_BUZZER_ON 0x57


/******************************************************************************
 * Global variables                                                           *
 ******************************************************************************/
//volatile uint8_t white;
//volatile uint8_t off;
//volatile uint8_t brightness;
//volatile uint8_t color;

//uint16_t red;
//uint16_t green;
//uint16_t blue;

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

