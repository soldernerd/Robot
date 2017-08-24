
#include <xc.h>
#include <stdint.h>
#include "i2c.h"
#include "common.h"

//State variables
//volatile uint8_t update;
volatile i2c_struct i2c_data;

/******************************************************************************
 * Interrupt Service Routine (ISR)                                            *
 * Read and average pot value via ADC                                         *
 ******************************************************************************/
void i2c_isr(void)
{
  uint8_t tmp;

  if(PIR1bits.SSPIF)
  {
    //Master Read Mode -> Sending data
    if(SSPSTATbits.R_nW)
    {
      //Master did not acknowledge -> Transmission is complete
      if(SSPCON2bits.ACKSTAT)
      {
        i2c_data.data_sent = 1; //Indicate that data has been sent
      }
      //Master is asking for data
      else
      {
        // Address has just been received -> First byte
        if(!SSPSTATbits.D_nA)
        {
          i2c_data.address = SSPBUF & 0b11111110; //Save address
          i2c_data.ptr = &i2c_data.send_buffer[0]; //Reset buffer pointer
          i2c_data.bytes_sent = 0; //Reset byte counter
          i2c_fill_send_buffer();
        }
        // Send data
        if(i2c_data.bytes_sent<I2C_SEND_BUFFER_SIZE)
        {
          SSPBUF = *i2c_data.ptr; //Fill buffer with next byte
          ++i2c_data.bytes_sent; //Increment byte counter
          ++i2c_data.ptr; //Increment pointer
        }
        else
        {
          SSPBUF = 0x00; //No more data available, send zeros
        }
      }
    }
    //Master Transmit Mode -> Receiving data
    else
    {  
      //Stop bit
      if(SSPSTATbits.P)
      {
        i2c_data.data_received = 1; //Indicate that data has been received
        SSPCON3bits.PCIE = 0; //Disable Stop detection interrupts
        
        //DEBUG!!!
        if(i2c_data.receive_buffer[0]==0x50)
            aux1_off();
        if(i2c_data.receive_buffer[0]==0x51)
            aux1_on();
        
        
      }
      //Not stop bit
      else
      {
        // Address, not data
        if(!SSPSTATbits.D_nA)
        {
          i2c_data.address = SSPBUF & 0b11111110; //Save address
          i2c_data.ptr = &i2c_data.receive_buffer[0]; //Reset buffer pointer
          i2c_data.bytes_received = 0; //Reset byte counter
          SSPCON3bits.PCIE = 1; //Enable Stop detection interrupts
        }
        // Data, not address
        else
        {
          if(i2c_data.bytes_received<I2C_RECEIVE_BUFFER_SIZE)
          {
            *i2c_data.ptr = SSPBUF; //Save data
            ++i2c_data.bytes_received; //Increment byte counter
            ++i2c_data.ptr; //Increment pointer
          }
        }
      }
    }
    SSPCON1bits.CKP = 1; //Release SCL line
    PIR1bits.SSPIF = 0; //Clear SSP interrupt flag
    return;
  }
}

/******************************************************************************
 * Initialize I2C module in 7-bit slave mode                                  *
 ******************************************************************************/
void i2c_slave_init(void)
{
  //Initialize i2c_data
  i2c_data.address = 0;
  i2c_data.dummy = 0;
  i2c_data.data_received = 0;
  i2c_data.bytes_received = 0;
  i2c_data.data_sent = 0;
  i2c_data.ptr = &i2c_data.send_buffer[0];
  for(i2c_data.bytes_sent=0; i2c_data.bytes_sent<I2C_SEND_BUFFER_SIZE; ++i2c_data.bytes_sent)
  {
    i2c_data.send_buffer[i2c_data.bytes_sent] = i2c_data.bytes_sent;
  }
  i2c_data.send_buffer[0] = 88;
  i2c_data.bytes_sent = 0;
  for(i2c_data.bytes_received=0; i2c_data.bytes_received<I2C_RECEIVE_BUFFER_SIZE; ++i2c_data.bytes_received)
  {
    i2c_data.receive_buffer[i2c_data.bytes_received] = 0;
  }
  i2c_data.bytes_received = 0;
  
  //Configure SCL1 and SDA1 as inputs
  TRISCbits.TRISC3 = 1; //pin 14, RC3, SCL
  TRISCbits.TRISC4 = 1; //pin 15, RC4, SDA
  
  //Set address and address mask
  SSPADD = I2C_SLAVE_ADDRESS; 
  SSPMSK = I2C_SLAVE_ADDRESS_MASK;
  
  //SSPSTAT: SSP STATUS REGISTER, p286
  SSPSTATbits.SMP = 1; //Disable slew rate control for high speed mode (400kHz), RaspberryPi uses 100kHz
  SSPSTATbits.CKE = 1; //Enable SMBus specific inputs
  SSPSTATbits.BF = 0; //Clear buffer full status bit

  //SSPCON1: SSP CONTROL REGISTER 1, p287
  SSPCON1bits.WCOL = 0; //Clear Write Collision Detect bit
  SSPCON1bits.SSPOV = 0; //Clear Receive Overflow Indicator bit
  SSPCON1bits.SSPEN = 0; //Disable Synchronous Serial Port for now
  SSPCON1bits.CKP = 1; //Release clock (do not hold CLK low)
  SSPCON1bits.SSPM3 = 1; //7-bit I2C Slave Mode with start and stop interrupts
  SSPCON1bits.SSPM2 = 1; //7-bit I2C Slave Mode with start and stop interrupts
  SSPCON1bits.SSPM1 = 1; //7-bit I2C Slave Mode with start and stop interrupts
  SSPCON1bits.SSPM0 = 0; //7-bit I2C Slave Mode with start and stop interrupts
  
  // SSPCON2: SSP CONTROL REGISTER 2, p288
  SSPCON2bits.GCEN = 0; //Disable General Call
  SSPCON2bits.SEN = 1; //Enable Clock Stretching
  
  // SSPCON3: SSP CONTROL REGISTER 3
  SSPCON3bits.PCIE = 0; //Disable Stop detection interrupts
  SSPCON3bits.SCIE = 0; //Disable Start detection interrupts
  SSPCON3bits.BOEN = 0; //Only update buffer when SSPOV is clear
  SSPCON3bits.SDAHT = 1; //Minimum of 300 ns hold time on SDA after the falling edge of SCL
  SSPCON3bits.SBCDE = 0; //Slave bus collision interrupts are disabled
  SSPCON3bits.AHEN = 0; //Disable address holding
  SSPCON3bits.DHEN = 0; //Disable data holding
  
  // Configure Interrupts, p284 */
  PIR1bits.SSPIF = 0; //Clear SSP Interrupt Flag
  PIE1bits.SSPIE = 1; //Enable MSSP interrupts
  PIR2bits.BCLIF = 0; //Clear MSSP Bus Collision Interrupt Flag bit
  PIE2bits.BCLIE = 0; //Disable MSSP Bus Collision Interrupt Enable bit
  INTCONbits.PEIE = 1; //Enable Peripheral Interrupts
  INTCONbits.GIE = 1; //Enable Interrupts
  
  // Turn Serial Port on
  SSPCON1bits.SSPEN = 1; //Enable Synchronous Serial Port
}

/******************************************************************************
 * Fill the I2C send buffer                                                   *
 ******************************************************************************/
void i2c_fill_send_buffer(void)
{
  i2c_data.send_buffer[0] = 0x55; // Status
  
  i2c_data.send_buffer[1] = aux1_is_on(); // Green
  i2c_data.send_buffer[2] = aux2_is_on();; // Red
  i2c_data.send_buffer[3] = aux3_is_on();; // Blue
  i2c_data.send_buffer[4] = aux4_is_on();; // Buzzer
}

uint8_t i2c_data_received(void)
{
    if(i2c_data.data_received)
    {
        i2c_data.data_received = 0;
        return i2c_data.bytes_received; 
    }
    else
    {
        return 0;
    }
}

uint8_t i2c_data_sent(void)
{
    return i2c_data.data_sent;
}

uint8_t* i2c_get_rx_handle(void)
{
    return &i2c_data.receive_buffer;
}

uint8_t* i2c_get_tx_handle(void)
{
    return &i2c_data.send_buffer;
}