/*****************************************************************************
*
* File              : i2cMultiMaster.c
* Compiler          : avr gcc
* Description       : This is a sample driver for the TWI hardware modules.
*                     It is interrupt driven. All functionality is controlled through
*                     passing information to and from functions.
*
****************************************************************************/

#include <stdio.h>
#include <stdbool.h>

#include <avr/interrupt.h>
#include <compat/twi.h>

/* Scheduler include files. */
#include "FreeRTOS.h"

#include "i2cMultiMaster.h"

/* Create a Semaphore binary flag for the i2c Bus. To ensure only single access. */
SemaphoreHandle_t xI2CSemaphore = NULL;

static uint8_t I2C_buf[ I2C_BUFFER_SIZE ];    // Transceiver buffer
static uint8_t I2C_msgSize;                   // Number of bytes to be transmitted.
static uint8_t I2C_state = I2C_NO_STATE;      // State byte. Default set to I2C_NO_STATE.

static uint8_t I2C_checkBusyAfterStop = 0; 	  // Number of busy check times following a Stop_Restart xA0

union I2C_statusReg I2C_statusReg = {0};      // I2C_statusReg is defined in i2cMultiMaster.h

/* Private Functions */

/****************************************************************************
 * Call this function to set up the TWI slave to its initial standby state.
 * Remember to enable interrupts from the main application after initialising the TWI.
 * Pass both the slave address and the requirements for triggering on a general call in the
 * same byte. Use e.g. this notation when calling this function:
 * I2C_Slave_Initialise( (I2C_slaveAddress<<I2C_ADR_BITS) | (TRUE<<I2C_GEN_BIT) );
 * The TWI module is configured to NACK on any requests. Use a I2C_Slave_Start_Transceiver
 * function to start the TWI.
 *****************************************************************************/
void I2C_Slave_Initialise( uint8_t I2C_ownAddress )
{
	// The Semaphore has to be created to allow the I2C bus to be shared.
	//Assuming the I2C bus will be shared.
	// Use this semaphore  (take, give) when calling I2C functions, and it can ensure single access.
    if( xI2CSemaphore == NULL ) 					// Check to see if the semaphore has not been created.
    {
		xI2CSemaphore = xSemaphoreCreateMutex();	// mutex semaphore for I2C bus
		if( ( xI2CSemaphore ) != NULL )
			xSemaphoreGive( ( xI2CSemaphore ) );	// make the I2C available
    }

	I2C_PORT_DIR &= ~(I2C_BIT_SCL | I2C_BIT_SDA);	// set the I2C SDA & SCL inputs.

	TWAR = I2C_ownAddress;                       	// Set own TWI slave address.  Accept TWI General Calls if ODD address.
	TWDR = 0xff;                                  	// Default content = SDA released.
	TWCR = (1<<TWEN)|                            	// Enable TWI-interface and release TWI pins.
		   (0<<TWIE)|(0<<TWINT)|                   	// Disable TWI Interrupt.
		   (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|        	// Do not ACK on any requests, yet.
		   (0<<TWWC);
}


/****************************************************************************
Call this function to set up the TWI master to its initial standby state.
Remember to enable interrupts from the main application after initialising the TWI.
****************************************************************************/
void I2C_Master_Initialise( uint8_t I2C_ownAddress )
{
	// The Semaphore has to be created to allow the I2C bus to be shared.
	// Assuming the I2C bus will be shared.
	// Use this semaphore  (take, give) when calling I2C functions, and it can ensure single access.
    if( xI2CSemaphore == NULL ) 					// Check to see if the semaphore has not been created.
    {
		xI2CSemaphore = xSemaphoreCreateMutex();	// mutex semaphore for I2C bus
		if( ( xI2CSemaphore ) != NULL )
			xSemaphoreGive( ( xI2CSemaphore ) );	// make the I2C bus available
    }


	I2C_PORT_DIR &= ~(I2C_BIT_SCL | I2C_BIT_SDA);	// set the I2C SDA & SCL inputs.
													// Pull up resistors
	I2C_PORT |= (I2C_BIT_SCL | I2C_BIT_SDA);		// only need these set at one place, usually Master.

													// Initialise TWI clock
    TWSR = 0;                         				// no prescaler
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  				// must be > 10 for stable operation

	TWAR = I2C_ownAddress;                     		// Set own TWI slave address, in case it is called.
													// Accept TWI General Calls if ODD address.

	TWDR = 0xff;                               		// Default content = SDA released.
	TWCR = (1<<TWEN)|                           	// Enable TWI-interface and release TWI pins.
		   (0<<TWIE)|(0<<TWINT)|                	// Disable Interrupt.
		   (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|     	// No Signal requests.
		   (0<<TWWC);
}

/****************************************************************************
 * Call this function to start the Transceiver without specifying new transmission data.
 * Useful for restarting a transmission, or just starting the transceiver for reception.
 * The driver will reuse the data previously put in the transceiver buffers. The function will
 * hold execution (loop) until the I2C_ISR has completed with the  previous operation, then
 * Initialise the next operation and return.
****************************************************************************/
void I2C_Slave_Start_Transceiver(void)
{
  while ( I2C_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.
  I2C_statusReg.all = 0;
  I2C_state         = I2C_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
         (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Prepare to ACK next time the Slave is addressed.
         (0<<TWWC);                             //
}

/****************************************************************************
Call this function to re-send the last message. The driver will reuse the data previously
put in the transceiver buffers. The function will hold execution (loop) until the I2C_ISR
has completed with the previous operation, then initialise the next operation and return.
****************************************************************************/
void I2C_Master_Start_Transceiver(void)
{
  while ( I2C_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.
  I2C_statusReg.all = 0;
  I2C_state         = I2C_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
         (1<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                             //
}


/*****************************************************************************
 * Call this function to send a prepared message, or start the Transceiver for reception. Include
 * a pointer to the data to be sent if a SLA+W is received. The data will be copied to the TWI
 * buffer.  Also include how many bytes that should be sent. Note that unlike the similar Master
 * function, the Address byte is not included in the message buffers.
 * The function will hold execution (loop) until the I2C_ISR has completed with the previous operation,
 * then initialize the next operation and return.
 ******************************************************************************/
void I2C_Slave_Start_Transceiver_With_Data( uint8_t *msg, uint8_t msgSize )
{

    while ( I2C_Transceiver_Busy() );	// Wait until TWI is ready for next transmission.

    I2C_msgSize = msgSize;				// Number of bytes to transmit.

    // Copy data that may be transmitted if the TWI Master requests data.
    for ( uint8_t i = 0; i < msgSize; i++ )
        I2C_buf[ i ] = msg[ i ];

    I2C_statusReg.all = 0;
    I2C_state         = I2C_NO_STATE ;
    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
           (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Prepare to ACK next time the Slave is addressed.
           (0<<TWWC);                             //
}

/****************************************************************************
Call this function to send a prepared message. The first byte must contain the slave address and the
read/write bit. Consecutive bytes contain the data to be sent, or empty locations for data to be read
from the slave. Also include how many bytes that should be sent/read including the address byte.
The function will hold execution (loop) until the I2C_ISR has completed with the previous operation,
then initialise the next operation and return.
****************************************************************************/
void I2C_Master_Start_Transceiver_With_Data( uint8_t *msg, uint8_t msgSize )
{
	while ( I2C_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.

	I2C_msgSize = msgSize;                        // Number of data to transmit.
	I2C_buf[0]  = msg[0];                         // Store slave address with R/W setting.

	if (( msg[0] & (true<<I2C_READ_BIT) ) == false)       // If it is a write operation, then also copy data.
		for ( uint8_t i = 1; i < msgSize; i++ )
			I2C_buf[ i ] = msg[ i ];


	I2C_statusReg.all = 0;
	I2C_state         = I2C_NO_STATE ;
	TWCR = (1<<TWEN)|                           // TWI Interface enabled.
		   (1<<TWIE)|(1<<TWINT)|                // Enable TWI Interrupt and clear the flag.
		   (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|     // Initiate a START condition.
		   (0<<TWWC);                           //
}

/****************************************************************************
 * Call this function to read out the received data from the TWI transceiver buffer. I.e. first
 * call I2C_Start_Transceiver to get the TWI Transceiver to fetch data. Then Run this function to
 * collect the data when they have arrived. Include a pointer to where to place the data and
 * the number of bytes to fetch in the function call. The function will hold execution (loop)
 * until the I2C_ISR has completed with the previous operation, before reading out the data
 * and returning. If there was an error in the previous transmission the function will return
 * the TWI State code.
 *****************************************************************************/
uint8_t I2C_Slave_Get_Data_From_Transceiver( uint8_t *msg, uint8_t msgSize )
{

    while ( I2C_Transceiver_Busy() );    // Wait until TWI finished with the transmission.

    if( I2C_statusReg.lastTransOK )    	// Last transmission completed successfully.
    {
        for ( uint8_t i=0; i<msgSize; i++ )        // Copy data from Transceiver buffer.
            msg[i] = I2C_buf[i];

        I2C_statusReg.RxDataInBuf = false;        // Slave Receive data has been read from buffer.
    }
    return I2C_statusReg.lastTransOK;
}

/****************************************************************************
Call this function to read out the requested data from the TWI transceiver buffer. I.e. first call
I2C_Start_Transceiver to send a request for data to the slave. Then Run this function to collect the
data when they have arrived. Include a pointer to where to place the data and the number of bytes
requested (including the address field) in the function call. The function will hold execution (loop)
until the I2C_ISR has completed with the previous operation, before reading out the data and returning.
If there was an error in the previous transmission the function will return the TWI error code.
****************************************************************************/
uint8_t I2C_Master_Get_Data_From_Transceiver( uint8_t *msg, uint8_t msgSize )
{

  while ( I2C_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.

  if( I2C_statusReg.lastTransOK )               // Last transmission completed successfully.
    for ( uint8_t i=0; i<msgSize; i++ )         // Copy data from Transceiver buffer.
			msg[ i ] = I2C_buf[ i ];

  return( I2C_statusReg.lastTransOK );
}

/****************************************************************************
Call this function to test if the I2C_ISR is busy transmitting.
****************************************************************************/
uint8_t I2C_Transceiver_Busy(void)
{
  return ( TWCR & (1<<TWIE) );                  // IF TWI Interrupt is enabled then the Transceiver is busy
}

/****************************************************************************
 * Manual Bus Check: Add this in your idle code, before issuing a start transceiver command.
 * Used only for multi-master operation.
******************************************************************************/
uint8_t I2C_Check_Free_After_Stop(void) //
{
//	I2C_checkBusyAfterStop = I2C_HOW_MANY_BUSY_CHECKS_AFTER_STOP;

	while ( I2C_checkBusyAfterStop > 0 )	// Call repeatedly
	{
		if((~(I2C_PORT_STATUS) && (I2C_BIT_SCL | I2C_BIT_SDA)) == 0) // both SCL and SDA should be high on idle bus.
			 // Good. The bus is quiet. Count down!
			 --I2C_checkBusyAfterStop;
		 else
			 // Bus is busy. Start the count down all over again.
			 I2C_checkBusyAfterStop = I2C_HOW_MANY_BUSY_CHECKS_AFTER_STOP;
	}
	return true;
}

/****************************************************************************
Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
until the TWI_ISR has completed with the previous operation. If there was an error, then the function
will return the TWI State code.
****************************************************************************/
uint8_t I2C_Get_State_Info(void)
{
  while ( I2C_Transceiver_Busy() );             // Wait until I2C has completed the transmission.
  return ( I2C_state );                         // Return error state.
}

uint8_t I2C_Get_State_InfoNonBlocking( void )
{
  return ( I2C_state );                         // Return error state.
}

