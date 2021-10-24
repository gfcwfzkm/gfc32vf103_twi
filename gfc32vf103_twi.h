
#ifndef GFC32VF103_TWI_H
#define GFC32VF103_TWI_H

#include <inttypes.h>
#include <gd32vf103.h>
#include <gd32vf103_i2c.h>

/**
 * @brief I2C Results of the transactions 
 */
typedef enum TWIM_Result{
	TWIM_RES_OK			= (0<<0),	/**< I2C transmission successful! */
	TWIM_RES_UNKNOWN	= (1<<0),	/**< I2C transmission result unknown, likely still busy */
	TWIM_RES_BUF_OVF	= (1<<1),	/**< I2C transmission failed due to over- or underflow */
	TWIM_RES_ARB_LOST	= (1<<2),	/**< I2C transmission failed, arbritation lost */
	TWIM_RES_BUS_ERR	= (1<<3),	/**< I2C transmission general bus error */
	TWIM_RES_NACK_RECV	= (1<<4)	/**< I2C no acknowledge received from slave when calling for it */
} TWIM_Result_t;

/**
 * @brief I2C errors
 */
typedef enum TWIM_Error{
	TWIM_ERR_OK			= (0<<0),	/**< I2C No error occurred */
	TWIM_ERR_ARGERROR	= (1<<0),	/**< Arguments invalid, requested bytes larger than buffer size */
	TWIM_ERR_BUSY		= (1<<1)	/**< I2C Peripheral still busy */
} TWIM_Error_t;

typedef enum TWIM_Status{
	TWIM_STATUS_READY	= (0<<0),	/**< I2C peripheral ready for new transactions */
	TWIM_STATUS_BUSY	= (1<<0),	/**< I2C peripheral busy with a transaction */
} TWIM_Status_t;

#define TWI_WRITEBUFFER_SIZE	32
#define TWI_READBUFFER_SIZE		32

/**
 * @brief Typedefinition for the struct containing the I2C instance 
 */
typedef struct {
	uint32_t i2c_periph;		/**< hardware pointer to the I2C peripheral */
	volatile uint8_t address;	/**< slave address */
	uint8_t writeBuffer[TWI_WRITEBUFFER_SIZE];	/**< write buffer, max 255 */
	uint8_t readBuffer[TWI_READBUFFER_SIZE];	/**< read buffer, max 255 */
	volatile uint8_t bytesToWrite;	/**< total bytes to write, max 255 */
	volatile uint8_t bytesToRead;	/**< total bytes to read, max 255 */
	volatile uint8_t bytesWritten;	/**< Buffer index for the bytes already written */
	volatile uint8_t bytesRead;		/**< Buffer index for the bytes already received */
	volatile TWIM_Status_t status:2;		/**< Status of the driver / transmission */
	volatile uint8_t mode:2;		/**< Read or Write mode of the current transaction */
	volatile TWIM_Result_t result;	/**< Result of the transaction, contains possible failures */
}TWI_Master_t;

/**
 * @brief I2C event interrupt
 * 
 * Call this function from your I2C event ISR interrupt function.
 * Example: \n \code{.c}
 * void I2C0_EV_IRQHandler(void)
 * {
 *     twim_masterIRQ(&twiInstance);
 * }
 * \endcode
 * @param twiPtr	Pointer to the initialised \a TWI_Master_t instance
 */
void twim_masterIRQ(TWI_Master_t *twiPtr);

/**
 * @brief I2C error interrupt
 * 
 * Call this function from your I2C error ISR interrupt function.
 * Example: \n \code{.c}
 * void I2C0_ER_IRQHandler(void)
 * {
 *     twim_errorIRQ(&twiInstance);
 * }
 * \endcode
 * @param twiPtr	Pointer to the initialised \a TWI_Master_t instance
 */
void twim_errorIRQ(TWI_Master_t *twiPtr);

/**
 * @brief Initialises the I2C interface
 * 
 * Initialise the I2C peripheral and the library
 * Example: \n \code{.c}
 * #define I2C_SPEED	400000
 * TWI_Master_t twiInstance;
 * ...
 * // Configure interrupts for the I2C peripheral, like this for example:
 * eclic_global_interrupt_enable();
 * eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
 * eclic_irq_enable(I2C0_EV_IRQn, 1, 0);
 * eclic_irq_enable(I2C0_ER_IRQn, 2, 0);
 * // Now initialise the TWI library
 * twim_init(&twiInstance, I2C0, I2C_SPEED);
 * \endcode
 * @param twiPtr	Pointer to the unsued \a TWI_Master_t instance
 * @param interface	Pointer / Address to the I2C peripheral
 * @param speed		The I2C Speed in Hertz
 */
void twim_init(TWI_Master_t *twiPtr, const uint32_t interface, const uint32_t speed);

/**
 * @brief Get the result of the I2C transaction
 * 
 * Read back the result of the last I2C operation
 * Example: \n \code{.c}
 * if (twim_result(&twiInstance) != TWIM_RES_OK)
 * {
 * 	   // Handle errors
 * }
 * \endcode
 * @param twiPtr	Pointer to the initialised \a TWI_Master_t instance
 * @return 			Returns the result and errors of the I2C transaction
 */
TWIM_Result_t twim_result(TWI_Master_t *twiPtr);

/**
 * @brief Ready for next transaction
 * 
 * Check if the peripheral / library is busy or not
 * Example: \n \code{.c}
 * if (twim_ready(&twiInstance) == TWIM_STATUS_READY)
 * {
 * 	  //Start next transaction or read the reports of the last transaction
 * }
 * \endcode
 * @param twiPtr	Pointer to the initialised \a TWI_Master_t instance
 * @return			Returns TWIM_STATUS_BUSY if busy, else TWIM_STATUS_READY
 */
TWIM_Status_t twim_ready(TWI_Master_t *twiPtr);

/**
 * @brief Perform a read operation
 * 
 * Initiate a I2C read transaction with this function. The task performs mostly in the background
 * using interrupts. Check with \a twim_ready if the transaction has been finished or not. The
 * saved data are stored in the readBuffer of the \a TWI_Master_t instance.
 * Example: \n \code{.c}
 * if (twim_read(&twiInstance, SLAVE_ADDRESS, 8))	// Read 8 Bytes from the I2C slave at address 0xA0
 * {
 * 		// Handle errors
 * }
 * else
 * {
 * 		while(twim_ready(&twiInstance) == TWIM_STATUS_BUSY);
 * 		// Process \a twim_result and read back the data from the buffer
 * }
 * OUT_PORT = twiInstance.readBuffer[0];	// Display the first received byte at some LEDs
 * \endcode
 * @param twiPtr		Pointer to the initialised \a TWI_Master_t instance
 * @param slaveAddress	I2C slave address (8-bit format!)
 * @param bytesToRead	Amount of bytes to read
 * @return 				\a TWIM_Error_t Returns a non-zero value if a error occures
 */
TWIM_Error_t twim_read(TWI_Master_t *twiPtr, const uint8_t slaveAddress, const uint8_t bytesToRead);

/**
 * @brief Perform a write operation
 * 
 * Initiate a I2C write operation with this function. The task performs mostly in the background
 * using interrupts. Check with \a twim_ready if the transaction has been finished or not. The
 * buffer's contents are passed over to the \a TWI_Master_t instance's buffer, so it can be re-used
 * directly after the function has been called.
 * Example: \n \code{.c}
 * uint8_t dataToSend[6] = "Hello";
 * if (twim_read(&twiInstance, SLAVE_ADDRESS, dataToSend, 8))
 * {
 * 		// Handle errors
 * }
 * \endcode
 * @param twiPtr		Pointer to the initialised \a TWI_Master_t instance
 * @param slaveAddress	I2C slave address (8-bit format!)
 * @param writeData		Pointer to the data to write 
 * @param bytesToWrite	Amount of bytes to write
 * @return 				\a TWIM_Error_t Returns a non-zero value if a error occures
 */
TWIM_Error_t twim_write(TWI_Master_t *twiPtr, const uint8_t slaveAddress, const uint8_t *const writeData, const uint8_t bytesToWrite);

/**
 * @brief Perform both a write and read operation
 * 
 * Initiate a general I2C operation with this function. If both bytes to write and to read are requested,
 * then first a write operation is performed before switching over to perform a read operation. The
 * whole transaction happens nearly fully in the background using interrupts. Read \a twim_read and \a twim_write
 * for details to specifically pass over data and read it back.
 * Example: \n \code{.c}
 * // Read 8 Bytes from the EEPROM that are stored at address 0x7F 
 * uint8_t eepromAddress = 0x7F;
 * if (twim_writeRead(&twiInstance, SLAVE_ADDRESS, &eepromAddress, 1, 8))
 * {
 * 		// Handle errors
 * }
 * else
 * {
 * 		// Lets wait for the sake of this demo
 * 		while(twim_ready(&twiInstance) == TWIM_STATUS_BUSY);
 * 		for (uint8_t i = 0; i < 8; i++){
 * 			OUT_PORT = twiInstance.readBuffer[i]; // Display the EEPROM's contents at the output LEDs
 * 			delay_ms(500);
 * 		}
 * }
 * \endcode
 * @param twiPtr		Pointer to the initialised \a TWI_Master_t instance
 * @param slaveAddress	I2C slave address (8-bit format!)
 * @param writeData		Pointer to the data to write 
 * @param bytesToWrite	Amount of bytes to write
 * @param bytesToRead	Amount of bytes to read
 * @return 				\a TWIM_Error_t Returns a non-zero value if a error occures
 */
TWIM_Error_t twim_writeRead(TWI_Master_t *twiPtr, const uint8_t slaveAddress, const uint8_t *const writeData, const uint8_t bytesToWrite, uint8_t bytesToRead);

/* Special I/O functions for the universal driver interface that my IC drivers / libraries use: */
uint8_t twim_InterfacePrepare(void *intTWI);
uint8_t twim_InterfaceSendBytes(void *intTWI, uint8_t addr, uint8_t *buf_ptr, uint16_t buf_len);
uint8_t twim_InterfaceTransceiveBytes(void *intTWI, uint8_t addr, uint8_t *buf_ptr, uint16_t buf_len);
uint8_t twim_InterfaceGetBytes(void *intTWI, uint8_t addr, uint8_t *buf_ptr, uint16_t buf_len);
uint8_t twim_InterfaceFinish(void *intTWI);

/* GFC32VF103_I2C_H */
#endif