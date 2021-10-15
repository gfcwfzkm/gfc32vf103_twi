
#ifndef GFC32VF103_TWI_H
#define GFC32VF103_TWI_H

#include <inttypes.h>
#include <gd32vf103.h>
#include <gd32vf103_i2c.h>

typedef enum TWIM_Result{
	TWIM_RES_OK			= (0<<0),
	TWIM_RES_UNKNOWN	= (1<<0),
	TWIM_RES_BUF_OVF	= (1<<1),
	TWIM_RES_ARB_LOST	= (1<<2),
	TWIM_RES_BUS_ERR	= (1<<3),
	TWIM_RES_NACK_RECV	= (1<<4),
	TWIM_RES_FAIL		= (1<<5),
} TWIM_Result_t;

typedef enum TWIM_Error{
	TWIM_ERR_OK			= (0<<0),
	TWIM_ERR_ARGERROR	= (1<<0),
	TWIM_ERR_BUSY	= (1<<1),
} TWIM_Error_t;

#define TWIM_STATUS_BUSY	0
#define TWIM_STATUS_READY	1

#define TWIM_MODE_SENDING	0
#define TWIM_MODE_RECEIVING	1

#define TWI_WRITEBUFFER_SIZE	32
#define TWI_READBUFFER_SIZE		32

typedef struct {
	uint32_t i2c_periph;
	volatile uint8_t address;
	uint8_t writeBuffer[TWI_WRITEBUFFER_SIZE];
	uint8_t readBuffer[TWI_READBUFFER_SIZE];
	volatile uint8_t bytesToWrite;
	volatile uint8_t bytesToRead;
	volatile uint8_t bytesWritten;
	volatile uint8_t bytesRead;
	volatile uint8_t status:2;
	volatile uint8_t mode:2;
	volatile TWIM_Result_t result;
}TWI_Master_t;

void TWIM_init(TWI_Master_t *twim, uint32_t i2c_interface, uint32_t speed);
TWIM_Error_t TWIM_Read(TWI_Master_t *twim, uint8_t slaveAddress, uint8_t bytesToRead);
TWIM_Error_t TWIM_Write(TWI_Master_t *twim, uint8_t slaveAddress, uint8_t *writeData, uint8_t bytesToWrite);
TWIM_Error_t TWIM_MasterWriteRead(TWI_Master_t *twim, uint8_t slaveAddress, uint8_t *writeData, uint8_t bytesToWrite, uint8_t bytesToRead);
void TWIM_MasterIRQ(TWI_Master_t *twim);
void TWIM_ErrorIRQ(TWI_Master_t *twim);

/* GFC32VF103_I2C_H */
#endif