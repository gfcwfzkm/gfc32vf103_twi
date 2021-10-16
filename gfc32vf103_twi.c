
#include "gfc32vf103_twi.h"

#define TWIM_MODE_SENDING	0
#define TWIM_MODE_RECEIVING	1

static void _WriteHandler(TWI_Master_t *twiPtr);
static void _ReadHandler(TWI_Master_t *twiPtr);

void twim_masterIRQ(TWI_Master_t *twiPtr)
{
	/* Are we sending or getting data? */
	if (twiPtr->mode == TWIM_MODE_SENDING)
	{
		_WriteHandler(twiPtr);
	}
	else
	{
		_ReadHandler(twiPtr);
	}
}

/* Chew through the errors... */
void twim_errorIRQ(TWI_Master_t *twiPtr)
{
	/* No Acknowledge received */
	if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_AERR))
	{
		twiPtr->result = TWIM_RES_NACK_RECV;
		i2c_interrupt_flag_clear(twiPtr->i2c_periph, I2C_INT_FLAG_AERR);
	}

	/* over-run or under-run happend */
	if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_OUERR))
	{
		twiPtr->result = TWIM_RES_BUF_OVF;
		i2c_interrupt_flag_clear(twiPtr->i2c_periph, I2C_INT_FLAG_OUERR);
	}

	/* arbitration lost */
	if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_LOSTARB))
	{
		twiPtr->result = TWIM_RES_ARB_LOST;
		i2c_interrupt_flag_clear(twiPtr->i2c_periph, I2C_INT_FLAG_LOSTARB);
	}

	/* bus error */
	if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_BERR))
	{
		twiPtr->result = TWIM_RES_BUS_ERR;
		i2c_interrupt_flag_clear(twiPtr->i2c_periph, I2C_INT_FLAG_BERR);
	}

	i2c_stop_on_bus(twiPtr->i2c_periph);
	/* Disable interrupts */
	i2c_interrupt_disable(twiPtr->i2c_periph, I2C_INT_ERR);
	i2c_interrupt_disable(twiPtr->i2c_periph, I2C_INT_BUF);
	i2c_interrupt_disable(twiPtr->i2c_periph, I2C_INT_EV);

	twiPtr->status = TWIM_STATUS_READY;
}


void twim_init(TWI_Master_t *twiPtr, const uint32_t interface, const uint32_t speed)
{
	twiPtr->bytesToRead = 0;
	twiPtr->bytesRead = 0;
	twiPtr->bytesToWrite = 0;
	twiPtr->bytesWritten = 0;
	twiPtr->mode = TWIM_MODE_RECEIVING;
	twiPtr->status = TWIM_STATUS_READY;
	twiPtr->result = TWIM_RES_OK;
	twiPtr->i2c_periph = interface;

	i2c_clock_config(twiPtr->i2c_periph, speed, I2C_DTCY_2);
	i2c_mode_addr_config(twiPtr->i2c_periph, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x02);
	i2c_enable(twiPtr->i2c_periph);
	i2c_ack_config(twiPtr->i2c_periph, I2C_ACKPOS_NEXT);
}

TWIM_Result_t twim_result(TWI_Master_t *twiPtr)
{
	return twiPtr->result;
}

TWIM_Status_t twim_ready(TWI_Master_t *twiPtr)
{
	return twiPtr->status;
}

TWIM_Error_t twim_read(TWI_Master_t *twiPtr, const uint8_t slaveAddress, const uint8_t bytesToRead)
{
	return twim_writeRead(twiPtr, slaveAddress, 0, 0, bytesToRead);
}

TWIM_Error_t twim_write(TWI_Master_t *twiPtr, const uint8_t slaveAddress, const uint8_t *const writeData, const uint8_t bytesToWrite)
{
	return twim_writeRead(twiPtr, slaveAddress, writeData, bytesToWrite, 0);
}

TWIM_Error_t twim_writeRead(TWI_Master_t *twiPtr, const uint8_t slaveAddress, const uint8_t *const writeData, const uint8_t bytesToWrite, const uint8_t bytesToRead)
{
	/* Check if things are even within parameters! */
	if (bytesToRead > TWI_READBUFFER_SIZE)
	{
		return TWIM_ERR_ARGERROR;
	}
	if (bytesToWrite > TWI_WRITEBUFFER_SIZE)
	{
		return TWIM_ERR_ARGERROR;
	}

	if (twiPtr->status == TWIM_STATUS_READY)
	{
		if (i2c_flag_get(twiPtr->i2c_periph, I2C_FLAG_I2CBSY))
		{
			return TWIM_ERR_BUSY;
		}

		twiPtr->status = TWIM_STATUS_BUSY;
		twiPtr->result = TWIM_RES_UNKNOWN;

		for (uint8_t bufIndex = 0; bufIndex < bytesToWrite; bufIndex++)
		{
			twiPtr->writeBuffer[bufIndex] = writeData[bufIndex];
		}

		twiPtr->bytesToWrite = bytesToWrite;
		twiPtr->bytesToRead = bytesToRead;
		twiPtr->bytesWritten = 0;
		twiPtr->bytesRead = 0;

		if (twiPtr->bytesToWrite > 0)
		{
			twiPtr->mode = TWIM_MODE_SENDING;
			twiPtr->address = slaveAddress & ~0x01;
		}
		else if (twiPtr->bytesToRead > 0)
		{
			twiPtr->mode = TWIM_MODE_RECEIVING;
			twiPtr->address = slaveAddress | 0x01;
		}

		i2c_interrupt_enable(twiPtr->i2c_periph, I2C_INT_ERR);
		i2c_interrupt_enable(twiPtr->i2c_periph, I2C_INT_BUF);
		i2c_interrupt_enable(twiPtr->i2c_periph, I2C_INT_EV);
		i2c_start_on_bus(twiPtr->i2c_periph);

		return TWIM_ERR_OK;
	}
	else
	{
		return TWIM_ERR_BUSY;
	}
}

static void _WriteHandler(TWI_Master_t *twiPtr)
{
	if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_SBSEND))
	{
		/* Start condition has been sent, send slave address */
		i2c_master_addressing(twiPtr->i2c_periph, twiPtr->address, I2C_TRANSMITTER);
	}
	else if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_ADDSEND))
	{
		/* Address has been sent, flag has to be cleared by software */
		i2c_interrupt_flag_clear(twiPtr->i2c_periph, I2C_INT_FLAG_ADDSEND);
	}
	else if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_TBE))
	{
		/* Transfer Buffer empty -> It is ready to send more data, so lets do that! */
		if (twiPtr->bytesWritten < twiPtr->bytesToWrite)
		{
			/* Still bytes left to write? Lets do so and increase the buffer index */
			i2c_data_transmit(twiPtr->i2c_periph, twiPtr->writeBuffer[twiPtr->bytesWritten++]);

		}
		else if (twiPtr->bytesRead < twiPtr->bytesToRead)
		{
			/* We have data left to read back? RESTART transmission */
			twiPtr->mode = TWIM_MODE_RECEIVING;
			i2c_start_on_bus(twiPtr->i2c_periph);
		}
		else
		{
			/* We're done with everything it seems, so send a stop condition */
			i2c_stop_on_bus(twiPtr->i2c_periph);

			/* Disable interrupts */
			i2c_interrupt_disable(twiPtr->i2c_periph, I2C_INT_ERR);
			i2c_interrupt_disable(twiPtr->i2c_periph, I2C_INT_BUF);
			i2c_interrupt_disable(twiPtr->i2c_periph, I2C_INT_EV);

			twiPtr->result = TWIM_RES_OK;
			twiPtr->status = TWIM_STATUS_READY;
		}
	}
}

static void _ReadHandler(TWI_Master_t *twiPtr)
{
	if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_SBSEND))
	{
		/* Start condition has been sent, send slave address */
		i2c_master_addressing(twiPtr->i2c_periph, twiPtr->address, I2C_RECEIVER);
	}
	else if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_ADDSEND))
	{
		/* Address has been sent, flag has to be cleared by software, 
		 * but we're also ready to receive now! But first lets check if
		 * we have to acknowledge ahead or not... */
		if (twiPtr->bytesToRead < 2)
		{
			i2c_ack_config(twiPtr->i2c_periph, I2C_ACK_DISABLE);
		}
		else
		{
			i2c_ack_config(twiPtr->i2c_periph, I2C_ACK_ENABLE);
		}
		i2c_interrupt_flag_clear(twiPtr->i2c_periph, I2C_INT_FLAG_ADDSEND);
	}
	else if (i2c_interrupt_flag_get(twiPtr->i2c_periph, I2C_INT_FLAG_RBNE))
	{
		/* We gotta (n)acknowledge ahead of the next package */
		if ((twiPtr->bytesToRead - twiPtr->bytesRead) < 3)
		{
			i2c_ack_config(twiPtr->i2c_periph, I2C_ACK_DISABLE);
		}
		else
		{
			i2c_ack_config(twiPtr->i2c_periph, I2C_ACK_ENABLE);
		}
		/* read the received data byte */
		if (twiPtr->bytesRead < twiPtr->bytesToRead)
		{
			twiPtr->readBuffer[twiPtr->bytesRead++] = i2c_data_receive(twiPtr->i2c_periph);
			/* Are we done? */
			if (twiPtr->bytesRead == twiPtr->bytesToRead)
			{
				/* We're done with everything it seems, so send a stop condition */
				i2c_stop_on_bus(twiPtr->i2c_periph);

				/* Disable interrupts */
				i2c_interrupt_disable(twiPtr->i2c_periph, I2C_INT_ERR);
				i2c_interrupt_disable(twiPtr->i2c_periph, I2C_INT_BUF);
				i2c_interrupt_disable(twiPtr->i2c_periph, I2C_INT_EV);

				twiPtr->result = TWIM_RES_OK;
				twiPtr->status = TWIM_STATUS_READY;
			}
		}
	}
}

uint8_t twim_InterfacePrepare(void *intTWI)
{
	TWI_Master_t *twiPtr = (TWI_Master_t*)intTWI;
	return twim_ready(twiPtr);
}

uint8_t twim_InterfaceSendBytes(void *intTWI, uint8_t addr, uint8_t *buf_ptr, uint8_t buf_len)
{
	TWI_Master_t *twiPtr = (TWI_Master_t*)intTWI;
	uint8_t _errors = 0;

	if (twim_write(twiPtr, addr, buf_ptr, buf_len) != TWIM_ERR_OK)	_errors = 1;

	while(twim_ready(twiPtr) == TWIM_STATUS_BUSY);

	if (twim_result(twiPtr) != TWIM_RES_OK)	_errors = 1;

	return _errors;
}

uint8_t twim_InterfaceTransceiveBytes(void *intTWI, uint8_t addr, uint8_t *buf_ptr, uint8_t buf_len)
{
	/* Function not supported yet */
	return 1;
}

uint8_t twim_InterfaceGetBytes(void *intTWI, uint8_t addr, uint8_t *buf_ptr, uint8_t buf_len)
{
	TWI_Master_t *twiPtr = (TWI_Master_t*)intTWI;
	uint8_t _errors = 0;
	uint8_t byteCounter;

	if (twim_read(twiPtr, addr, buf_len) != TWIM_ERR_OK)	_errors = 1;

	while(twim_ready(twiPtr) == TWIM_STATUS_BUSY);

	if (twim_result(twiPtr) == TWIM_RES_OK)
	{
		for (byteCounter = 0; byteCounter < buf_len; byteCounter++)
		{
			buf_ptr[byteCounter] = twiPtr->readBuffer[byteCounter];
		}
	}
	else
	{
		_errors = 1;
	}

	return _errors;
}

uint8_t twim_InterfaceFinish(void *intTWI)
{
	return 0;
}