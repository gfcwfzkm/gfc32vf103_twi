
#include "gfc32vf103_twi.h"

void TWIM_init(TWI_Master_t *twim, uint32_t i2c_interface, uint32_t speed)
{
	//i2c_ackpos_config(twim->i2c_periph, I2C_ACKPOS_NEXT);
}

TWIM_Result_t TWIM_Result(TWI_Master_t *twim)
{
	return twim->result;
}

uint8_t TWIM_Ready(TWI_Master_t *twim)
{
	return twim->status;
}

TWIM_Error_t TWIM_Read(TWI_Master_t *twim, uint8_t slaveAddress, uint8_t bytesToRead)
{
	return TWIM_MasterWriteRead(twim, slaveAddress, 0, 0, bytesToRead);
}

TWIM_Error_t TWIM_Write(TWI_Master_t *twim, uint8_t slaveAddress, uint8_t *writeData, uint8_t bytesToWrite)
{
	return TWIM_MasterWriteRead(twim, slaveAddress, writeData, bytesToWrite, 0);
}

TWIM_Error_t TWIM_MasterWriteRead(TWI_Master_t *twim, uint8_t slaveAddress, uint8_t *writeData, uint8_t bytesToWrite, uint8_t bytesToRead)
{
	if (bytesToRead > TWI_READBUFFER_SIZE)
	{
		return TWIM_ERR_ARGERROR;
	}
	if (bytesToWrite > TWI_WRITEBUFFER_SIZE)
	{
		return TWIM_ERR_ARGERROR;
	}

	if (twim->status == TWIM_STATUS_READY)
	{
		if (i2c_flag_get(twim->i2c_periph, I2C_FLAG_I2CBSY))
		{
			return TWIM_ERR_BUSY;
		}

		twim->status = TWIM_STATUS_BUSY;
		twim->result = TWIM_RES_UNKNOWN;

		for (uint8_t bufIndex = 0; bufIndex < bytesToWrite; bufIndex++)
		{
			twim->writeBuffer[bufIndex] = writeData[bufIndex];
		}

		twim->bytesToWrite = bytesToWrite;
		twim->bytesToRead = bytesToRead;
		twim->bytesWritten = 0;
		twim->bytesRead = 0;

		if (twim->bytesToWrite > 0)
		{
			twim->mode = TWIM_MODE_SENDING;
			twim->address = slaveAddress & ~0x01;
		}
		else if (twim->bytesToRead > 0)
		{
			twim->mode = TWIM_MODE_RECEIVING;
			twim->address = slaveAddress | 0x01;
		}

		i2c_interrupt_enable(twim->i2c_periph, I2C_INT_ERR);
		i2c_interrupt_enable(twim->i2c_periph, I2C_INT_BUF);
		i2c_interrupt_enable(twim->i2c_periph, I2C_INT_EV);
		i2c_start_on_bus(twim->i2c_periph);

		return TWIM_ERR_OK;
	}
	else
	{
		return TWIM_ERR_BUSY;
	}
}

void TWIM_MasterIRQ(TWI_Master_t *twim)
{
	/* Are we sending or getting data? */
	if (twim->mode == TWIM_MODE_SENDING)
	{
		TWIM_WriteHandler(twim);
	}
	else
	{
		TWIM_ReadHandler(twim);
	}
}

/* Chew through the errors... */
void TWIM_ErrorIRQ(TWI_Master_t *twim)
{
	/* No Acknowledge received */
	if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_AERR))
	{
		twim->result = TWIM_RES_NACK_RECV;
		i2c_interrupt_flag_clear(twim->i2c_periph, I2C_INT_FLAG_AERR);
	}

	/* over-run or under-run happend */
	if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_OUERR))
	{
		twim->result = TWIM_RES_BUF_OVF;
		i2c_interrupt_flag_clear(twim->i2c_periph, I2C_INT_FLAG_OUERR);
	}

	/* arbitration lost */
	if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_LOSTARB))
	{
		twim->result = TWIM_RES_ARB_LOST;
		i2c_interrupt_flag_clear(twim->i2c_periph, I2C_INT_FLAG_LOSTARB);
	}

	/* bus error */
	if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_BERR))
	{
		twim->result = TWIM_RES_BUS_ERR;
		i2c_interrupt_flag_clear(twim->i2c_periph, I2C_INT_FLAG_BERR);
	}
	
}

static void TWIM_WriteHandler(TWI_Master_t *twim)
{
	if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_SBSEND))
	{
		/* Start condition has been sent, send slave address */
		i2c_master_addressing(twim->i2c_periph, twim->address, I2C_TRANSMITTER);
	}
	else if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_ADDSEND))
	{
		/* Address has been sent, flag has to be cleared by software */
		i2c_interrupt_flag_clear(twim->i2c_periph, I2C_INT_FLAG_ADDSEND);
	}
	else if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_TBE))
	{
		/* Transfer Buffer empty -> It is ready to send more data, so lets do that! */
		if (twim->bytesWritten < twim->bytesToWrite)
		{
			/* Still bytes left to write? Lets do so and increase the buffer index */
			i2c_data_transmit(twim->i2c_periph, twim->writeBuffer[twim->bytesWritten++]);

		}
		else if (twim->bytesRead < twim->bytesToRead)
		{
			/* We have data left to read back? RESTART transmission */
			twim->mode = TWIM_MODE_RECEIVING;
			i2c_start_on_bus(twim->i2c_periph);
		}
		else
		{
			/* We're done with everything it seems, so send a stop condition */
			i2c_stop_on_bus(twim->i2c_periph);

			/* Disable interrupts */
			i2c_interrupt_disable(twim->i2c_periph, I2C_INT_ERR);
			i2c_interrupt_disable(twim->i2c_periph, I2C_INT_BUF);
			i2c_interrupt_disable(twim->i2c_periph, I2C_INT_EV);

			twim->result = TWIM_RES_OK;
			twim->status = TWIM_STATUS_READY;
		}
	}
}

static void TWIM_ReadHandler(TWI_Master_t *twim)
{
	if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_SBSEND))
	{
		/* Start condition has been sent, send slave address */
		i2c_master_addressing(twim->i2c_periph, twim->address, I2C_RECEIVER);
	}
	else if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_ADDSEND))
	{
		/* Address has been sent, flag has to be cleared by software, 
		 * but we're also ready to receive now! But first lets check if
		 * we have to acknowledge or not... */
		if (twim->bytesToRead < 3)
		{
			i2c_ack_config(twim->i2c_periph, I2C_ACK_DISABLE);
		}
		else
		{
			i2c_ack_config(twim->i2c_periph, I2C_ACK_ENABLE);
		}
		i2c_interrupt_flag_clear(twim->i2c_periph, I2C_INT_FLAG_ADDSEND);
	}
	else if (i2c_interrupt_flag_get(twim->i2c_periph, I2C_INT_FLAG_RBNE))
	{
		/* We gotta (n)acknowledge ahead of the next package */
		if ((twim->bytesToRead - twim->bytesRead) < 3)
		{
			i2c_ack_config(twim->i2c_periph, I2C_ACK_DISABLE);
		}
		else
		{
			i2c_ack_config(twim->i2c_periph, I2C_ACK_ENABLE);
		}
		/* read the received data byte */
		if (twim->bytesRead < twim->bytesToRead)
		{
			twim->readBuffer[twim->bytesRead++] = i2c_data_receive(twim->i2c_periph);
			/* Are we done? */
			if (twim->bytesRead == twim->bytesToRead)
			{
				/* We're done with everything it seems, so send a stop condition */
				i2c_stop_on_bus(twim->i2c_periph);

				/* Disable interrupts */
				i2c_interrupt_disable(twim->i2c_periph, I2C_INT_ERR);
				i2c_interrupt_disable(twim->i2c_periph, I2C_INT_BUF);
				i2c_interrupt_disable(twim->i2c_periph, I2C_INT_EV);

				twim->result = TWIM_RES_OK;
				twim->status = TWIM_STATUS_READY;
			}
		}
	}
}