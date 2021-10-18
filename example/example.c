/**
 * Example code, demonstrating the usage of my I2C library
 * for the GD32V RISC-V microcontrollers.
 **/

#include "gd32vf103.h"
#include "gfc32vf103_twi.h"

/* Port B */
#define SCL         	BIT(6)	// PB.6 is SCL
#define SDA         	BIT(7)	// PB.7 is SDA
#define I2C_SPEED		100000	// 100kHz I2C speed
#define M24C16_ADDRESS	0xA0	// Address of the M24C16 I2C EEPROM (2kByte)
// Should work with any of the M24Cxx series I2C EEPROMs tho!

/* I2C Driver Interface */
TWI_Master_t twiInstance;

/* The two interrupt calls going back to the library for processing */
void I2C0_EV_IRQHandler(void)
{
	twim_masterIRQ(&twiInstance);
}

void I2C0_ER_IRQHandler(void)
{
	twim_errorIRQ(&twiInstance);
}

int main(void)
{
	// First byte represents the internal eeprom address, the rest the test data
	uint8_t testData[9] = {0,1,2,3,4,5,6,7,8};

	/* Enable clocks for peripherals */
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_I2C0);

	/* Set the I/Os up for I2C */
	gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_2MHZ, SCL | SDA);

	/* Set up the I2C interrupt */
	eclic_global_interrupt_enable();
	eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
	eclic_irq_enable(I2C0_ER_IRQn, 1, 0);
	eclic_irq_enable(I2C0_EV_IRQn, 2, 0);

	/* Now finally initialise the TWI library */
	twim_init(&twiInstance, I2C0, I2C_SPEED);

	/* Lets start the operation and wait for it to finish */
	twim_write(&twiInstance, M24C16_ADDRESS, testData, 9);
	while(twim_ready(&twiInstance) == TWIM_STATUS_BUSY);

	/* Check if the operation performed without errors */
	if (twim_result(&twiInstance) != TWIM_RES_OK)
	{
		/* An error occured! */
		while(1);
	}

	/* Now lets read back the data. Since we have to state the internal address of 
	 * the eeprom, we have to perform a write-read operation using the first testData 
	 * entry again, followed by reading back 8 bytes from the eeprom */
	twim_writeRead(&twiInstance, M24C16_ADDRESS, testData, 1, 8);
	while(twim_ready(&twiInstance) == TWIM_STATUS_BUSY);

	/* Check if the operation performed without errors */
	if (twim_result(&twiInstance) != TWIM_RES_OK)
	{
		/* An error occured! */
		while(1);
	}

	/* Low lets compare the read back data against the testData array. The first index is
	 * not part of the data but part of the eeprom address */
	for (uint8_t i = 0; i < 8; i++)
	{
		if (testData[i+1] != twiInstance.readBuffer[i])
		{
			/* EEPROM Data not the same as what we've sent before */
			while(1);
		}
	}

	/* Test has been performed sucessfully! Loop forever */
	while(1);
}