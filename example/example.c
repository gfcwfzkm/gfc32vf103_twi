
#include "gd32vf103.h"
#include "gfc32vf103_twi.h"

/* Port B */
#define SCL         	BIT(6)	// PB.6 is SCL
#define SDA         	BIT(7)	// PB.7 is SDA
#define I2C_SPEED		100000	// 100kHz I2C speed
#define M24C16_ADDRESS	0xA0	// Address of the M24C16 I2C EEPROM (2kByte)

/* I2C Driver Interface */
TWI_Master_t twiInstance;

int main(void)
{
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
}