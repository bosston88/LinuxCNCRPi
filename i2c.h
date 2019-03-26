#ifndef I2C_H
#define I2C_H

#define REQ_TIMEOUT		10000ul

#define SPIBUFSIZE		20		/* SPI buffer size */
#define BUFSIZE			(SPIBUFSIZE/4)

#define PAGE_SIZE		(4*1024)
#define BLOCK_SIZE		(4*1024)

/* Broadcom defines */

#define BCM2835_SPICLKDIV	32		/* ~8 Mhz */

#define BCM2835_PERI_BASE	0x20000000
#define BCM2709_OFFSET		0x1F000000
#define BCM2835_GPIO_BASE	(BCM2835_PERI_BASE + 0x200000) /* GPIO controller */
#define BCM2835_SPI_BASE	(BCM2835_PERI_BASE + 0x204000) /* SPI controller */

#define BCM2835_GPFSEL0		  *(mem1)
#define BCM2835_GPFSEL1		  *(mem1 + 1)
#define BCM2835_GPFSEL2		  *(mem1 + 2)
#define BCM2835_GPFSEL3		  *(mem1 + 3)
#define BCM2835_GPFSEL4		  *(mem1 + 4)
#define BCM2835_GPFSEL5		  *(mem1 + 5)
#define BCM2835_GPSET0		  *(mem1 + 7)
#define BCM2835_GPSET1		  *(mem1 + 8)
#define BCM2835_GPCLR0		  *(mem1 + 10)
#define BCM2835_GPCLR1		  *(mem1 + 11)
#define BCM2835_GPLEV0		  *(mem1 + 13)
#define BCM2835_GPLEV1		  *(mem1 + 14)

#define BCM2835_SPICS 		  *(mem2 + 0)
#define BCM2835_SPIFIFO     *(mem2 + 1)
#define BCM2835_SPICLK 		  *(mem2 + 2)

#define BCM_SPI_CS_DONE		  0x00010000
#define BCM_SPI_CS_TA		    0x00000080
#define BCM_SPI_CS_CLEAR_RX	0x00000020
#define BCM_SPI_CS_CLEAR_TX	0x00000010
#define BCM_SPI_CS_CPHA		  0x00000004

#endif
