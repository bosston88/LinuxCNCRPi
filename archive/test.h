#ifndef I2C_H
#define I2C_H

#define REQ_TIMEOUT		10000ul

#define SPIBUFSIZE		20		/* SPI buffer size */
#define BUFSIZE			(SPIBUFSIZE/4)

#define PAGE_SIZE		(4*1024)
#define BLOCK_SIZE		(4*1024)

/* Broadcom defines */

#define PERI_BASE	0x20000000
#define BCM2709_OFFSET		0x1F000000
#define GPIO_BASE	(PERI_BASE + 0x200000) /* GPIO controller */
#define BSC1_BASE	(PERI_BASE + 0x804000) /* I2C controller */

#define GPFSEL0		  *(mem1)
#define GPFSEL1		  *(mem1 + 1)
#define GPFSEL2		  *(mem1 + 2)
#define GPFSEL3		  *(mem1 + 3)
#define GPFSEL4		  *(mem1 + 4)
#define GPFSEL5		  *(mem1 + 5)
#define GPSET0		  *(mem1 + 7)
#define GPSET1		  *(mem1 + 8)
#define GPCLR0		  *(mem1 + 10)
#define GPCLR1		  *(mem1 + 11)
#define GPLEV0		  *(mem1 + 13)
#define GPLEV1		  *(mem1 + 14)

#define BSC1_C 		  *(mem2 + 0)
#define BSC1_S      *(mem2 + 1)
#define BSC1_DLEN 	*(mem2 + 2)
#define BSC1_A      *(mem2 + 3)
#define BSC1_FIFO 	*(mem2 + 4)
#define BSC1_DIV 	  *(mem2 + 5)
#define BSC1_DEL    *(mem2 + 6)
#define BSC1_CLKT 	*(mem2 + 7)

#define BSC_C_I2CEN	(1 << 15)
#define BSC_C_INTR	(1 << 10)
#define BSC_C_INTT	(1 << 9)
#define BSC_C_INTD	(1 << 8)
#define BSC_C_ST	(1 << 7)
#define BSC_C_CLEAR	(1 << 4)
#define BSC_C_READ	1

#define START_READ	BSC_C_I2CEN|BSC_C_ST|BSC_C_CLEAR|BSC_C_READ
#define START_WRITE	BSC_C_I2CEN|BSC_C_ST

#define BSC_S_CLKT	(1 << 9)
#define BSC_S_ERR	(1 << 8)
#define BSC_S_RXF	(1 << 7)
#define BSC_S_TXE	(1 << 6)
#define BSC_S_RXD	(1 << 5)
#define BSC_S_TXD	(1 << 4)
#define BSC_S_RXR	(1 << 3)
#define BSC_S_TXW	(1 << 2)
#define BSC_S_DONE	(1 << 1)
#define BSC_S_TA	1

#define CLEAR_STATUS	BSC_S_CLKT|BSC_S_ERR|BSC_S_DONE

#endif
