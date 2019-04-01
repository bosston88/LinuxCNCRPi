#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "test.h"

volatile unsigned *mem1, *mem2;
char dev_addr=0x40;						//Device I2C address

static void read_buf(char reg_addr, char *buf, unsigned short len);
static void write_buf(char reg_addr, char *buf, unsigned short len);
static int map_gpio();
static void setup_gpio();
static void restore_gpio();
static void wait_i2c_done(void);

int main()
{
	char buf;
	map_gpio();
	setup_gpio();
	read_buf(0xF3, &buf, 1);
	printf("%d", buf);
	restore_gpio();
}

void wait_i2c_done(void) {
    while ((!((BCM2835_BSC1_S) & BSC_S_DONE))) {
       // udelay(100);
    }
}

void read_buf(char reg_addr, char *buf, unsigned short len)
{
// Function to read a number of bytes into a  buffer from the FIFO of the I2C controller
//static void i2c_read(char dev_addr, char reg_addr, char *buf, unsigned short len)

    write_buf(reg_addr, NULL, 0);

    unsigned short bufidx;
    bufidx = 0;

    memset(buf, 0, len); // clear the buffer

    BCM2835_BSC1_DLEN = len;
    BCM2835_BSC1_S = CLEAR_STATUS; // Reset status bits (see #define)
    BCM2835_BSC1_C = START_READ; // Start Read after clearing FIFO (see #define)

    do {
        // Wait for some data to appear in the FIFO
        while ((BCM2835_BSC1_S & BSC_S_TA) && !(BCM2835_BSC1_S & BSC_S_RXD));

        // Consume the FIFO
        while ((BCM2835_BSC1_S & BSC_S_RXD) && (bufidx < len)) {
            buf[bufidx++] = BCM2835_BSC1_FIFO;
        }
    } while ((!(BCM2835_BSC1_S & BSC_S_DONE)));
}

void write_buf(char reg_addr, char *buf, unsigned short len)
{
	
// Function to write data to an I2C device via the FIFO.  This doesn't refill the FIFO, so writes are limited to 16 bytes
// including the register address. len specifies the number of bytes in the buffer.
//static void i2c_write(char dev_addr, char reg_addr, char *buf, unsigned short len)
	
    int idx;

    BCM2835_BSC1_A = dev_addr;
    BCM2835_BSC1_DLEN = len + 1; // one byte for the register address, plus the buffer length

    BCM2835_BSC1_FIFO = reg_addr; // start register address
    for (idx = 0; idx < len; idx++)
        BCM2835_BSC1_FIFO = buf[idx];

    BCM2835_BSC1_S = CLEAR_STATUS; // Reset status bits (see #define)
    BCM2835_BSC1_C = START_WRITE; // Start Write (see #define)

    wait_i2c_done();
}

int map_gpio()
{
	int fd;
	static uint32_t mem1_base, mem2_base;

	mem1_base = BCM2835_GPIO_BASE + BCM2709_OFFSET;
	mem2_base = BCM2835_BSC1_BASE + BCM2709_OFFSET;
seteuid(0);
setfsuid(geteuid());
	fd = open("/dev/mem", O_RDWR|O_SYNC);
setfsuid(getuid());
	
	if (fd < 0) {
		printf("Can't open /dev/mem \n");
		return -1;
	}

	mem1 = mmap(
	        NULL,
	        BLOCK_SIZE,
	        PROT_READ|PROT_WRITE,
	        MAP_SHARED,
	        fd,
	        mem1_base);

	if (mem1 == MAP_FAILED) {
		printf("Can't map mem1\n");
		close(fd);
		return -1;
	}

	mem2 = mmap(
	        NULL,
	        BLOCK_SIZE,
	        PROT_READ|PROT_WRITE,
	        MAP_SHARED,
	        fd,
	        mem2_base);

	close(fd);

	if (mem2 == MAP_FAILED) {
		printf("Can't map mem2\n");
		return -1;
	}

	return 0;
}

void setup_gpio()
{
	uint32_t x;

	/* change I2C pins */
	x = BCM2835_GPFSEL0;
	x &= ~(0x00000FC0);
	x |= 0x00000900;
	BCM2835_GPFSEL0 = x;
}

void restore_gpio()
{
	uint32_t x;

	/* change I2C pins to inputs*/
	x = BCM2835_GPFSEL0;
	x &= ~(0x00000FC0);
	BCM2835_GPFSEL0 = x;
}
