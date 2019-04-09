#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "test.h"
#include <bcm2835.h>

volatile unsigned *mem1, *mem2;
char dev_addr=0x40;						//Device I2C address

static void read_buf(char reg_addr, char *buf, unsigned short len);
static void write_buf(char reg_addr, char *buf, unsigned short len);
static int map_gpio();
static void setup_gpio();
static void restore_gpio();
static void wait_i2c_done(void);
static void test_gpio();

int main()
{
	uint8_t data;
	printf("Start\n");
	char buf;
	map_gpio();
	//test_gpio();
	setup_gpio();
	read_buf(0xF3, &buf, 1);
	//bcm2835_i2c_setSlaveAddress(dev_addr);
   	//bcm2835_i2c_setClockDivider(2500);
	//data = bcm2835_i2c_write(0xF3, 1);
        //printf("Write Result = %d\n", data);
	//printf("%d \n", buf);
	restore_gpio();
}

void wait_i2c_done(void) {
	printf("Wait for i2c\n");
    while ((!((BSC1_S) & BSC_S_DONE))) {
        //udelay(100);
    }
	printf("i2c done\n");
}

void read_buf(char reg_addr, char *buf, unsigned short len)
{
// Function to read a number of bytes into a  buffer from the FIFO of the I2C controller
//static void i2c_read(char dev_addr, char reg_addr, char *buf, unsigned short len)
	printf("Write to buf\n");
    write_buf(reg_addr, NULL, 0);
	printf("Wrote to buf\n");
    unsigned short bufidx;
    bufidx = 0;

    memset(buf, 0, len); // clear the buffer

    BSC1_DLEN = len;
    BSC1_S = CLEAR_STATUS; // Reset status bits (see #define)
    BSC1_C = START_READ; // Start Read after clearing FIFO (see #define)

    do {
        // Wait for some data to appear in the FIFO
        while ((BSC1_S & BSC_S_TA) && !(BSC1_S & BSC_S_RXD));

        // Consume the FIFO
        while ((BSC1_S & BSC_S_RXD) && (bufidx < len)) {
            buf[bufidx++] = BSC1_FIFO;
        }
    } while ((!(BSC1_S & BSC_S_DONE)));
	printf("Read from buf\n");
}

void write_buf(char reg_addr, char *buf, unsigned short len)
{
	
// Function to write data to an I2C device via the FIFO.  This doesn't refill the FIFO, so writes are limited to 16 bytes
// including the register address. len specifies the number of bytes in the buffer.
//static void i2c_write(char dev_addr, char reg_addr, char *buf, unsigned short len)
	
    int idx;

    BSC1_A = dev_addr;
    BSC1_DLEN = len + 1; // one byte for the register address, plus the buffer length

    BSC1_FIFO = reg_addr; // start register address
    for (idx = 0; idx < len; idx++)
        BSC1_FIFO = buf[idx];

    BSC1_S = CLEAR_STATUS; // Reset status bits (see #define)
    BSC1_C = START_WRITE; // Start Write (see #define)

    wait_i2c_done();
}

int map_gpio()
{
	int fd;
	static uint32_t mem1_base, mem2_base;

	mem1_base = GPIO_BASE + BCM2709_OFFSET;
	mem2_base = BSC1_BASE + BCM2709_OFFSET;
seteuid(0);
setfsuid(geteuid());
	fd = open("/dev/mem", O_RDWR|O_SYNC);
setfsuid(getuid());
	
	if (fd < 0) {
		printf("Can't open /dev/mem \n");
		return -1;
	}
	printf("Opened /dev/mem \n");
	
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
	printf("Mapped mem1\n");

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
	printf("Mapped mem2\n");

	return 0;
}

void setup_gpio()
{
	uint32_t x;

	/* change I2C pins */
	x = GPFSEL0;
	x &= ~(0x00000FC0);
	x |= 0x00000900;
	GPFSEL0 = x;
	printf("GPIO set up\n");
}

void test_gpio()
{
	uint32_t x;

	/* change I2C pins to outputs*/
	x = GPFSEL0;
	x &= ~(0x00007FC0);
	x |= 0x00001240;
	GPFSEL0 = x;
	printf("GPIO set up OUTPUT\n");
	
	//x = GPSET0;
	//x &= ~(0x0000000C);
	x |= 0x000000FF;
	GPSET0 = x;
	printf("GPIO 2,3 set high\n");
	GPCLR0 = x;
	printf("GPIO 2,3 set low\n");
}

void restore_gpio()
{
	uint32_t x;

	/* change I2C pins to inputs*/
	x = GPFSEL0;
	x &= ~(0x00007FC0);
	GPFSEL0 = x;
	printf("GPIO restored\n");
}
