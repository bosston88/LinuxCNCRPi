#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "si7021.h"

//#if !defined(BUILD_SYS_USER_DSO)
//#error "This driver is for usermode threads only"
//#endif

#define MODNAME "si7021"
#define PREFIX "si7021"

MODULE_AUTHOR("Bosston88");
MODULE_DESCRIPTION("Driver for Si7021 I2C");
MODULE_LICENSE("GPL");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* this structure contains the runtime data needed by the
   driver for a single port/channel
*/

typedef struct {
	hal_float_t *data_in;
} data_t;

/* pointer to array of data_t structs in shared memory*/
static data_t *data;

static int comp_id;						/* component ID */
static const char *modname = MODNAME;
static const char *prefix = PREFIX;

volatile unsigned *mem1, *mem2;
char dev_addr=0x40;						//Device I2C address

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
/* These is the functions that actually do the I/O
   everything else is just init code
*/
static void read_i2c(void *arg, long period);
static void read_buf(char reg_addr, char *buf, unsigned short len);
static void write_buf(char reg_addr, char *buf, unsigned short len);
static int map_gpio();
static void setup_gpio();
static void restore_gpio();
static void wait_i2c_done(void);
static int setup_gpiomem_access(void);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/
int rtapi_app_main(void)
{
	char name[HAL_NAME_LEN + 1];
	int n, retval;
	n=0;
	
	/* initialise the driver */
   	comp_id = hal_init(modname);
    	if (comp_id < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", modname);
		return -1;
    	}
	
	/* allocate shared memory */
	data = hal_malloc(sizeof(data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	
	retval = map_gpio();
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: cannot map GPIO memory\n", modname);
		return retval;
	}
	else rtapi_print_msg(RTAPI_MSG_INFO, "%s: GPIO mapped\n", modname);

	setup_gpio();
	rtapi_print_msg(RTAPI_MSG_INFO, "%s: GPIO set up\n", modname);
	
	/* export the pin(s) */
	retval = hal_pin_float_newf(HAL_OUT, &(data->data_in), comp_id, "%s.temp.in", prefix, n); //dodane n
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin export failed with err=%i\n", modname, retval);
		hal_exit(comp_id);
		return -1;
    	}
	*(data->data_in) = 0.0;
	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Pin created\n", modname);
	
	/* export functions */
	rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
	retval = hal_export_funct(name, read_i2c, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: read function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	rtapi_print_msg(RTAPI_MSG_INFO, "%s: Read exported\n", modname);
	
	write_buf(0xF3, NULL, 0);
	
	/* ready */
	rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
	return 0;
}

void rtapi_app_exit(void)
{
	restore_gpio();
	munmap((void *)mem1,BLOCK_SIZE);
	munmap((void *)mem2,BLOCK_SIZE);
	hal_exit(comp_id);
}

void read_i2c(void *arg, long period)
{
	data_t *dat = (data_t *)arg;
	char buf[2];
	uint16_t out;
	
	read_buf(0, buf, 2);
	out=buf[0]<< 8 | buf[1];      

      	float temperature = out;
      	temperature *= 175.72;
      	temperature /= 65536;
      	temperature -= 46.85;
	
	write_buf(0xF3, NULL, 0);
	
	*(dat->data_in) = temperature;//wynik konwersacji
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

    if(reg_addr==0) BCM2835_BSC1_A = dev_addr;
    else write_buf(reg_addr, NULL, 0);

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
		rtapi_print_msg(RTAPI_MSG_ERR,"%s: can't open /dev/mem \n",modname);
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
		rtapi_print_msg(RTAPI_MSG_ERR,"%s: can't map mem1\n",modname);
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
		rtapi_print_msg(RTAPI_MSG_ERR,"%s: can't map mem2\n",modname);
		return -1;
	}

	return 0;
}
	
/*    	GPIO USAGE
 *
 *   	RPI:
 *
 *		GPIO	Dir		Signal
 *
 *		2	IN/OUT		SDA
 *		3	OUT		SCL
 */

void setup_gpio()
{
	uint32_t x;

	/* change I2C pins */
	x = BCM2835_GPFSEL0;
	x &= ~(0x00000FC0);
	x |= 0x00000900;
	BCM2835_GPFSEL0 = x;

	/* set up SPI */
	//BCM2835_SPICLK = BCM2835_SPICLKDIV; 		tutaj mozna dodac prescaler - domyslnie 100kHz 

	//BCM2835_SPICS = 0;

	/* clear FIFOs */
	//BCM2835_SPICS |= BCM_SPI_CS_CLEAR_RX | BCM_SPI_CS_CLEAR_TX;
}

void restore_gpio()
{
	uint32_t x;

	/* change I2C pins to inputs*/
	x = BCM2835_GPFSEL0;
	x &= ~(0x00000FC0);
	BCM2835_GPFSEL0 = x;
}
