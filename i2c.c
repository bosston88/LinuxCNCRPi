#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"

#if !defined(BUILD_SYS_USER_DSO)
#error "This driver is for usermode threads only"
#endif

#define MODNAME "I2C"
#define PREFIX "I2C"

MODULE_AUTHOR("Bosston88");
MODULE_DESCRIPTION("Driver for ADC I2C");
MODULE_LICENSE("GPL");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* this structure contains the runtime data needed by the
   driver for a single port/channel
*/

typedef struct {
	hal_float_t *data_in,
} data_t;

/* pointer to array of data_t structs in shared memory*/
static data_t *data;

static int comp_id;						/* component ID */
static const char *modname = MODNAME;
static const char *prefix = PREFIX;

volatile unsigned *mem1, *mem2;

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
/* These is the functions that actually do the I/O
   everything else is just init code
*/
static void read_i2c(void *arg, long period);
static void read_buf();
static void write_buf();
static int map_gpio();
static void setup_gpio();
static void restore_gpio();

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/
int rtapi_app_main(void)
{
	char name[HAL_NAME_LEN + 1];
	int n, retval;
	
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
	
	/* configure board */
	retval = map_gpio();
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: cannot map GPIO memory\n", modname);
		return retval;
	}

	setup_gpio();
	
	/* export the pin(s) */
	retval = hal_pin_bit_newf(HAL_OUT, &(data->data_in), comp_id, "%s.in.adc", prefix);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin export failed with err=%i\n", modname, retval);
		hal_exit(comp_id);
		return -1;
    }
	*(data->data_in) = 0;
	
	/* export functions */
	rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
	retval = hal_export_funct(name, read_i2c, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: read function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	
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
	
	*(dat->data_in) = ;
}

void read_buf()
{
	char *buf;
	int i;

	/* read buffer */
	buf = (char *)rxBuf;
	for (i=0; i<SPIBUFSIZE; i++) {
		*buf++ = BCM2835_SPIFIFO;
	}
}

void write_buf()
{
	char *buf;
	int i;

	/* activate transfer */
	BCM2835_SPICS = BCM_SPI_CS_TA | BCM_SPI_CS_CPHA;

	/* send txBuf */
	buf = (char *)txBuf;
	for (i=0; i<SPIBUFSIZE; i++) {
		BCM2835_SPIFIFO = *buf++;
	}

	/* wait until transfer is finished */
	while (!(BCM2835_SPICS & BCM_SPI_CS_DONE));

	/* deactivate transfer */
	BCM2835_SPICS = 0;
}

int map_gpio()
{
	int fd;
	static u32 mem1_base, mem2_base;

	mem1_base = BCM2835_GPIO_BASE + BCM2709_OFFSET;
	mem2_base = BCM2835_SPI_BASE + BCM2709_OFFSET;  // SPI na I2C

	fd = open("/dev/mem", O_RDWR | O_SYNC);
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
 *		8		OUT		CE0
 *		9		IN		MISO
 *		10		OUT		MOSI
 *		11		OUT		SCLK
 *
 */

void setup_gpio()
{
	u32 x;

	/* change SPI pins */
	x = BCM2835_GPFSEL0;
	x &= ~(0x3F000000);
	x |= 0x24000000;
	BCM2835_GPFSEL0 = x;

	x = BCM2835_GPFSEL1;
	x &= ~(0x0000003F);
	x |= 0x00000024;
	BCM2835_GPFSEL1 = x;

	/* set up SPI */
	BCM2835_SPICLK = BCM2835_SPICLKDIV;

	BCM2835_SPICS = 0;

	/* clear FIFOs */
	BCM2835_SPICS |= BCM_SPI_CS_CLEAR_RX | BCM_SPI_CS_CLEAR_TX;

}

void restore_gpio()
{
	u32 x;

	/* change SPI pins to inputs*/
	x = BCM2835_GPFSEL0;
	x &= ~(0x3F000000);
	BCM2835_GPFSEL0 = x;

	x = BCM2835_GPFSEL1;
	x &= ~(0x0000003F);
	BCM2835_GPFSEL1 = x;

}
