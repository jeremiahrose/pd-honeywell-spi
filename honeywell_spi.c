/***********************************************************************
 * This header file contains the honeywell_spi class definition.
 * Its main purpose is to communicate with a Honeywell HSC pressure sensor with an SPI interface
 * No calculations or adjustments are done and the output provided is the pure digital data coming
 * the sensor's onboard analog to digital converter.
 * The class contains four variables:
 * mode        -> defines the SPI mode used. In our case it is SPI_MODE_0.
 * bitsPerWord -> defines the bit width of the data transmitted.
 *        This is normally 8. Experimentation with other values
 *        didn't work for me
 * speed       -> Bus speed or SPI clock frequency. According to
 *                https://projects.drogon.net/understanding-spi-on-the-raspberry-pi/
 *            It can be only 0.5, 1, 2, 4, 8, 16, 32 MHz.
 *                Will use 1MHz for now and test it further.
 * spifd       -> file descriptor for the SPI device
 * ****************************************************************************/
#include "m_pd.h"
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

static t_class *honeywell_spi_class;

typedef struct _honeywell_spi
{
    t_object x_obj;
    t_outlet *x_out1; // honeywell pressure
    t_outlet *x_out2; // honeywell temperature
    t_outlet *x_out3; // honeywell status
    t_outlet *x_out4; // spi status
    t_symbol *spidev;
    unsigned char mode;
    unsigned char bitsPerWord;
    unsigned int speed;
    int spifd;
} t_honeywell_spi;

static t_honeywell_spi *honeywell_spi_new(t_symbol *devspi);
static int honeywell_spi_write_read(t_honeywell_spi *spi, unsigned char *data, int length);
static void honeywell_spi_open(t_honeywell_spi *spi, t_symbol *devspi);
static int honeywell_spi_close(t_honeywell_spi *spi);
static void honeywell_spi_free(t_honeywell_spi *spi);

/**********************************************************
 * honeywell_spi_open() :function is called by the "open" command
 * It is responsible for opening the spidev device
 * "devspi" and then setting up the spidev interface.
 * member variables are used to configure spidev.
 * They must be set appropriately before calling
 * this function.
 * *********************************************************/
static void honeywell_spi_open(t_honeywell_spi *spi, t_symbol *devspi){
    int statusVal = 0;
    if (strlen(devspi->s_name) == 0) {
      spi->spidev = gensym("/dev/spidev0.0");
    } else {
      spi->spidev = devspi;
    }
    spi->spifd = open(spi->spidev->s_name, O_RDWR);
    if(spi->spifd < 0) {
      statusVal = -1;
      pd_error(spi, "could not open SPI device");
      goto spi_output;
    }
 
    statusVal = ioctl (spi->spifd, SPI_IOC_WR_MODE, &(spi->mode));
    if(statusVal < 0){
      pd_error(spi, "Could not set SPIMode (WR)...ioctl fail");
      honeywell_spi_close(spi);
      goto spi_output;
    }
 
    statusVal = ioctl (spi->spifd, SPI_IOC_RD_MODE, &(spi->mode));
    if(statusVal < 0) {
      pd_error(spi, "Could not set SPIMode (RD)...ioctl fail");
      honeywell_spi_close(spi);
      goto spi_output;
    }
 
    statusVal = ioctl (spi->spifd, SPI_IOC_WR_BITS_PER_WORD, &(spi->bitsPerWord));
    if(statusVal < 0) {
      pd_error(spi, "Could not set SPI bitsPerWord (WR)...ioctl fail");
      honeywell_spi_close(spi);
      goto spi_output;
    }
 
    statusVal = ioctl (spi->spifd, SPI_IOC_RD_BITS_PER_WORD, &(spi->bitsPerWord));
    if(statusVal < 0) {
      pd_error(spi, "Could not set SPI bitsPerWord(RD)...ioctl fail");
      honeywell_spi_close(spi);
      goto spi_output;
    }  
 
    statusVal = ioctl (spi->spifd, SPI_IOC_WR_MAX_SPEED_HZ, &(spi->speed));    
    if(statusVal < 0) {
      pd_error(spi, "Could not set SPI speed (WR)...ioctl fail");
      honeywell_spi_close(spi);
      goto spi_output;
    }
 
    statusVal = ioctl (spi->spifd, SPI_IOC_RD_MAX_SPEED_HZ, &(spi->speed));    
    if(statusVal < 0) {
      pd_error(spi, "Could not set SPI speed (RD)...ioctl fail");
      honeywell_spi_close(spi);
      goto spi_output;
    }
spi_output:
    if (!statusVal) statusVal = 1;
    else statusVal = 0;
    outlet_float(spi->x_out4, statusVal);
}

/***********************************************************
 * honeywell_spi_close(): Responsible for closing the spidev interface.
 * *********************************************************/
 
static int honeywell_spi_close(t_honeywell_spi *spi){
    int statusVal = -1;
    if (spi->spifd == -1) {
      pd_error(spi, "honeywell_spi: device not open");
      return(-1);
    }
    statusVal = close(spi->spifd);
    if(statusVal < 0) {
      pd_error(spi, "honeywell_spi: could not close SPI device");
      exit(1);
    }
    outlet_float(spi->x_out4, 0);
    spi->spifd = -1;
    return(statusVal);
}

/********************************************************************
 * This function frees the object (destructor).
 * ******************************************************************/
static void honeywell_spi_free(t_honeywell_spi *spi){
    if (spi->spifd == 0) {
      honeywell_spi_close(spi);
    }
}
 
/********************************************************************
 * This function writes data "data" of length "length" to the spidev
 * device. Data shifted in from the spidev device is saved back into
 * "data".
 * ******************************************************************/
static int honeywell_spi_write_read(t_honeywell_spi *spi, unsigned char *data, int length){
 
  struct spi_ioc_transfer spid[length];
  int i = 0;
  int retVal = -1; 
 
// one spi transfer for each byte
 
  for (i = 0 ; i < length ; i++){
 
    spid[i].tx_buf        = (unsigned long)(data + i); // transmit from "data"
    spid[i].rx_buf        = (unsigned long)(data + i); // receive into "data"
    spid[i].len           = sizeof(*(data + i));
    spid[i].delay_usecs   = 0;
    spid[i].speed_hz      = spi->speed;
    spid[i].bits_per_word = spi->bitsPerWord;
    spid[i].cs_change     = 0;
  }
 
  retVal = ioctl(spi->spifd, SPI_IOC_MESSAGE(length), &spid);

  if(retVal < 0){
    pd_error(spi, "problem transmitting spi data..ioctl");
  }

  return retVal;
}

/***********************************************************************
 * Honeywell HSC Pressure sensor enabled external that by default interacts 
 * with /dev/spidev0.0 device using
 * honeywell_spi_MODE_0 (MODE 0) (defined in linux/spi/spidev.h), speed = 1MHz &
 * bitsPerWord=8.
 *
 * on bang call the spi_write_read function on the a2d object and make sure
 * that conversion is configured for single ended conversion on CH0
 * i.e. transmit ->  byte1 = 0b00000001 (start bit)
 *                   byte2 = 0b1000000  (SGL/DIF = 1, D2=D1=D0=0)
 *                   byte3 = 0b00000000  (Don't care)
 *      receive  ->  byte1 = junk
 *                   byte2 = junk + b8 + b9
 *                   byte3 = b7 - b0
 *    
 * after conversion must merge data[1] and data[2] to get final result
 * *********************************************************************/
 
static void honeywell_spi_bang(t_honeywell_spi *spi)
{
  if (spi->spifd == -1) {
    pd_error(spi, "device not open %d", spi->spifd);
    return;
  }

  uint8_t status = 0;
  uint16_t pressure = 0;
  uint16_t temperature = 0;
  unsigned char data[4]; // four bytes for Honeywell

  uint8_t const STATUS_MASK =    0b11000000;
  uint16_t const PRESSURE_MASK = 0b0011111111111111;
  uint16_t const TEMPERATURE_MASK              = 0b1111111111100000;

  data[0] = 0b00000000; // we don't send any data to the sensor,
  data[1] = 0b00000000; // so just initialise with empty bytes.
  data[2] = 0b00000000;
  data[3] = 0b00000000;

  honeywell_spi_write_read(spi, data, sizeof(data));

  // extract vaues from binary data. We have to do bytes separately (unfortunately - maybe there's a better way to access entire con
  status = ((uint8_t) data[0] & STATUS_MASK) >> 6; 
  pressure = (((uint16_t) data[0] << 8) + (uint16_t) data[1]) & PRESSURE_MASK;
  temperature = ((((uint16_t) data[2] << 8) + (uint16_t) data[3]) & TEMPERATURE_MASK) >> 5;

  outlet_float(spi->x_out1, pressure);
  outlet_float(spi->x_out2, temperature);
  outlet_float(spi->x_out3, status);

}

/*************************************************
 * init function.
 * ***********************************************/
static t_honeywell_spi *honeywell_spi_new(t_symbol *devspi){
    t_honeywell_spi *spi = (t_honeywell_spi *)pd_new(honeywell_spi_class);
    //fprintf(stderr,"devspi<%s>\n", devspi->s_name); 
    //t_honeywell_spi *a2d = honeywell_spi_new("/dev/spidev0.0", spi_MODE_0, 1000000, 8);
    spi->x_out1 = outlet_new(&spi->x_obj, gensym("float"));
    spi->x_out2 = outlet_new(&spi->x_obj, gensym("float"));
    spi->x_out3 = outlet_new(&spi->x_obj, gensym("float"));
    spi->x_out4 = outlet_new(&spi->x_obj, gensym("float"));
    spi->spidev = devspi;
    spi->mode = SPI_MODE_0;
    spi->bitsPerWord = 8;
    spi->speed = 1000000;
    spi->spifd = -1;
 
    return(spi);
}


void honeywell_spi_setup(void)
{
    honeywell_spi_class = class_new(gensym("honeywell_spi"), (t_newmethod)honeywell_spi_new,
        (t_method)honeywell_spi_free, sizeof(t_honeywell_spi), 0, A_DEFSYM, 0);
    class_addmethod(honeywell_spi_class, (t_method)honeywell_spi_open, gensym("open"), 
        A_DEFSYM, 0);
    class_addmethod(honeywell_spi_class, (t_method)honeywell_spi_close, gensym("close"), 
        0, 0);
    //class_addfloat(honeywell_gpio_class, honeywell_gpio_float); (later do sending data back to the spi)
    class_addbang(honeywell_spi_class, honeywell_spi_bang);
}
