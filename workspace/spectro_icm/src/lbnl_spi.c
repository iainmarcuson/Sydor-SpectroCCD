/*
 * lbnl_spi.c
 *
 *  Created on: Dec 3, 2014
 *      Author: fabecker
 */

#include "lbnl_spi.h"

//int config_spi(char * device, unsigned int mode)
//{
//	int ret = 0;
//	int fd;
//
//	/* VideoSPI config */
//	mode |= SPI_CPHA;
//
//	fd = open(device, O_RDWR);
//	if (fd < 0){
//		return(FAILED);
//	}
//
//	/*
//	 * spi mode
//	 */
//	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
//	if (ret == -1){
//		return(FAILED);
//	}
//
//	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
//	if (ret == -1){
//		return(FAILED);
//	}
//	/*
//	 * bits per word
//	 */
//	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
//	if (ret == -1){
//		return(FAILED);
//	}
//
//	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
//	if (ret == -1){
//		return(FAILED);
//	}
//
//	/*
//	 * max speed hz
//	 */
//	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
//	if (ret == -1){
//		return(FAILED);
//	}
//
//	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
//	if (ret == -1){
//		return(FAILED);
//	}
//
//	close(fd);
//	return 0;
//}

/*
 * Configure the video SPI
 */
int config_video_spi()
{
	int ret = 0;
	int video_spi;
	int mode = SPI_CPHA;
	char* device = SPI_VIDEO;

	uint8_t tx[] = {
			0x00,	// Reset
			0x01,
			DAC_CMD_RESET,
			0x00
		};

	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	video_spi = open(device, O_RDWR);
	if (video_spi < 0){
		return(FAILED);
	}

	/*
	 * spi mode
	 */
	ret = ioctl(video_spi, SPI_IOC_WR_MODE, &mode);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(video_spi, SPI_IOC_RD_MODE, &mode);
	if (ret == -1){
		return(FAILED);
	}
	/*
	 * bits per word
	 */
	ret = ioctl(video_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(video_spi, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1){
		return(FAILED);
	}

	/*
	 * max speed hz
	 */
	ret = ioctl(video_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(video_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1){
		return(FAILED);
	}

	/*
	 * send reset signal to the pic
	 */
	ret = ioctl(video_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 0){
		return(FAILED);
	}

	close(video_spi);
	return(0);
}

/*
 * Configure the Clock SPI
 */
int config_clock_spi()
{
	int ret = 0;
	int clock_spi;
	int mode = SPI_CPHA;
	char* device = SPI_CLOCK;

	uint8_t tx[] = {
			0x00,	// Reset
			0x01,
			DAC_CMD_RESET,
			0x00
		};

	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};


	clock_spi = open(device, O_RDWR);
	if (clock_spi < 0){
		return(FAILED);
	}

	/*
	 * spi mode
	 */
	ret = ioctl(clock_spi, SPI_IOC_WR_MODE, &mode);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(clock_spi, SPI_IOC_RD_MODE, &mode);
	if (ret == -1){
		return(FAILED);
	}
	/*
	 * bits per word
	 */
	ret = ioctl(clock_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(clock_spi, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1){
		return(FAILED);
	}

	/*
	 * max speed hz
	 */
	ret = ioctl(clock_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(clock_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1){
		return(FAILED);
	}

	/*
	 * send reset signal to the pic
	 */
	ret = ioctl(clock_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 0){
		return(FAILED);
	}

	close(clock_spi);
	return(0);
}

/*
 * Configure the temperature sensor (ADC).
 */
int config_temperature_spi()
{
	char* device = SPI_TEMPERATURE;
	unsigned int mode = SPI_CPHA;	// Clock Phase = 1
	int temp_spi;
	int ret=0;

	uint8_t tx[] = {
			0x43,	// wreg 0x00 - 0x11A
			0x01,
			0x04,
			0x55,
			0x80,
			0x08,	// Start/Sync
	};

	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	temp_spi = open(device, O_RDWR);
	if (temp_spi < 0){
		return(FAILED);
	}

	/*
	 * spi mode
	 */
	ret = ioctl(temp_spi, SPI_IOC_WR_MODE, &mode);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(temp_spi, SPI_IOC_RD_MODE, &mode);
	if (ret == -1){
		return(FAILED);
	}
	/*
	 * bits per word
	 */
	ret = ioctl(temp_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(temp_spi, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1){
		return(FAILED);
	}

	/*
	 * max speed hz
	 */
	ret = ioctl(temp_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(temp_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1){
		return(FAILED);
	}

	ret = ioctl(temp_spi, SPI_IOC_MESSAGE(1), &tr);		// Configure the sensor
	if (ret < 0){
		return(FAILED);
	}

	close(temp_spi);
	return(0);
}

int spi_get_temperature(float * temp)
{
	char* device = SPI_TEMPERATURE;
	int temp_spi;
	int ret=0, steps, adc_value;
	float rrtd, rref;
	float val_temp;

	rref=1400;					// reference resistance
	steps = pow(2,15)-1;		// adc steps

	uint8_t tx[] = {
			0x10,	// get data
			0x00,	// receive buffer
			0x00,
	};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	temp_spi = open(device, O_RDWR);
	if (temp_spi < 0){
		return(FAILED);
	}
	ret = ioctl(temp_spi, SPI_IOC_MESSAGE(1), &tr);		// Send and receive
	if (ret < 0){										// message
		return(FAILED);
	}

	adc_value =   rx[2] & 0xff;			// lsb
	adc_value += (rx[1] << 8) & 0xff00;	//msb

	rrtd=(adc_value*rref)/steps;		// calculate resistance of the RTD
	val_temp = T_rtd(rrtd);
	*temp = val_temp;			// calculate the temperature
	//printf("Temperature raw: %x  resistance %f temperature %hf\n", adc_value,rrtd, *temp);
	//printf("Temperature raw: %x  resistance %f temperature %f\n", adc_value,rrtd,(double) *temp);
	printf("Temperature raw: %x  resistance %f temperature %f\n", adc_value,rrtd,*temp);
	close(temp_spi);
	return(0);
}

//int spi_get_clk_temperature(float * temp)
//{
//	char* device = SPI_CLOCK;
//	int temp_spi;
//	int ret=0, rref, steps, adc_value;
//	float rrtd;
//
//	rref=1400;					// reference resistance
//	steps = pow(2,15)-1;		// adc steps
//
//	uint8_t tx[48] = {
//			0,	// set array to zero
//	};
//
//	tx[0] = 0x01;	// send to adc
//	tx[1] = 0x00;
//	tx[2] = 22;		// get 22 values from the pic
//	tx[3] = 0x10;	// get data
//
//	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
//	struct spi_ioc_transfer tr = {
//		.tx_buf = (unsigned long)tx,
//		.rx_buf = (unsigned long)rx,
//		.len = ARRAY_SIZE(tx),
//		.delay_usecs = delay,
//		.speed_hz = speed,
//		.bits_per_word = bits,
//	};
//
//	temp_spi = open(device, O_RDWR);
//	if (temp_spi < 0){
//		return(FAILED);
//	}
//	ret = ioctl(temp_spi, SPI_IOC_MESSAGE(1), &tr);		// Send and receive
//	if (ret < 0){										// message
//		return(FAILED);
//	}
//
//	adc_value =   rx[42] & 0xff;			// lsb
//	adc_value += (rx[43] << 8) & 0xff00;	//msb
//
//	int i;
//	for (i = 0; i <48; i++) {
//		printf("Reg[%d]: %x\n", i, rx[i]);
//	}
//
//	rrtd=(adc_value*rref)/steps;		// calculate resistance of the RTD
//	*temp = T_rtd(rrtd);			// calculate the temperature
//
//	return(0);
//}

/*
 * Transfer the tx buffer and receive the rx buffer.
 * Works only with the pics!
 * Everything is in bytes
 */
int spi_transfer(char * device, uint8_t * tx, uint tx_length, uint8_t * rx, uint rx_length)
{
	int spi;
	int ret;
	spi = open(device, O_RDWR);
	if (spi < 0){
		return(FAILED);
	}

//	//TODO BUFFER check
//	if ((ARRAY_SIZE(tx) < tx_length) | (ARRAY_SIZE(rx) < rx_length)){
//		return(FAILED);
//	}

	// Send
	uint8_t rx_send[tx_length];
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx_send,		//TODO make longer buffer and read back until 0xaa comes
		.len = tx_length,
		.delay_usecs = SPI_DELAY,
		.speed_hz = SPI_SPEED,
		.bits_per_word = SPI_BITS,
	};

	ret = ioctl(spi, SPI_IOC_MESSAGE(1), &tr);		// Send and receive
	if (ret < 0){									// message
		return(SPI_SEND_FAILED);
	}

	// Search start bits
	if (rx_length > 0) {
		uint8_t tx_receive[rx_length];
		memset(tx_receive,0,rx_length*sizeof(uint8_t));
		tr.tx_buf = (unsigned long)tx_receive;
		tr.rx_buf = (unsigned long)rx_send;
		tr.len = 4;		//TODO should be 2

		ret = ioctl(spi, SPI_IOC_MESSAGE(1), &tr);		// Send and receive
		if ((ret < 0) | (rx_send[3] !=0xaa)){			// acknowledge
			int i;
			printf("%s\n", device);
			for (i=0; i<4; i++)
				printf("Reg[%d]=%x\n", i, rx_send[i]);
			return(SPI_ACK_FAILED);
		}
		tr.rx_buf = (unsigned long)rx,
		tr.len = rx_length,

		ret = ioctl(spi, SPI_IOC_MESSAGE(1), &tr);		// Send and receive
		if (ret < 0){									// message
			return(SPI_RECEIVE_FAILED);
		}
	}
	close(spi);
	return(DONE);
}

int spi_checksum_wait(long sleep_nsecs)
{
	struct timespec sleep_time, rem_time;
	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = sleep_nsecs;
	return nanosleep(&sleep_time, &rem_time);
}

void config_adc()
{
	char* device = SPI_VIDEO;
	int fd;
	int ret;

	fd = open(device, O_RDWR);
//	if (fd < 0)
//		pabort("can't open device");

	uint8_t tx[] = {
		0x00,		// Write to ADC over serial interface
		0x03,		//
		0x20,		// ADC_CMD_WRITE
		0x00,		// ADC_CMD_WRITE2
		0x0,		// ADCaddr
		0x25,		// ADCaddr2
		0x00,		//Data MSB	0001 0000
		0x013,		//Data LSB	0000 0000
	};

	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
//	if (ret < 1)
//		pabort("can't send spi message");

//	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
//		if (!(ret % 8))
//			puts("");
//		printf("%.2X ", rx[ret]);
//	}
//	puts("");

	tx[5]= 0x26;
	tx[6]= 0x25;
	tx[7]= 0x58;
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

	close(fd);
}

int spi_deskew_pattern(int enable)
{
	int error;

	uint8_t tx[] = {
		0x00,		// Write to ADC over serial interface
		0x03,		//
		0x20,		// ADC_CMD_WRITE
		0x00,		// ADC_CMD_WRITE2
		0x00,		// ADCaddr
		0x45,		// ADCaddr2
		0x00,		//Data MSB
		0x00,		//Data LSB
	};

	// Enable or disable deskew pattern
	if (enable == 1)
		tx[7] = 0x01;

	error = spi_transfer(SPI_VIDEO, tx, 8, NULL, 0);

	return error;
}

//void configPatternAdc()
//{
//	char* device = SPI_VIDEO;
//	int fd;
//	int ret;
//
//	fd = open(device, O_RDWR);
////	if (fd < 0)
////		pabort("can't open device");
//
//	uint8_t tx[] = {
//		0x03,		// Write to ADC over serial interface
//		0x00,		//
//		0x00,		// ADC_CMD_WRITE
//		0x20,		// ADC_CMD_WRITE2
//		0x26,		// ADCaddr
//		0x00,		// ADCaddr2
////		0x3c,		//Data LSB	0011 1100
////		0xc0		//Data MSB	1100 0000
//		0xc0,		//Data LSB	1100 0000
//		0x3F		//Data MSB	0011 1111
////		0xFC,		//Data LSB	1111 1100
////		0xFF		//Data MSB	1111 1111
//	};
//
//	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
//	struct spi_ioc_transfer tr = {
//		.tx_buf = (unsigned long)tx,
//		.rx_buf = (unsigned long)rx,
//		.len = ARRAY_SIZE(tx),
//		.delay_usecs = delay,
//		.speed_hz = speed,
//		.bits_per_word = bits,
//	};
//
//	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
////	if (ret < 1)
////		pabort("can't send spi message");
//
//	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
//		if (!(ret % 8))
//			puts("");
//		printf("%.2X ", rx[ret]);
//	}
//	puts("");
//
//	close(fd);
//}
