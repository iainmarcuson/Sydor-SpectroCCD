/*
 * lbnl_spi.h
 *
 *  Created on: Dec 3, 2014
 *      Author: fabecker
 */

#ifndef LBNL_SPI_H_
#define LBNL_SPI_H_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <time.h>

#include "lbnl_typedefs.h"
#include "RTDmath.h"

#define SPI_CLOCK "/dev/spidev3.0"
#define SPI_VIDEO  "/dev/spidev2.0"
#define SPI_TEMPERATURE "/dev/spidev1.0"

//static const char *device = "/dev/spidev32764.0";	// clkboard
//static const char *device = "/dev/spidev32765.0";	// Videobaord
//static const char *device = "/dev/spidev32766.0";	// TemperatureSPI

static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay = 20;

#define SPI_BITS 8
#define SPI_SPEED 500000
#define SPI_DELAY 20

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

int config_spi(char * device, unsigned int mode);

int config_video_spi();
int config_clock_spi();
int config_temperature_spi();
int spi_get_temperature(float * temp);
int spi_get_clk_temperature(float * temp);
int spi_transfer(char * device, uint8_t * tx, uint tx_length, uint8_t * rx, uint rx_length);
int spi_deskew_pattern(int enable);
int spi_checksum_wait(long sleep_nsecs);


void config_adc();
void configPatternAdc();

void resetAdc();

void pabort(const char *s);

#endif /* LBNL_SPI_H_ */
