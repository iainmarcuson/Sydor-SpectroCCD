/*
 * powerenable.h
 *
 *  Created on: Dec 3, 2014
 *      Author: fabecker
 *
 *      Turns on LED 0/GPIO 0 and enables power / GPIO 4
 */

#ifndef LBNL_MEM_H_
#define LBNL_MEM_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "lbnl_typedefs.h"
// Firmware V1
#define		VIDEOMEM_SIZE  16384
#define		VIDEOMEM_ADDR  0x40010000
// Firmware Version 2
//#define		VIDEOMEM_SIZE  262144
//#define		VIDEOMEM_ADDR  0x40040000

#define WAVEMEM_SIZE 1024*4

#define WAVEMEM_ADDR 	0x40000000
#define GPIO_ADDR 		0x41200000
#define CCD_STATE_ADDR	0x43C00000

#define STATUS_OFFSET (31*4)  //status register
#define TYPE_CCDSTATE   0
#define TYPE_GPIO       1
#define TYPE_VIDEOMEM   2
#define TYPE_WAVEMEM    3

/* Function to enable power.
 * GPIO address is hardcoded as well as the register value.
 */

typedef struct {
   int fd;
   void * ptr;
   unsigned int size;
   unsigned int page_offset;	// ptr is aligned to the page size, always use the offset to manipulate
   /* declare as many members as desired, but the entire structure size
   must be known to the compiler. */
} memory;

int power_enable();
int power_disable();
int lvds_master();
int lvds_slave();
int check_power();

int ccd_mem_open(memory * newmemory, unsigned int address, unsigned int size);
int ccd_mem_close(memory * yourmemory);
int ccd_mem_read(memory * mem, unsigned int offset);
void ccd_mem_write(memory * mem, unsigned int offset, int value);

void resetCCDstate();

/* Small wrapper that handles mmap for you
 * Values are hardcoded.
 */
int memWrite(unsigned memAddress, unsigned value);

int memRead(unsigned memAddress);


#endif /* LBNL_MEM_H_ */
