/*
 * powerenable.c
 *
 *  Created on: Dec 3, 2014
 *      Author: fabecker
 */

#include "lbnl_mem.h"

int power_enable()
{
	unsigned address = GPIO_ADDR;
	unsigned value = 0x11;		// Power enable plus LED0 is turned on
	unsigned int size = 1024;
	int error=0;

	memory gpio;
	error = ccd_mem_open(&gpio, address, size);
	value |= ccd_mem_read(&gpio,0);
	ccd_mem_write(&gpio, 0, value);
	error += ccd_mem_close(&gpio);

	return(error);
}
int lvds_master()
{
	unsigned address = GPIO_ADDR;
	unsigned value = 0x100;		//  enable LVDS drivers
	unsigned int size = 1024;
	int error=0;

	memory gpio;
	error = ccd_mem_open(&gpio, address, size);
	value |= ccd_mem_read(&gpio,0);
	ccd_mem_write(&gpio, 0, value);
	error += ccd_mem_close(&gpio);
	return(error);
}

int power_disable()
{
	unsigned address = GPIO_ADDR;
	unsigned value = 0x11;
	unsigned int size = 1024;
	int error=0;

	memory gpio;
	error = ccd_mem_open(&gpio, address, size);
	value = ccd_mem_read(&gpio,0) & ~value;
	ccd_mem_write(&gpio, 0, value);
	error += ccd_mem_close(&gpio);
	return(error);
}

int lvds_slave()
{
	unsigned address = GPIO_ADDR;
	unsigned value = 0x100;
	unsigned int size = 1024;
	int error=0;

	memory gpio;
	error = ccd_mem_open(&gpio, address, size);
	value = ccd_mem_read(&gpio,0) & ~value;
	ccd_mem_write(&gpio, 0, value);
	error += ccd_mem_close(&gpio);
	return(error);
}

int check_power()
{
	unsigned address = GPIO_ADDR;
	unsigned value = 0x10;
	unsigned int size = 1024;
	int error=0;
	memory gpio;

	error = ccd_mem_open(&gpio, address, size);
	value = ccd_mem_read(&gpio,0) & value;
	error += ccd_mem_close(&gpio);
	if (error != 0) {
		return FAILED;
	}
    printf ("check_power: %d\n", value);
	if (value == 0 ){
		return(NO_POWER);
	} else {
		return DONE;
	}
}

//void resetCCDstate(){
//	unsigned address = 0x43C00000;
//	unsigned value = 0x200;
//	value |= memRead(address);
//	memWrite(address, value);
//	struct timespec sleept={0,100};
//	nanosleep(&sleept, NULL);
//	value &= !(0x200);
//	memWrite(address, value);
//}

int ccd_mem_read(memory * mem, unsigned int offset)
{
	int value = 0;
	value = *((unsigned *)(mem->ptr + mem->page_offset + offset));
	return value;
}

void ccd_mem_write(memory * mem, unsigned int offset, int value)
{
	*((unsigned *)(mem->ptr + mem->page_offset + offset)) = value;
}

int ccd_mem_open(memory * mem, unsigned int address, unsigned int size)
{
//	int fd;
//	int mem_size = 16*1024/4; //size of ??
//	void *ptr;

	unsigned page_addr, page_offset;
	unsigned page_size=sysconf(_SC_PAGESIZE);

	mem->size = size;						// Assign the size
	mem->fd = open ("/dev/mem", O_RDWR);
	if (mem->fd < 1) {
		return FAILED;
	}

	page_addr = (address & (~(page_size-1)));	// calculate the page address
	page_offset = address - page_addr;
	mem->ptr = mmap(NULL, mem->size, PROT_READ|PROT_WRITE, MAP_SHARED, mem->fd, page_addr);
	mem->page_offset = page_offset;		// ptr is mapped along the page size
										// add the offset to receive the real address
	return(DONE);

}

int ccd_mem_close(memory * mem)
{
	int error;
	error = munmap(mem->ptr, mem->size);
	if (error != 0)
		return(FAILED);
	close(mem->fd);
	return(DONE);
}

//int memWrite(unsigned memAddress, unsigned value){
//	int fd;
//		unsigned mem_addr = memAddress;
//		int max = 16*1024/4; //size of ??
//
//		unsigned page_addr, page_offset;
//		void *ptr;
//		unsigned page_size=sysconf(_SC_PAGESIZE);
//
//		fd = open ("/dev/mem", O_RDWR);
//		if (fd < 1) {
//			return FAILED;
//		}
//
//		/* mmap the device into memory */
//		page_addr = (mem_addr & (~(page_size-1)));
//		page_offset = mem_addr - page_addr;
//		ptr = mmap(NULL, 16*1024, PROT_READ|PROT_WRITE, MAP_SHARED, fd, page_addr);
//
//		*((unsigned *)(ptr + page_offset)) = value;
//		munmap(ptr, 16*1024);	// don't forget this! releases the memory
//		return DONE;
//}
//
//int memRead(unsigned memAddress){
//	int fd;
//		unsigned mem_addr = memAddress;
//		int max = 16*1024/4;
//
//		unsigned page_addr, page_offset;
//		void *ptr;
//		unsigned page_size=sysconf(_SC_PAGESIZE);
//
//		fd = open ("/dev/mem", O_RDWR);
//		if (fd < 1) {
//			perror("Can't open /dev/mem");
//		}
//
//		/* mmap the device into memory */
//		page_addr = (mem_addr & (~(page_size-1)));
//		page_offset = mem_addr - page_addr;
//		ptr = mmap(NULL, 16*1024, PROT_READ|PROT_WRITE, MAP_SHARED, fd, page_addr);
//
//		int value = 0;
//		value = *((unsigned *)(ptr + page_offset));
//
//		munmap(ptr, 16*1024);	// don't forget this! releases the memory
//		return value;
//}

