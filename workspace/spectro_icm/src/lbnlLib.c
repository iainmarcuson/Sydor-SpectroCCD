/* Library for accessing the LBNL controller driver */
#include <signal.h>

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include "lbnl_prototypes.h"
#include "lbnl_spi.h"
#include "lbnl_mem.h"
#include "fitsio.h"
#include "lbnl_parser.h"
#include "lbnl_params.h"
#include "lbnl_aux.h"

static u32 exposure_time = 0;

//global to track readout progress
static u32 ccd_read_progress = 0;

//globals to cache the last enabled masks
static u32 ccd_dac_mask;
static u32 ccd_clk_mask;
static delays_t gbl_delays;
static int simdata = 0;

//globals for default and current erase and purge parameters
static f32 current_Erase_Vclk_voltage = default_Erase_Vclk_voltage;
static f32 current_Erase_Vsub_voltage = default_Erase_Vsub_voltage;
static i32 current_Erase_slew = default_Erase_slew;
static i32 current_Erase_delay = default_Erase_delay;
static f32 current_Purge_Vclk_voltage = default_Purge_Vclk_voltage;
static i32 current_Purge_delay = default_Purge_delay;
static i32 current_clearcols = default_clearcols;

//globals for timing file control
static u16 read_address_ccd = default_read_address;
static u16 read_up_address_ccd = default_read_up_address;
static u16 read_down_address_ccd = default_read_down_address;
static u16 clear_up_address_ccd = default_clear_up_address;
static u16 clear_down_address_ccd = default_clear_down_address;
static u16 clear_address_ccd = default_clear_address;
static dac_t offsets[NOFFSETS];

//globals for Config Option comments
static char delay_desc[NDELAYS][EPICS_STRLEN];
static char offset_desc[NOFFSETS][EPICS_STRLEN];
static char dac_desc[NDACS][EPICS_STRLEN];
static char clock_desc[NCLOCKS][EPICS_STRLEN];

//globals for Parameter Hi/Los
static f32 clock1_hilo[2][NCLOCKS];
static f32 clock2_hilo[2][NCLOCKS];
static f32 dac_hilo[2][NDACS];
static f32 offset_hilo[2][NOFFSETS];
static int delay_hilo[2][NDELAYS];

//globals for Parameter Values (config only)
static f32 clock1_vals[NCLOCKS];
static f32 clock2_vals[NCLOCKS];
static f32 dac_vals[NDACS];
static f32 offset_vals[NOFFSETS];
static int delay_vals[NDELAYS];



//globals for config
static u32 cfg_gain=5;

//global for array of config parameters
param_t config_params[5];

/* Initializes config_params to point to their data */
void lbnl_register_params()
{
  config_params[0].desc = clock_desc;
  config_params[0].val_array.f = clock1_vals;
  config_params[0].lo_array.f = clock1_hilo[0];
  config_params[0].hi_array.f = clock1_hilo[1];
  config_params[0].num_elem = NCLOCKS;
  config_params[0].format = "%11.5f";
  config_params[0].val_len = 11;
  config_params[0].to_string = lbnl_param_to_string;
  config_params[0].param_type = TYPE_F32;

  config_params[1].desc = clock_desc;
  config_params[1].val_array.f = clock2_vals;
  config_params[1].lo_array.f = clock2_hilo[0];
  config_params[1].hi_array.f = clock2_hilo[1];
  config_params[1].num_elem = NCLOCKS;
  config_params[1].format = "%11.5f";
  config_params[1].val_len = 11;
  config_params[1].to_string = lbnl_param_to_string;
  config_params[1].param_type = TYPE_F32;

  config_params[2].desc = offset_desc;
  config_params[2].val_array.f = offset_vals;
  config_params[2].lo_array.f = offset_hilo[0];
  config_params[2].hi_array.f = offset_hilo[1];
  config_params[2].num_elem = NOFFSETS;
  config_params[2].format = "%11.5f";
  config_params[2].val_len = 11;
  config_params[2].to_string = lbnl_param_to_string;
  config_params[2].param_type = TYPE_F32;
  
  config_params[3].desc = dac_desc;
  config_params[3].val_array.f = dac_vals;
  config_params[3].lo_array.f = dac_hilo[0];
  config_params[3].hi_array.f = dac_hilo[1];
  config_params[3].num_elem = NDACS;
  config_params[3].format = "%11.5f";
  config_params[3].val_len = 11;
  config_params[3].to_string = lbnl_param_to_string;
  config_params[3].param_type = TYPE_F32;

  config_params[4].desc = delay_desc;
  config_params[4].val_array.i = delay_vals;
  config_params[4].lo_array.i = delay_hilo[0];
  config_params[4].hi_array.i = delay_hilo[1];
  config_params[4].num_elem = NDELAYS;
  config_params[4].format = "%11i";
  config_params[4].val_len = 11;
  config_params[4].to_string = lbnl_param_to_string;
  config_params[4].param_type = TYPE_I32;


}
/**
 * opens library and/or controller. Returns reference (driver handle) to the caller
 * @param[in] params are possible open parameters (like device number, or address, or whatever may be required
 **/
dref lbnl_open(char *params)
{
  srand(time(NULL));
  int id = rand();
  FILE * lock_file;
  if (access(LOCKFILE_PATH, F_OK) != -1) {	// Check if file exists
    printf("locked /n");
    return LOCKED;
  }

  lock_file = fopen(LOCKFILE_PATH, "w");
  if (lock_file < 0) {
    return FAILED;
  } else {
    fprintf(lock_file, "%x\n", id);
    fclose(lock_file);
  }
  return id;
};
/**
 * closes library and/or controller.
 * @param[in] fd is the driver reference
 **/
int lbnl_close (dref fd)
{
  int i;
  i = lbnl_check_lock(fd);
  if (i == 0) {
    return unlink(LOCKFILE_PATH);
  } else {
    return i;
  }
}
/**
 *  used by any hardware access function to see if controller instance is available
 * @param[in] fd is the driver reference
 **/
int lbnl_check_lock (dref fd)
{
#if 0
  FILE * lock_file;
  int id;
  lock_file = fopen(LOCKFILE_PATH, "r");
  if (lock_file <= 0) {
    return NOT_OPEN;
  }
  fscanf(lock_file, "%x", &id);
  fclose(lock_file);
  if (id == fd) {
    return OK;
  } else {
    return WRONG_HANDLE;
  }
#else
  return OK;
#endif
}
/**
 *  provided as a point of initial/startup tasks
 * @param[in] fd is the driver reference
 **/
int lbnl_init (dref fd)
{
  int error;

  error = lbnl_check_lock(fd);
  if (error != 0) {
    return error;
  }
  simdata = 0;
  error = check_power();
  if (error != 0) {
    return error;
  }
  printf ("initializing...\n");
  error = config_video_spi();
  error += config_temperature_spi();
  error += config_clock_spi();

  // Load default erase and purge parameters
  current_Erase_Vclk_voltage = default_Erase_Vclk_voltage;
  current_Erase_Vsub_voltage = default_Erase_Vsub_voltage;
  current_Purge_Vclk_voltage = default_Purge_Vclk_voltage;
  current_Erase_slew = default_Erase_slew;
  current_Erase_delay = default_Erase_delay;
  current_Purge_delay = default_Purge_delay;

  // Reset statemachine
  unsigned address = CCD_STATE_ADDR;
  unsigned int size = 1024;
  int value = 0x200;
  memory ccd_statemachine;
  lbnl_sleep_ms(100);
  spi_deskew_pattern(1);
  lbnl_sleep_ms(100);
  error += ccd_mem_open(&ccd_statemachine, address, size);
  value |= ccd_mem_read(&ccd_statemachine, 0);
  ccd_mem_write(&ccd_statemachine, 0, value);
  lbnl_sleep_ms(1);
  ccd_mem_write(&ccd_statemachine, 0, value & 0xfffffdff);
  error += ccd_mem_close(&ccd_statemachine);
  lbnl_sleep_ms(100);
  spi_deskew_pattern(0);
  return (error);
}
/**
 * clears array
 * @param[in] fd is the driver reference
 **/
int lbnl_ccd_clear_up_or_down (dref fd, int up_or_down)
{
  int error = 0;
  int value;
  u32 control_register, status_register;
  u32 timeout =0, size = 1024;
  memory ccd_statemachine;

  error = lbnl_check_lock(fd);
  if (error != 0) {
    return error;
  }
  error  = ccd_mem_open(&ccd_statemachine, CCD_STATE_ADDR, size);
  value = (ccd_mem_read(&ccd_statemachine, 12*4) & 0xffff0000) | (current_clearcols-1);
  ccd_mem_write(&ccd_statemachine, 12*4, value);
  if (up_or_down == 1){
	  value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (clear_up_address_ccd<<16); //set clear start address in timing state machine
  } else if (up_or_down == 2){
	  value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (clear_down_address_ccd<<16); //set clear start address in timing state machine
  } else {
	  value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (clear_address_ccd<<16); //set clear start address in timing state machine
  }
  ccd_mem_write(&ccd_statemachine, 4, value);

  control_register = ccd_mem_read(&ccd_statemachine, 0);
  control_register |= 0x2;
  ccd_mem_write(&ccd_statemachine, 0, control_register);	// trigger clear
  status_register = ccd_mem_read(&ccd_statemachine, STATUS_OFFSET);
  while ( status_register & 0x8){	// CCD idle mode(clear has not started)
    lbnl_sleep_ms(1);
    status_register = ccd_mem_read(&ccd_statemachine, STATUS_OFFSET) ;	// update register
    timeout++;
    if (timeout ==1000){ //increased timeout to 1s for sync
      ccd_mem_write(&ccd_statemachine, 0, control_register &= ~0x2);	// clear trigger bit
      value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (read_address_ccd<<16); //set clear start address in timing state machine
      ccd_mem_write(&ccd_statemachine, 4, value);
      error += ccd_mem_close(&ccd_statemachine);
      return(TIMEOUT);
    }
  }//CCD is now clearing
  ccd_mem_write(&ccd_statemachine, 0, control_register &= ~0x2);	// clear trigger bit
  timeout = 0;
  while (( status_register & 0x8)==0){ // CCD not idle
    lbnl_sleep_ms(10);
    status_register = ccd_mem_read(&ccd_statemachine, STATUS_OFFSET) ;	// update register
    timeout++;
    if (timeout ==10000){ //100s timeout (eternity)
      value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (read_address_ccd<<16); //set clear start address in timing state machine
      ccd_mem_write(&ccd_statemachine, 4, value);
      error += ccd_mem_close(&ccd_statemachine);
      return(TIMEOUT);
    }
  }
  value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (read_address_ccd<<16); //set clear start address in timing state machine
  ccd_mem_write(&ccd_statemachine, 4, value);
  error += ccd_mem_close(&ccd_statemachine);

  return error;
}
/**
 * erases array
 * @param[in] fd is the driver reference
 **/
int lbnl_ccd_erase (dref fd)
{
  int error = 0;
  u16 i;
  f32 Vclk_voltage = current_Erase_Vclk_voltage;
  f32 Vsub_voltage = current_Erase_Vsub_voltage;
  i32 Erase_slew = current_Erase_slew;
  i32 Erase_delay = current_Erase_delay;
  lbnl_clock_t clocks[20];
  u16 nclocks;
  dac_t dacs[20];
  u16 ndacs;

  error = lbnl_controller_get_all_dacs(fd, dacs, &ndacs);
  error += lbnl_controller_get_all_clocks(fd, clocks, &nclocks);
  error += lbnl_controller_set_dac_value(fd,NDACS-1,current_Erase_Vsub_voltage+1);
  for(i=8;i<16;i++){
    error+= lbnl_controller_set_clk_value(fd,i,current_Erase_Vclk_voltage,current_Erase_Vclk_voltage);
  }
  error += lbnl_controller_set_dac_value(fd,NDACS-1,current_Erase_Vsub_voltage);
  printf("ERASING at %f\n", current_Erase_Vclk_voltage);
  lbnl_sleep_ms(current_Erase_delay);
  for(i=1;i<30;i++){
    Vsub_voltage = current_Erase_Vsub_voltage + i*(dacs[NDACS-1].tvalue-current_Erase_Vsub_voltage)/100;
    error+= lbnl_controller_set_dac_value(fd,NDACS-1,Vsub_voltage);
    lbnl_sleep_ms(current_Erase_slew/10);
  }
  for(i=8;i<16;i++){
    error+= lbnl_controller_set_clk_value(fd,i,clocks[i].thigh_value,clocks[i].tlow_value);
  }
  for(i=30;i<100;i++){
    Vsub_voltage = current_Erase_Vsub_voltage + i*(dacs[NDACS-1].tvalue-current_Erase_Vsub_voltage)/100;
    error+= lbnl_controller_set_dac_value(fd,NDACS-1,Vsub_voltage);
    lbnl_sleep_ms(current_Erase_slew/10);
  }
  error+= lbnl_controller_set_dac_value(fd,NDACS-1,dacs[NDACS-1].tvalue);

  //	Vclk_voltage = ((Vclk_voltage/ clk_buffergain));
  //	i32 Vclk_dacCode =
  //			(int) (Vclk_voltage * 65536 / (4 * VREF_BIAS) + 4 * OFFSETVALUE_bias);
  //	Vclk_dacCode = (int) ((Vclk_dacCode - CVALUE_bias + 32768)
  //			* (65536 / (MVALUE_bias + 1)));
  //	if (Vclk_dacCode > 0xffff)
  //		Vclk_dacCode = 0xffff;				// trim to 16 bit
  //	if (Vclk_dacCode < 0)
  //		Vclk_dacCode = 0;					// set to zero
  //
  //	//convert Vsub_DAC from voltage to DAC code
  //	i32 Vsub_dacCode = (int) (Vsub_voltage / clk_hvbuffgain * 65536);	// for bias voltage
  //	if (Vsub_dacCode > 0xffff)
  //		Vsub_dacCode = 0xffff;				// trim to 16 bit
  //	if (Vsub_dacCode < 0)
  //		Vsub_dacCode = 0;							// set to zero
  //
  //	uint8_t tx[] = {0x0, //0x0
  //					5,   // # of words transfered, including PIC command
  //					HV_CMD_ERASE, // PIC command
  //					0x0, //no reply words
  //					(Vclk_dacCode >> 8) & 0xff, (Vclk_dacCode) & 0xff,
  //					(Vsub_dacCode >> 8) & 0xff, (Vsub_dacCode) & 0xff,
  //					(Erase_slew >> 8) & 0xff, (Erase_slew) & 0xff,
  //					(Erase_delay >> 8) & 0xff, (Erase_delay) & 0xff};
  //
  //	error = spi_transfer(SPI_CLOCK,tx,12,NULL,0);
  //
  //	if (error != 0) {
  //		return error;
  //	}
  //
  //	uint8_t tx2[] = {0x0, 0x1, CHECK_ERASE_PURGE, 0};
  //	uint8_t rx[] = {0x0, 0x0};
  //	u32 k = 0;
  //	while (! rx[0]) {
  //		lbnl_sleep_ms(100);
  //		error = spi_transfer(SPI_CLOCK, tx2, 4, rx, 1);
  //
  //		if (error != 0) {
  //			return error;
  //		}
  //
  //		k++;
  //		if (k == 1000) {
  //			return TIMEOUT;
  //		}
  //	}
  return error;
}

/**
 * purges array
 * @param[in] fd is the driver reference
 **/
int lbnl_ccd_purge (dref fd)
{
  int error = 0;
  u16 i;
  f32 clkhi[20];
  f32 clklo[20];
  f32 Vclk_voltage = current_Purge_Vclk_voltage;
  i32 Purge_delay = current_Purge_delay;
  lbnl_clock_t clocks[20];
  u16 nclocks;

  error = lbnl_controller_get_all_clocks(fd, clocks, &nclocks);

  for(i=8;i<16;i++){
    error+= lbnl_controller_set_clk_value(fd,i,current_Purge_Vclk_voltage,current_Purge_Vclk_voltage);
  }
  printf("Purging at %f\n", current_Purge_Vclk_voltage);
  lbnl_sleep_ms(current_Purge_delay);
  for(i=8;i<16;i++){
    error+= lbnl_controller_set_clk_value(fd,i,clocks[i].thigh_value,clocks[i].tlow_value);
  }
  //	error = lbnl_check_lock(fd);
  //	if (error != 0) {
  //		return error;
  //	}
  //
  //	//convert clock DAC voltage to dacCode
  //	f32 Vclk_voltage = current_Purge_Vclk_voltage;
  //	i32 Purge_delay = current_Purge_delay;
  //	Vclk_voltage = (Vclk_voltage/ clk_buffergain);
  //	i32 Vclk_dacCode =
  //			(int) (Vclk_voltage * 65536 / (4 * VREF_BIAS) + 4 * OFFSETVALUE_bias);
  //	Vclk_dacCode = (int) ((Vclk_dacCode - CVALUE_bias + 32768)
  //			* (65536 / (MVALUE_bias + 1)));
  //	if (Vclk_dacCode > 0xffff)
  //		Vclk_dacCode = 0xffff;	// trim to 16 bit
  //	if (Vclk_dacCode < 0)
  //		Vclk_dacCode = 0;		// set to zero
  //
  //	uint8_t tx[] = {0x0,
  //					3,
  //					HV_CMD_PURGE,
  //					0x0,
  //					(Vclk_dacCode >> 8) & 0xff, Vclk_dacCode & 0xff,
  //					(Purge_delay >> 8) & 0xff, Purge_delay & 0xff};
  //
  //	error = spi_transfer(SPI_CLOCK, tx, 8, NULL, 0x0);
  //	if (error != 0) {
  //		return error;
  //	}
  //
  //	uint8_t tx2[] = {0x0, 1, CHECK_ERASE_PURGE, 0};
  //	uint8_t rx[] = {0x0, 0x0};
  //	u32 k = 0;
  //	while (! rx[0]) {
  //		lbnl_sleep_ms(100);
  //		error = spi_transfer(SPI_CLOCK, tx2, 4, rx, 1);
  //		if (error != 0) {
  //			return error;
  //		}
  //
  //		k++;
  //		if (k == 1000) {
  //			return TIMEOUT;
  //		}
  //	}
  return error;
}

/**
 * reads the image buffer
 * @param[in] fd is the driver reference
 * 	up_or_down = 1 read ccd one-way up
 * 	up_or_down = 2 read ccd one-way down
 * 	up_or_down = 3 read ccd interleaved readout
 **/

int lbnl_ccd_read_up_or_down (dref fd, u16 imageData[], int up_or_down)
{
  ccd_read_progress = 0;
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  if (up_or_down == 1) {
	  printf ("lib: reading ccd Up\n");
  } else if (up_or_down == 2) {
	  printf ("lib: reading ccd Down\n");
  } else if (up_or_down == 3) {
	  printf ("lib: reading ccd Interleaved columns\n");
  } else {
	  printf ("lib: need flag for up/down/interleaved read of ccd\n");
	  return FAILED;
  }
  int i;
  u32 control_register, status_register;
  u32 timeout =0, size = 1024;
  u32 rowIndex, columnIndex, pixels, old_bufferstate;
  u16 rows, columns;
  u32 tempdata,offset,index;
  u16 tmpchanna, tmpchannb;
  i16 tmpchannas, tmpchannbs;
  double duration;
  struct timespec tstart={0,0}, tend={0,0};
  memory ccd_statemachine;
  memory videobuffer;
  int value;

#ifdef READ_BENCHMARK
  // Benchmark
  FILE *benchmark;
  if ((benchmark = fopen("/media/sambashare/benchmark.csv", "w")) == NULL ) {
    printf("Failed to open file\n");
    return FAILED;
  }
  fprintf(benchmark, "start,end,div\n");
#endif

  rowIndex=columnIndex=index=0; //reset image location
  lbnl_ccd_get_size(fd, &columns, &rows);
  pixels = ( columns * rows ) / 4;

  error  = ccd_mem_open(&ccd_statemachine, CCD_STATE_ADDR, size);
  error += ccd_mem_open(&videobuffer, VIDEOMEM_ADDR, VIDEOMEM_SIZE);

/* code from the clear that will be changed to use the dummy address */
  //value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (read_down_address_ccd<<16); //set dummy (down read) start address in timing state machine
  //XXX FIXME changed previous line to default_read_down_address
  if (up_or_down == 1) {
	  value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (read_up_address_ccd<<16); //start address in timing state machine
  } else if (up_or_down ==2){
	  value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (read_down_address_ccd<<16); //start address in timing state machine
  } else if (up_or_down ==3){
	  value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (read_address_ccd<<16); //start address in timing state machine
  }
  ccd_mem_write(&ccd_statemachine, 4, value);
  control_register = ccd_mem_read(&ccd_statemachine, 0);
  control_register |= 0x6; /* force the reading out bit on */
  ccd_mem_write(&ccd_statemachine, 0, control_register);	// trigger read
/* end block */

  status_register = ccd_mem_read(&ccd_statemachine, STATUS_OFFSET);
  while ( status_register & 0x8){	// CCD idle mode(read out has not started)
    lbnl_sleep_ms(1);
    status_register = ccd_mem_read(&ccd_statemachine, STATUS_OFFSET) ;	// update register
    timeout++;
    if (timeout ==10000){ //increased timeout to 10s for sync
      ccd_mem_write(&ccd_statemachine, 0, control_register &= ~0x6);	// clear trigger bit
      error += ccd_mem_close(&videobuffer);
      error += ccd_mem_close(&ccd_statemachine);
      printf("lib: timeout at read start =%d/n",timeout);
      return(TIMEOUT);
    }
  }//CCD is now reading out
  ccd_mem_write(&ccd_statemachine, 0, control_register &= ~0x2);	// clear trigger bit

  // poll
  old_bufferstate =0;//readout starts, all buffers empty
  timeout = 0;
  printf ("lib: readout started (%dx%d)\n", columns, rows);
  clock_gettime(CLOCK_MONOTONIC, &tstart);
  while (control_register & 4){ //readout pending
    //	    control_register = ccd_mem_read(&ccd_statemachine, 0);
    //		timeout++;
    //		if (control_register & 0x80){		//if abort bit set
    //			control_register &= ~0x4; // clear pending bit
    //			control_register |= (3<<8);	//set reset state machine bit
    //			ccd_mem_write(&ccd_statemachine, 0, control_register);
    //			spi_checksum_wait(100000); //wait for reset to happen
    //			control_register &= 0xfc7f;		//clear reset state machine, abort bits
    //			ccd_mem_write(&ccd_statemachine, 0, control_register);
    //			error += ccd_mem_close(&videobuffer);
    //			error += ccd_mem_close(&ccd_statemachine);
    //			return(ABORT);
    //		}
    //		if (timeout == 100000){
    //			control_register &= ~0x4; // clear pending bit
    //			control_register |= (3<<8);	//set reset state machine bit
    //			ccd_mem_write(&ccd_statemachine, 0, control_register);
    //			spi_checksum_wait(100000); //wait for reset to happen
    //			control_register &= 0xfc7f;		//clear reset state machine, abort bits
    //			ccd_mem_write(&ccd_statemachine, 0, control_register);
    //			error += ccd_mem_close(&videobuffer);
    //			error += ccd_mem_close(&ccd_statemachine);
    //			return(TIMEOUT);
    //		}
    //		lbnl_sleep_us(1); //sleep to slow hits between register access

    status_register = ccd_mem_read(&ccd_statemachine, STATUS_OFFSET);
    if(  ((status_register & 7) != old_bufferstate)){
      if(((status_register&3)==0 )||((status_register&3)==3 )){ //need to move data from buffer to DDR RAM
	printf("status %d \n",status_register);
      }
      //if buffer state is 0 or 3 the register is not stable, wait and re-read
      timeout = 0;
      old_bufferstate = status_register & 3;
      if(old_bufferstate >1){
	offset =(VIDEOMEM_SIZE/2);	// choose buffer: 0*offset or 1*offset
      }else{
	offset=0;
      }

#ifdef BENCHMARK_READ
      // benchmark start
      struct timespec tstart={0,0}, tend={0,0};
      clock_gettime(CLOCK_MONOTONIC, &tstart);
#endif
      for(i = 0; index < (pixels) && i < (VIDEOMEM_SIZE/16); ++index, i++ , columnIndex++){
	if (columnIndex == (columns>>1)){
	  rowIndex++;
	  ccd_read_progress = 2*rowIndex;
	  columnIndex = 0;
	}

	//				tempdata = *((unsigned *)(videobuffer.ptr + videobuffer.page_offset + offset+(i*8)));
	//				imageData[columns * rows-1 - (columns*rowIndex) - columnIndex] = tempdata&0xffff; //CH1
	//				imageData[columns * (rows-1) - (columns*rowIndex)+ columnIndex] =  tempdata>>16; // CH3 i+1 used for testing quadrant1

	//				tempdata = *((unsigned *)(videobuffer.ptr + videobuffer.page_offset + offset+(i*8)+4));
	//				imageData[rowIndex * columns + columnIndex] =  tempdata>>16; //CH4
	//				imageData[rowIndex * columns + columns - columnIndex -1] = tempdata &0xffff; //CH2

	tempdata = *((u32 *)(videobuffer.ptr + videobuffer.page_offset + offset+(i*8)));
	tmpchanna = tempdata&0xffff;
	tmpchannas = *((i16 *) &tmpchanna);
	tmpchannb = tempdata>>16;
	tmpchannbs = *((i16 *) &tmpchannb);
	//	code before the new controller with spectroccd v0
	//	imageData[columns * rows-1 - (columns*rowIndex) - columnIndex] =  tmpchannas + 32768u; //CH1
	//	imageData[columns * (rows-1) - (columns*rowIndex)+ columnIndex] = tmpchannbs + 32768u; // CH3 i+1 used for testing quadrant1

		// Azriel new variant to correct orientation with new controller and spectroccd V0 Aug 2017
		imageData[columns * rows-1 - (columns*rowIndex) - columnIndex] =  tmpchannas + 32768u;
		imageData[rowIndex * columns + columns - columnIndex -1] = tmpchannbs + 32768u;

		tempdata = *((u32 *)(videobuffer.ptr + videobuffer.page_offset + offset+(i*8)+4));
		tmpchanna = tempdata&0xffff;
		tmpchannas = *((i16 *) &tmpchanna);
		tmpchannb = tempdata>>16;
		tmpchannbs = *((i16 *) &tmpchannb);
	//	code before the new controller with spectroccd v0
	//	imageData[rowIndex * columns + columnIndex] =  tmpchannbs + 32768u; //CH4
	//	imageData[rowIndex * columns + columns - columnIndex -1] = tmpchannas + 32768u; //CH2

		// Azriel new variant to correct orientation with new controller and spectroccd V0 Aug 2017
		imageData[columns * (rows-1) - (columns*rowIndex)+ columnIndex] = tmpchannbs + 32768u;
		imageData[rowIndex * columns + columnIndex] = tmpchannas + 32768u;


      }
#ifdef BENCHMARK_READ
      // Benchmark end
      clock_gettime(CLOCK_MONOTONIC, &tend);
      fprintf(benchmark, "%.9f,%.9f,%.9f\n",
	      (double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec,
	      (double)tend.tv_sec + 1.0e-9*tend.tv_nsec,
	      ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) -
	      ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec));
#endif
     if(status_register & 4){ //image ready bit is set
    	  if(1){//(index <10000){
    		  clock_gettime(CLOCK_MONOTONIC, &tend);
    		  printf( "time to trigger:%.9f ",	((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) -((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec));
    		  printf("timeout=%d,status=%d,control=%d,index=%d/n",timeout,status_register,control_register,index);
    	  }
    	  control_register &= ~(1<<2); // clear pending bit
    	  ccd_mem_write(&ccd_statemachine, 0, control_register);
    	  rowIndex=columnIndex=index=0; //reset image location
     }
    }else{
      lbnl_sleep_us(100); //sleep if no transfer was needed
    }
  }

  error += ccd_mem_close(&videobuffer);
  error += ccd_mem_close(&ccd_statemachine);

#ifdef BENCHMARK_READ
  // Benchmark
  fclose(benchmark);
#endif
  return(error);
}



/**
 * starts exposure and reads the image buffer
 * @param[in] fd is the driver reference
 **/
int lbnl_ccd_read_sim (dref fd, u16 imageData[])
{
  simdata = 1;
  ccd_read_progress = 0;
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  u32 rowIndex, columnIndex, pixels;
  u16 rows, columns,i;
  u32 index;
  u32 pixcnt=0;

  lbnl_ccd_get_size(fd, &columns, &rows);
  pixels = ( columns * rows ) / 4;
  printf ("lib: reading sim data (line delay %d ms", gbl_delays.clock_parallel);
  printf ("lib: size %dx%d ", columns, rows);

  rowIndex=columnIndex=index=i=pixcnt=0; //reset image location
  while (pixcnt < pixels){
    for(i = 0; index < (pixels) && i < (VIDEOMEM_SIZE/16); ++index, i++ , columnIndex++){;//				printf ("colIndex %d ", columnIndex);
      if (columnIndex == (columns>>1)){
	rowIndex++;
	ccd_read_progress = 2 * rowIndex; /*this is quad readout*/
	columnIndex = 0;
	lbnl_sleep_ms (gbl_delays.clock_parallel);
	printf ("rowIndex %d ", rowIndex);
      }
      pixcnt++;

      imageData[columns * rows-1 - (columns*rowIndex) - columnIndex] = 1; //CH1
      imageData[columns * (rows-1) - (columns*rowIndex)+ columnIndex] =  3; // CH3 i+1 used for testing quadrant1

      imageData[rowIndex * columns + columnIndex] =  4; //CH4
      imageData[rowIndex * columns + columns - columnIndex -1] = 2; //CH2


    }
  }
  ccd_read_progress = rows;
  printf ("sim data finished\n");
  simdata = 0;
  return(error);
}

/**
 * This may implement clear, erase and purge at once
 * @param[in] fd is the driver reference
 **/
int lbnl_readout_prepare (dref fd)
{
  int error = lbnl_ccd_clear_up_or_down(fd,3);
  error += lbnl_ccd_erase(fd);
  error += lbnl_ccd_purge(fd);
  return error;
}

/**
 * Discards image already taken (do not download, discards buffer)
 * @param[in] fd is the driver reference
 **/
int lbnl_readout_discard (dref fd)
{
  return (DONE); //TODO
}

/**
 * sets ccd erase parameters (until the next time lbnl_init is called)
 * @param[in] fd is the driver reference
 * @param[in] vclk is the clk voltage in Volts
 * @param[in] vsub is the substrate voltage in Volts
 * @param[in] slew is the slew in ms
 * @param[in] delay is the delay in ms
 **/
int lbnl_ccd_set_erase_params (dref fd, f32 vclk, f32 vsub, i32 slew, i32 delay)
{
  // Allow full range on everything since this is not harmful to the CCD, flexibility is encouraged
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  current_Erase_Vclk_voltage = vclk;
  current_Erase_Vsub_voltage = vsub;
  current_Erase_slew = slew;
  current_Erase_delay = delay;
  return (error);
}

/**
 * get the current erasee parameters.
 * @param[out] fd is the driver reference
 * @param[out] vclk is the current erase Vclk voltage in Volts
 * @param[out] vsub is the current erase Vsub voltage in Volts
 * @param[out] slew is the current erase slew in ms
 * @param[out] delay is the current erase delay delay in ms
 **/
int lbnl_ccd_get_erase_params (dref fd, f32 *vclk, f32 *vsub, i32 *slew, i32 *delay)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  *vclk = current_Erase_Vclk_voltage;
  *vsub = current_Erase_Vsub_voltage;
  *slew = current_Erase_slew;
  *delay = current_Erase_delay;
  return (error);
}
/**
 * sets ccd clear parameters (until the next time lbnl_init is called)
 * @param[in] clearcols is the serial shift count during clear
 **/
int lbnl_ccd_set_clear_params (dref fd,  i32 clearcols)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  current_clearcols = clearcols;
  return (error);
}

/**
 * get the current clear parameters.
 * @param[out] clearcols is the current serial shift count during clear
 **/
int lbnl_ccd_get_clear_params (dref fd,  i32 *clearcols)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  *clearcols = current_clearcols;
  return (error);
}

/**
 * sets ccd purge parameters (until the next time lbnl_init is called)
 * @param[in] fd is the driver reference
 * @param[in] vclk is the clk voltage in Volts
 * @param[in] delay is the delay in ms
 **/
int lbnl_ccd_set_purge_params (dref fd, f32 vclk, i32 delay)
{
  // Allow full range on everything since this is not harmful to the CCD, flexibility is encouraged
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  current_Purge_Vclk_voltage = vclk;
  current_Purge_delay = delay;
  return (error);
}


/**
 * get the current purge parameters.
 * @param[out] fd is the driver reference
 * @param[out] vclk is the current purge Vclk voltage in Volts
 * @param[out] delay is the delay in ms
 **/
int lbnl_ccd_get_purge_params (dref fd, f32 *vclk, i32 *delay)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  *vclk = current_Purge_Vclk_voltage;
  *delay = current_Purge_delay;
  return (error);
}

/**
 * sets ccd total  size, including prescans and overscan for all amps.
 * @param[in] fd is the driver reference
 * @param[in] ncols is the total size in X
 * @param[in] nrows is the total size in Y
 **/
int lbnl_ccd_set_size (dref fd, u16 ncols, u16 nrows)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  unsigned address = CCD_STATE_ADDR;
  unsigned value;
  unsigned int size = 1024;
  memory ccd_statemachine;

  error = ccd_mem_open(&ccd_statemachine, address, size);
  value = (ccd_mem_read(&ccd_statemachine, 10*4) & 0xffff0000) | ((ncols>>1)-1);
  ccd_mem_write(&ccd_statemachine, 10*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 11*4) & 0xffff0000) | ((nrows>>1)-1);
  ccd_mem_write(&ccd_statemachine, 11*4, value);
  error += ccd_mem_close(&ccd_statemachine);
  printf ("lib: detector size set to %dx%d\n", ncols, nrows);

  return (error);
}

/**
 * get ccd total  size, including prescans and overscan for all amps.
 * @param[out] fd is the driver reference
 * @param[out] ncols is the total size in X
 * @param[out] nrows is the total size in Y
 **/
int lbnl_ccd_get_size (dref fd, u16 *ncols, u16 *nrows)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }

  unsigned int size = 1024;
  memory ccd_statemachine;

  error = ccd_mem_open(&ccd_statemachine, CCD_STATE_ADDR, size);
  *ncols = (ccd_mem_read(&ccd_statemachine, 10*4) & 0xffff)*2 +2;
  *nrows = (ccd_mem_read(&ccd_statemachine, 11*4) & 0xffff)*2 +2;
  error += ccd_mem_close(&ccd_statemachine);
  return (error);
}

/**
 * Turn on/off idle mode (blanking waveforms)
 * @param[in] fd is the driver reference
 * @param[in] flag is start_idle(>=1) or stop_idle(<=0)
 **/
int lbnl_ccd_idle (dref fd, i8 flag)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }

  unsigned address = CCD_STATE_ADDR;
  unsigned value = 0x8;		// CCD idle
  unsigned int size = 1024;
  memory ccd_statemachine;

  if (flag >= 1) {
    error = ccd_mem_open(&ccd_statemachine, address, size);
    value |= ccd_mem_read(&ccd_statemachine, 0);
    ccd_mem_write(&ccd_statemachine, 0, value);
    error += ccd_mem_close(&ccd_statemachine);
  } else {
    error = ccd_mem_open(&ccd_statemachine, address, size);
    value = ccd_mem_read(&ccd_statemachine, 0) & ~value;
    ccd_mem_write(&ccd_statemachine, 0, value);
    error += ccd_mem_close(&ccd_statemachine);
  }
  return (error);
}

/**
 * Enables/disables analog power
 * @param[in] fd is the driver reference
 * @param[in] flag is power_on(>=1) or power_off(<=0)
 **/
int lbnl_controller_power (dref fd, i8 flag)
{
  int error = 0;
  error = lbnl_check_lock(fd);	printf ("controller power...(1)\n");
  if (error != 0) {
    return (error);
  }
  if (flag >= 1) {
    error = power_enable();
    lbnl_sleep_ms(100);
  } else {
    error = power_disable();
  }
  return (error);
}

/**
 * Enables/disables LVDS drivers for controller sync.
 * @param[in] fd is the driver reference
 * @param[in] flag is master (>=1) or slave(<=0)
 **/
int lbnl_controller_master (dref fd, i8 flag)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  if (flag >= 1) {
    error = lvds_master();
    lbnl_sleep_ms(100);
  } else {
    error = lvds_slave();
  }
  return (error);
}
/**
 * Enables/disables biases
 * @param[in] fd is the driver reference
 * @param[in] mask is a mask for what channels to enable (1) or disable (0)
 **/
int lbnl_controller_enable (dref fd, u32 dac_mask, u32 clk_mask)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  printf ("lib: enable masks dac 0x%x clk 0x%x\n", dac_mask, clk_mask);
  uint8_t tx_video[] = {	// video pic send buffer
    0x00,
    0x03,
    PIC_CMD_SET_SEGMENT,
    0x00,
    (dac_mask >> 8) & 0x3f,// byteswap
    dac_mask & 0x3f,
    (dac_mask >> 24) & 0x3f,
    (dac_mask >> 16) & 0x3f, };

  uint8_t tx_clock[] = {	// clock pic send buffer
    0x00,
    0x03,
    PIC_CMD_SET_SEGMENT,
    0x00,
    (clk_mask >> 8) & 0x3,// 2 bits	byteswap
    (clk_mask) & 0xff,		// 8 bits
    ((clk_mask >> 18) & 0x3) + ((dac_mask >> 28) & 0x4),// 2 bits clk + HV enable
    (clk_mask >> 10) & 0xff,	// 8 bits
  };

  printf("mask: %x %x \n", dac_mask, clk_mask);

  error = spi_transfer(SPI_CLOCK, tx_clock, 8, NULL, 0);
  if (error != 0) {
    return (error);
  }
  ccd_clk_mask = clk_mask;

  error = spi_transfer(SPI_VIDEO, tx_video, 8, NULL, 0);
  if (error != 0) {
    return (error);
  }
  ccd_dac_mask = dac_mask;

  return (DONE);
}

/**
 * Enables/disables biases
 * @param[in] fd is the driver reference
 * @param[in] mask is a mask for what channels to enable (1) or disable (0)
 **/
int lbnl_controller_set_gain (dref fd, u32 gain)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }

  uint8_t tx_video[] = {	// video pic send buffer
    0x00,
    0x01,
    PIC_CMD_SET_GAIN,
    gain & 0xff
  };

  error = spi_transfer(SPI_VIDEO, tx_video, 4, NULL, 0);
  if (error != 0) {
    return (error);
  }

  //Update gain global
  cfg_gain = gain;

  return (DONE);
}

/**
 * read temperatures
 * @param[in] fd is the driver reference
 * @param[out] temp1 returns temp1 value
 * @param[out] temp2 returns temp2 value
 **/
int lbnl_controller_get_temps (dref fd, f32 *temp1, f32 *temp2)
{
  int error;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return error;
  }

  uint8_t tx[4] = {
    0,	// set array to zero
  };

  tx[0] = 0x00;	// send to adc
  tx[1] = 0x01;
  tx[2] = PIC_CMD_READ_ADC;
  tx[3] = 22;		// get 22 values from the pic //TODO

  uint8_t rx[22 * 2] = { 0, };

  error += spi_transfer(SPI_CLOCK, tx, 4, rx, 44);
  if (error != 0) {
    return error;
  }

  int steps, adc_value;
  float rref;
  float rrtd;
  rref = 1400.0;					// reference resistance
  steps = pow(2, 15) - 1;		// adc steps

  adc_value = rx[43] & 0xff;			// lsb
  adc_value += (rx[42] << 8) & 0xff00;	//msb


  rrtd = (adc_value * rref) / steps;		// calculate resistance of the RTD
  *temp1 = T_rtd(rrtd);				// calculate the temperature
  printf("Temperature raw: %x  resistance %f temperature %f\n", adc_value,rrtd,temp1);

  spi_get_temperature(temp2);
  return (DONE);
}

/**
 * used during readout -copies the data from one of the controller buffers into the provided buffer
 * @param[in] fd is the driver reference
 * @param[out] bufptr returns the next available buffer in memory
 **/
int lbnl_readout_get_data (dref fd, u16 *bufptr)
{
  return (DONE); //TODO
}

/**
 * starts a readout -alternative 1: returns the complete image on memory. This will probbaly use
 * lbnl_readout_get_data() to keep track of the buffers, copying them to the provided memory block
 * @param[in] fd is the driver reference
 * @param[out] imptr returns the complete image in memory. This needs to be typecasted by the caller
 **/
int lbnl_readout_get_memfits (dref fd, void *imptr, u16 image_data[])
{
  //THIS CODE IS UNTESTED
  /*
   * Code from the fits homepage: http://heasarc.gsfc.nasa.gov/docs/software/fitsio/fitsio.html
   */
  fitsfile *fptr;       /* pointer to the FITS file, defined in fitsio.h */
  int status;
  int error =0;
  long  fpixel, nelements;
  u32 exposure;
  u16 rows, columns;

  lbnl_ccd_get_size(fd, &columns, &rows);
  /* initialize FITS image parameters */
  int bitpix   =  USHORT_IMG; /* 16-bit unsigned short pixel values       */
  long naxis    =   2;  /* 2-dimensional image                            */
  long naxes[2] = { columns, rows };   /* image is 300 pixels wide by 200 rows */

  status = 0;         /* initialize status before calling fitsio routines */

  status = ffinit(&fptr, "shmem://h0", &error); /* create new FITS file */

  /* write the required keywords for the primary array image.     */
  /* Since bitpix = USHORT_IMG, this will cause cfitsio to create */
  /* a FITS image with BITPIX = 16 (signed short integers) with   */
  /* BSCALE = 1.0 and BZERO = 32768.  This is the convention that */
  /* FITS uses to store unsigned integers.  Note that the BSCALE  */
  /* and BZERO keywords will be automatically written by cfitsio  */
  /* in this case.                                                */

  if ( fits_create_img(fptr,  bitpix, naxis, naxes, &status) )
    return( status );

  fpixel = 1;                               /* first pixel to write      */
  nelements = naxes[0] * naxes[1];          /* number of pixels to write */

  /* write the array of unsigned integers to the FITS file */
  if ( fits_write_img(fptr, TUSHORT, fpixel, nelements, image_data, &status) )
    return( status );

  /* write another optional keyword to the header */
  /* Note that the ADDRESS of the value is passed in the routine */
  lbnl_controller_get_exptime(fd, &exposure);
  if ( fits_update_key(fptr, TUINT, "EXPOSURE", &exposure,
		       "Total Exposure Time", &status) )
    return( status );

  time_t current_time;
  struct tm* current_time_struct;
  char current_time_char[40];

  current_time = time(NULL);
  current_time_struct = gmtime(&current_time);
  strftime(current_time_char, 40, "%Y-%m-%dT%H:%M:%S", current_time_struct);
  // yyyy-mm-ddTHH:MM:SS

  if ( fits_update_key(fptr, TSTRING, "DATE", current_time_char,
		       "date of file creation in UTC", &status) )
    return( status );

  if ( fits_close_file(fptr, &status) )                /* close the file */
    return( status );

  imptr = fptr;
  return (DONE);

}

/**
 * starts a readout -alternative 2: returns a pointer to an image on disk. Internally it will
 * probably use lbnl_readout_get_memfits(), but will add an extra step to write the fits in memory
 * to the specified location on disk
 * @param[in] fd is the driver reference
 * @param[in] impath is the path to the final fits
 **/
int lbnl_readout_get_fits (dref fd, char *impath, u16 image_data[])
{
  /*
   * Code from the fits homepage: http://heasarc.gsfc.nasa.gov/docs/software/fitsio/fitsio.html
   */
  fitsfile *fptr;       /* pointer to the FITS file, defined in fitsio.h */
  int status;
  int error =0;
  long  fpixel, nelements;
  u32 exposure;
  u16 rows, columns;

  lbnl_ccd_get_size(fd, &columns, &rows);
  /* initialize FITS image parameters */
  int bitpix   =  USHORT_IMG; /* 16-bit unsigned short pixel values       */
  long naxis    =   2;  /* 2-dimensional image                            */
  long naxes[2] = { columns, rows };   /* image is 300 pixels wide by 200 rows */

  remove(impath);               /* Delete old file if it already exists */

  status = 0;         /* initialize status before calling fitsio routines */

  status = ffinit(&fptr, impath, &error); /* create new FITS file */

  /* write the required keywords for the primary array image.     */
  /* Since bitpix = USHORT_IMG, this will cause cfitsio to create */
  /* a FITS image with BITPIX = 16 (signed short integers) with   */
  /* BSCALE = 1.0 and BZERO = 32768.  This is the convention that */
  /* FITS uses to store unsigned integers.  Note that the BSCALE  */
  /* and BZERO keywords will be automatically written by cfitsio  */
  /* in this case.                                                */

  if ( fits_create_img(fptr,  bitpix, naxis, naxes, &status) )
    return( status );

  //DEBUGGING REMOVETHIS
  printf("Past fits_create_image\n");

  fpixel = 1;                               /* first pixel to write      */
  nelements = naxes[0] * naxes[1];          /* number of pixels to write */

  /* write the array of unsigned integers to the FITS file */
  if ( fits_write_img(fptr, TUSHORT, fpixel, nelements, image_data, &status) )
    {
      printf("fits_write_img status value is %i.\n",status);
      return( status );
    }

  //DEBUGGING REMOVETHIS
  printf("Past fits_write_img\n");

  /* write another optional keyword to the header */
  /* Note that the ADDRESS of the value is passed in the routine */
  lbnl_controller_get_exptime(fd, &exposure);
  if ( fits_update_key(fptr, TUINT, "EXPOSURE", &exposure,
		       "Total Exposure Time", &status) )
    return( status );

  //DEBUGGING REMOVETHIS
  printf("Past fits_updata_key EXPOSURE\n");

  time_t current_time;
  struct tm* current_time_struct;
  char current_time_char[40];

  current_time = time(NULL);
  current_time_struct = gmtime(&current_time);
  strftime(current_time_char, 40, "%Y-%m-%dT%H:%M:%S", current_time_struct);
  // yyyy-mm-ddTHH:MM:SS

  if ( fits_update_key(fptr, TSTRING, "DATE", current_time_char,
		       "date of file creation in UTC", &status) )
    return( status );

  //DEBUGGING REMOVETHIS
  printf("Past fits_update_key DATE\n");

  if ( fits_close_file(fptr, &status) )                /* close the file */
    return( status );

  printf("Past fits_close_file\n");
  return (DONE);
}

/**
 * get status of the controller, including readout progress
 * @param[in] fd is the driver reference
 * @param[out] status returns the status structure
 **/
int lbnl_controller_get_status (dref fd, status_t *contstat)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }

  unsigned address = CCD_STATE_ADDR;
  unsigned value = 0x8;		// CCD idle
  unsigned int size = 1024;
  memory ccd_statemachine;

  if (simdata)
    contstat->ccd_idle = 0;
  else {
    error = ccd_mem_open(&ccd_statemachine, address, size);
    //	int ccd_status = ccd_mem_read(&ccd_statemachine, 0);
    int ccd_status = ccd_mem_read(&ccd_statemachine, STATUS_OFFSET);
    contstat->ccd_idle = ((ccd_status & value) != 0);
    error += ccd_mem_close(&ccd_statemachine);
  }

  contstat->power_on = (i8) check_power();
  contstat->clk_mask = ccd_clk_mask;
  contstat->dac_mask = ccd_dac_mask;
  //    printf ("lib: returning masks dac 0x%x clk 0x%x\n", contstat->dac_mask,contstat->clk_mask);
  printf ("idle: %d\n", contstat->ccd_idle);

  
  return (error); //TODO
}

/**
 * get readout status progress.
 * @param[in] fd is the driver reference
 * @param[out] readstat returns the status structure
 **/
int lbnl_readout_get_status (dref fd, readout_t *readstat)
{
  readstat->rows_read = ccd_read_progress;
  u16 rows, cols;
  lbnl_ccd_get_size(fd, &cols, &rows);
  if (rows == 0)
  {
	  readstat->progress = 0;
  }
  else
  {
	  readstat->progress = (ccd_read_progress * 100) / rows;
  }
  return 0;
  //TODO
}

/**
 * uploads a timing file/waveforms to the controller
 * @param[in] fd is the driver reference
 * @param[in] timingpath is the path to the timing file to upload
 **/
int lbnl_controller_upload_timing (dref fd, char *timingpath)
{
  FILE *timing_file;
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return error;
  }
  //    printf ("lib: about to upload %s\n", timingpath);
  if ((timing_file = fopen(timingpath, "r")) == NULL ) {
    printf("File %s can't be opened\n", timingpath);
    return FAILED;
  }
  memory wave_mem;
  error = ccd_mem_open(&wave_mem, WAVEMEM_ADDR, WAVEMEM_SIZE);
  if (error != 0) {
    return error;
  }
  char line[256];
  char *addr_ptr, *data_ptr;
  int addr = -1;			// Start at address 0x0
  unsigned int data = 0;
  while (fgets(line, sizeof(line), timing_file)) {
    addr_ptr = strstr(line, "@");
    data_ptr = strstr(line, "|");
    if (data_ptr) {		// Don't scan line if there is no data in it
      if (addr_ptr == NULL ) {
	addr++;		// If no address is given, increment the old one
      } else {
	sscanf(addr_ptr + 1, "%d", &addr);
      }
      //			printf("Address: %d\t", addr);

      sscanf(data_ptr + 1, "%x", &data);
      //			printf("0x%x %08x\n", addr, data);
      ccd_mem_write(&wave_mem, addr * 4, data);// Byte addressing! Multiply address by 4
    }
  }

  error = ccd_mem_close(&wave_mem);
  fclose(timing_file);

  return (error);
}

/**
 * uploads a config file to the controller
 * @param[in] fd is the driver reference
 * @param[in] cfgpath is the path to the timing file to upload
 **/
int lbnl_controller_upload_config (dref fd, char *cfgpath)
{
  FILE *cfg_file;
  int error = 0;
  char line[256];
  char *DAC_ptr, *CLK_ptr, *OFF_ptr, *TIM_ptr, *data_ptr, *sec_ptr, *comment_ptr, *lo_ptr, *hi_ptr;
  int addr = -1;			// Start at address 0x0
  int data=0;
  f32 fdata = 0.0;
  f32 sec = 0;
  int i;
  int delays[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int curr_line_number = 0;

  error = lbnl_check_lock(fd);
  if (error != 0) {
    return error;
  }
  //    printf ("lib: about to upload %s\n", timingpath);
  if ((cfg_file = fopen(cfgpath, "r")) == NULL ) {
    printf("File %s can't be opened\n", cfgpath);
    return FAILED;
  }

  //TODO Put into function
  //Initialize DELAY DESC strings with spaces
  for (i=0; i<16; i++)
    {
      delay_desc[i][0]='\0';
      strcpy(delay_desc[i]," ");
    }

  for (i=0; i<8; i++)
    {
      offset_desc[i][0]='\0';
      strcpy(offset_desc[i]," ");
    }

  for (i=0; i<17; i++)
    {
      dac_desc[i][0]='\0';
      strcpy(dac_desc[i]," ");
    }

  for (i=0; i<20; i++)
    {
      clock_desc[i][0]='\0';
      strcpy(clock_desc[i]," ");
    }
  while (fgets(line, sizeof(line), cfg_file))
    {
      curr_line_number++;
      //DAC_ptr = strstr(line, "D");
      //CLK_ptr = strstr(line, "C");
      //OFF_ptr = strstr(line, "O");
      //TIM_ptr = strstr(line, "T");

      DAC_ptr = line[0] == 'D' ? line : NULL;
      CLK_ptr = line[0] == 'C' ? line : NULL;
      OFF_ptr = line[0] == 'O' ? line : NULL;
      TIM_ptr = line[0] == 'T' ? line : NULL;

      data_ptr = strstr(line, "|");
      sec_ptr = strstr(line, "/");
      comment_ptr = strstr(line, "#");
      lo_ptr = strstr(line,"@");
      hi_ptr = strstr(line,"&");
		

      if (data_ptr) {		// Don't scan line if there is no data in it
	if ((lo_ptr == NULL) || (hi_ptr == NULL))
	  {
	    printf("No LO and HI values specified in line %i, skipping.\n", curr_line_number);
	    continue;
	  }
	if (DAC_ptr != NULL ) {
	  sscanf(DAC_ptr + 1, "%d", &addr);
	  sscanf(data_ptr + 1, "%f", &fdata);
	  sscanf(lo_ptr + 1, "%f", &(dac_hilo[0][addr]));
	  sscanf(hi_ptr + 1, "%f", &(dac_hilo[1][addr]));

	  printf("DAC addr %d, data %f\n",addr,fdata);
	  error += lbnl_controller_set_dac_value(fd, addr, fdata);
	  /* Put the dac value into the array */
	  /* FIXME TODO XXX Add bounds checking */
	  dac_vals[addr] = fdata;
	
	  //Now parse the comment field
	  if (comment_ptr !=  NULL)
	    {
	      //strcpy(delay_desc[addr],comment_ptr+1);
	      sscanf(comment_ptr+1,"%[^\r\n]\r\n", dac_desc[addr]);
	      printf("DAC addr %d, comment %s\n",addr,dac_desc[addr]);
	    }
	  else
	    {
	      strcpy(dac_desc[addr]," ");
	    }
	}
	if ((CLK_ptr != NULL ) & (sec_ptr != NULL )) {
	  sscanf(CLK_ptr + 1, "%d", &addr);
	  sscanf(data_ptr + 1, "%f", &fdata);
	  sscanf(sec_ptr + 1, "%f", &sec);
	  sscanf(lo_ptr+1,"%f,%f", &(clock1_hilo[0][addr]), &(clock2_hilo[0][addr]));
	  sscanf(hi_ptr+1,"%f,%f", &(clock1_hilo[1][addr]), &(clock2_hilo[1][addr]));
	  printf("CLK addr %d, data %f, %f\n",addr,fdata,sec);
	  error += lbnl_controller_set_clk_value(fd, addr, fdata, sec);

	  /* Put the clock value into the array */
	  /* FIXME TODO XXX Add bounds checking */
	  clock1_vals[addr] = fdata;
	  clock2_vals[addr] = sec;
			      		 
	  //Now parse the comment field
	  if (comment_ptr !=  NULL)
	    {
	      //strcpy(delay_desc[addr],comment_ptr+1);
	      sscanf(comment_ptr+1,"%[^\r\n]\r\n", clock_desc[addr]);
	      printf("CLOCK addr %d, comment %s\n",addr,clock_desc[addr]);
	    }
	  else
	    {
	      strcpy(clock_desc[addr]," ");
	    }
	}
	if (OFF_ptr != NULL ) {
	  sscanf(OFF_ptr + 1, "%d", &addr);
	  sscanf(data_ptr + 1, "%f", &fdata);
	  sscanf(lo_ptr+1, "%f", &(offset_hilo[0][addr]));
	  sscanf(hi_ptr+1, "%f", &(offset_hilo[1][addr]));
	  printf("OFF addr %d, data %f\n",addr,fdata);
	  error += lbnl_controller_set_offset_value (fd, addr, fdata);

	  /* Put the offset value into the array */
	  /* FIXME TODO XXX Add bounds checking */
	  offset_vals[addr] = fdata;

	  //Now parse the comment field
	  if (comment_ptr !=  NULL)
	    {
	      //strcpy(delay_desc[addr],comment_ptr+1);
	      sscanf(comment_ptr+1,"%[^\r\n]\r\n", offset_desc[addr]);
	      printf("OFF addr %d, comment %s\n",addr,offset_desc[addr]);
	    }
	  else
	    {
	      strcpy(offset_desc[addr]," ");
	    }
	}
	if (TIM_ptr != NULL ) {
	  sscanf(TIM_ptr + 1, "%d", &addr);
	  sscanf(data_ptr + 1, "%d", &data);
	  sscanf(lo_ptr+1, "%i", &(delay_hilo[0][addr]));
	  sscanf(hi_ptr+1, "%i", &(delay_hilo[1][addr]));
	  printf("TIM addr %d, data %d\n",addr,data);
	  delays[addr] = data;

	  /* Put the delay value into the array */
	  /* FIXME TODO XXX Add bounds checking */
	  delay_vals[addr] = data;

	  //Now parse the comment field
	  if (comment_ptr !=  NULL)
	    {
	      //strcpy(delay_desc[addr],comment_ptr+1);
	      sscanf(comment_ptr+1,"%[^\r\n]\r\n", delay_desc[addr]);
	      printf("TIM addr %d, comment %s\n",addr,delay_desc[addr]);
	    }
	  else
	    {
	      strcpy(delay_desc[addr]," ");
	    }
	}
      }
    }

  fclose(cfg_file);

  /*
   * Delays
   */
  delays_t ccd_delay;
  if(delays[1]==0)
    ccd_delay.clock_parallel = 1000;
  else	ccd_delay.clock_parallel = delays[1];
  if(delays[2]==0)
    ccd_delay.clock_reset = 5;
  else		ccd_delay.clock_reset = delays[2];
  if(delays[3]==0)
    ccd_delay.clock_serial = 22;
  else	ccd_delay.clock_serial = delays[3];
  if(delays[4]==0)
    ccd_delay.clock_sumwell = 30;
  else	ccd_delay.clock_sumwell = delays[4];
  if(delays[5]==0)
    ccd_delay.settling_reset = 30;
  else	ccd_delay.settling_reset = delays[5];
  if(delays[6]==0)
    ccd_delay.settling_signal = 30;
  else	ccd_delay.settling_signal = delays[6];
  ccd_delay.other1 = 7;
  ccd_delay.other2 = 7;
  ccd_delay.other3 = 7;
  ccd_delay.other4 = 7;
  error = lbnl_controller_set_delays(fd, ccd_delay);
  printf("set_delays error code: %d\n", error);

  /*
   * CDS
   */
  cds_t cds;
  if(delays[10]==0)
    cds.averaging = 9;
  else	cds.averaging = delays[10];
  if(delays[11]==0)
    cds.digioff = 2000;
  else	cds.digioff = delays[11];
  if(delays[12]==0)
    cds.nsamp_reset = 512;
  else	cds.nsamp_reset = delays[12];
  if(delays[13]==0)
    cds.nsamp_signal = 512;
  else	cds.nsamp_signal = delays[13];
  error = lbnl_controller_set_cds(fd, cds);
  printf("set_cds error code: %d\n", error);
  if(delays[15]==0)
    cfg_gain = 5;
  else	cfg_gain = delays[15];
  error = lbnl_controller_set_gain(fd, cfg_gain);
  printf("set gain gain: %d error code: %d\n", cfg_gain,error);

  // Enable controller and set start address (timing file)
  //	error = lbnl_controller_enable(dfd, DACMASK, CLKMASK);
  //	printf("enable error code: %i\n", error);
  error = lbnl_controller_set_start_address(fd, 32);
  printf("set_start_address error code: %i\n", error);


  return (error);
}


/**
 * uploads a timing file/waveforms to the controller
 * @param[in] fd is the driver reference
 * @start_address[in] is the location in the timing file of the readout routine to use.
 **/
int lbnl_controller_set_start_address (dref fd, u16 start_address)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return error;
  }

  unsigned address = CCD_STATE_ADDR;
  unsigned int size = 1024;
  int value;
  memory ccd_statemachine;
  start_address &= 0x3ff;
  error = ccd_mem_open(&ccd_statemachine, address, size);
  value = (ccd_mem_read(&ccd_statemachine, 4) & 0xfc00ffff)   | (start_address<<16);
  ccd_mem_write(&ccd_statemachine, 4, value);
  error += ccd_mem_close(&ccd_statemachine);

  return (error);

}

/*IMPORTANT: the next ones may not be required if the external caller will implement
 * them externally
 */

/**
 * sets the exposure time. If the controller implements a proper timer, it may go to the
 * controller, if not, it may implement a software timer (directly in the function)
 * @param[in] fd is the driver reference
 * @param[in] exptime is the requested exposure time, in msecs
 **/
int lbnl_controller_set_exptime (dref fd, u32 exptime)
{
  exposure_time = exptime;
  return (DONE);
}

/**
 * gets the exposure time. If the controller implements a proper timer, it may go to the
 * controller, if not, it may return the last set value (local global value)
 * @param[in] fd is the driver reference
 * @param[out] exptime is the current exposure time, in msecs
 **/
int lbnl_controller_get_exptime (dref fd, u32 *exptime)
{
  *exptime = exposure_time;
  return (DONE);
}

/**
 * controls the automatic shutter handling. If the controller implements this, it may go
 * go to the controller, otherwise it may implement some internal logic
 * @param[in] fd is the driver reference
 * @param[in] autoshut tells if shutter is handled automatically on exposure (1) or not (0)
 **/
int lbnl_controller_set_autoshutter (dref fd, i8 autoshutter)
{
  return (DONE);
}

/**
 * returns the automatic shutter handling. If the controller implements this, it may go
 * go to the controller, otherwise it may implement some internal logic
 * @param[in] fd is the driver reference
 * @param[out] autoshut returns autoshutter value (1 opened/closed, 0, untouched)
 **/
int lbnl_controller_get_autoshutter (dref fd, i8 *autoshutter)
{
  return (DONE);
}

/**
 * opens or closes the shutter manually.
 * @param[in] fd is the driver reference
 * @param[in] shut is shutter_open (0) or shutter_close(1)
 **/
int lbnl_controller_set_shutter (dref fd, i8 shut)
{
  unsigned int address = GPIO_ADDR;
  u32 value = 0x20;		// shutter is on gpio(5)--
  unsigned int size = 1024;
  u32 regval;
  int error=0;
  
  value = (value * shut)& 0x60; //allow aux shutter on gpio(6)
  memory gpio;
  error = ccd_mem_open(&gpio, address, size);
  regval = ccd_mem_read(&gpio,0);
  regval = (regval & 0xffffff9f)|value; //clear shutter bits, overwrite with shut value
  ccd_mem_write(&gpio, 0, regval);
  error += ccd_mem_close(&gpio);
  return(error);
  
}

/**
 * get shutter status. This may go to the controller if implemented, otherwise
 * return the last global shutter state
 * @param[in] fd is the driver reference
 * @param[out] shut return if shutter_open (0) or shutter_close(1)
 **/
int lbnl_controller_get_shutter (dref fd, i8 *shut)
{
  unsigned int address = GPIO_ADDR;
  unsigned int size = 1024;
  u32 regval;
  int error=0;
  
  memory gpio;
  error = ccd_mem_open(&gpio, address, size);
  regval = ccd_mem_read(&gpio,0);
  error += ccd_mem_close(&gpio);
  *shut = (regval >> 5)&0x3;
  return(error);

}

int lbnl_controller_read_aux(dref fd, u32 *regval)
{
  unsigned int port_addr = GPIO_ADDR;
  unsigned int size = 1024;
  int error = 0;
  volatile memory gpio;

  error = ccd_mem_open(&gpio, port_addr, size);
  *regval = ccd_mem_read(&gpio,0);
  error += ccd_mem_close(&gpio);
  return error;
}
  
  
/*IMPORTANT: the next ones are low level, engineering ones, and they may not be 
 * implemented by all callers -this is, only the engineering-oriented interfaces
 * but not necessarily the run-only clients. Some may require knowledge and safety
 * measures by the caller*/

/**
 * Downloads the current timing file/waveforms from the controller
 * @param[in] fd is the driver reference
 * @param[in] timingarray is the path where the timing file will be dowloaded
 **/
int lbnl_controller_download_timing (dref fd, u32 *timingarray)
{
  int error;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return error;
  }

  if (ARRAY_SIZE(timingarray) < WAVEMEM_SIZE) {
    return (FAILED);
  }

  memory waveform;
  error = ccd_mem_open(&waveform, WAVEMEM_ADDR, WAVEMEM_SIZE);
  if (error != 0)
    return error;

  memcpy(timingarray, waveform.ptr + waveform.page_offset, waveform.size);
  error = ccd_mem_close(&waveform);
  if (error != 0)
    return error;

  return (DONE);
}

/**
 * get number of total DACs. This may be used to allocate memory for the next call
 * which will actually get the dacs
 * names. Of course this could be changed to use DAC indexes, etc
 * @param[in] fd is the driver reference
 * @param[out] ndacs returns the number of DACs on the controller
 **/
int lbnl_controller_get_ndacs (dref fd, u16 *ndacs)
{
  *ndacs = NDACS;	// only dacs no clocks
  return (DONE);
}

/**
 * get number of total DACs. This may be used to allocate memory for the next call
 * which will actually get the dacs
 * names. Of course this could be changed to use DAC indexes, etc
 * @param[in] fd is the driver reference
 * @param[out] ndacs returns the number of DACs on the controller
 **/
int lbnl_controller_get_noffsets (dref fd, u16 *noff)
{
  *noff = NOFFSETS;	// only offsets no bias or clocks
  return (DONE);
}

/**
 * get number of total Clocks. This may be used to allocate memory for the next call
 * which will actually get the dacs
 * names. Of course this could be changed to use DAC indexes, etc
 * @param[in] fd is the driver reference
 * @param[out] nclocks returns the number of clocks on the controller
 **/
int lbnl_controller_get_nclocks (dref fd, u16 *nclocks)
{
  *nclocks = NCLOCKS;	// clocks only
  return (DONE);
}

/**
 * get current DACs values.
 * @param[in] fd is the driver reference
 * @param[out] dacs returns an array of DACs structures
 * @param[out] ndacs returns the number of DACs returned
 **/
int lbnl_controller_get_all_dacs (dref fd, dac_t *dacs, u16 *ndacs)
{
  int error = 0;
  int i, binval, channel_address, checksum, k = 0;
  printf ("getting dacs...\n");
  error = lbnl_check_lock(fd);
  if (error != 0) {
    printf("Error checking lock.\n");
    return (error);
  }

  //TODO bufferoverflow check

  *ndacs = NDACS;					// Number of clocks

  uint8_t tx[(NDACS - 1) * 8 + 4];// 4 bytes for every high/low clock + 4 command bytes - high voltage
  tx[0] = 0x00;					// clk pic send buffer: 4 bytes
  tx[1] = (NDACS - 1) * 4 + 1;	// Send 4 bytes for every channel (high + low) + 1 command word
  tx[2] = DAC_CMD_READ;
  tx[3] = 0x00;

  for (i = 1; i <= ((NDACS - 1) * 2); i++) {// Set the readback commands: 40*4 bytes
    channel_address = 15 + i; //DAC0-7 broadcast, 8-15 offsets
    tx[i * 4] = 0;
    tx[i * 4 + 1] = (DAC_SPECIAL_FUNCTIONS_MODE | DAC_SF_READBACK);
    tx[i * 4 + 2] = X1A_REGISTER + ((channel_address & 0x3E) >> 1);
    tx[i * 4 + 3] = ((channel_address & 0x01) << 7);
  }

  error = spi_transfer(SPI_VIDEO, tx, 4 + 8 * (NDACS - 1), NULL, 0);
  if (error != 0)
    {
      printf("Error in first SPI transfer.\n");
      return (error);
    }

  tx[0] = 0x00;			// video pic pic send buffer: 4 bytes
  tx[1] = 0x01;
  tx[2] = CHECK_DAC_READBACK;
  tx[3] = 0x00;
  checksum = 1;
  uint8_t rx_checksum[2];				// checksum is only two bytes
  while (checksum != 0) {				//wait for read back to finish
    lbnl_sleep_ms(100);	// too many interrupts cause the PIC to stall
    error = spi_transfer(SPI_VIDEO, tx, 4, rx_checksum, 2);
    if (error != 0)
      {
	printf("Error in second SPI transfer.\n");
	return (error);
      }
    checksum = rx_checksum[0];
    k++;
    if (k >= 500) {	// timeout
      break;
    }
  }

  tx[0] = 0x00;			// clk pic send buffer: 4 bytes
  tx[1] = 0x01;
  tx[2] = PIC_CMD_READ_ADC;
  tx[3] = (NDACS - 1) * 2;	// 16 DAC values + Offset Bias values
  uint8_t rx_receive[(NDACS - 1) * 4] = { 0 };	// words*2 = bytes
  error = spi_transfer(SPI_VIDEO, tx, 4, rx_receive, (NDACS - 1) * 4);// Send
  if (error != 0)
    {
      printf("Error in third SPI transfer.\n");
      return (error);
    }

  for (i = 0; i < (NDACS - 1); ++i) {
    dacs[i].address = i;
    binval = rx_receive[i * 2 + 1] & 0xff;
    binval += (rx_receive[i * 2] << 8) & 0x0f00;
    dacs[i].telemetry = biasAdcGains[i] * (binval - biasAdcOffset[i]);
    dacs[i].raw_telemetry = binval;
    binval = rx_receive[i * 2 + 1 + (NDACS - 1) * 2] & 0xff;
    binval += (rx_receive[i * 2 + (NDACS - 1) * 2] << 8) & 0xff00;
    dacs[i].raw_value = binval;
    dacs[i].tvalue = (binval - 4 * OFFSETVALUE_bias) * biasBuffGains[i]
      / 65536 * (4 * VREF_BIAS);
  }

  tx[0] = 0x00;	// send to adc
  tx[1] = 0x01;
  tx[2] = PIC_CMD_READ_ADC;
  tx[3] = 63;		// get 22 values from the pic

  uint8_t rx[63 * 2] = { 0, };

  error += spi_transfer(SPI_CLOCK, tx, 4, rx, 63 * 2);
  if (error != 0) {
    printf("Error in fourth SPI transfer.\n");
    return error;
  }
  binval = rx[41] & 0xff;				// lsb
  binval += (rx[40] << 8) & 0x0f00;	// msb
  dacs[NDACS - 1].telemetry = clk_hv_adc_gain * (binval - clk_hv_adc_offset);
  dacs[NDACS - 1].raw_telemetry = binval;
  binval = rx[125] & 0xff;			// lsb
  binval += (rx[124] << 8) & 0xff00;	// msb
  dacs[NDACS - 1].raw_value = binval;
  dacs[NDACS - 1].tvalue =  (binval * clk_hvbuffgain)/65536;
  dacs[NDACS - 1].address = NDACS - 1;

  return (DONE);
}

/**
 * get current offset values.
 * @param[in] fd is the driver reference
 * @param[out] dacs returns an array of DACs structures
 * @param[out] noff returns the number of offsets returned
 **/
int lbnl_controller_get_all_offsets (dref fd, dac_t *dacs, u16 *noff)
{
  int error = 0;
  int i, binval, channel_address, checksum, k = 0;
  printf ("getting offsets...\n");
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }

  //TODO bufferoverflow check

  *noff = NOFFSETS;					// Number of clocks

  uint8_t tx[(NOFFSETS) * 8 + 4];// 4 bytes for every high/low clock + 4 command bytes
  tx[0] = 0x00;					// clk pic send buffer: 4 bytes
  tx[1] = (NOFFSETS) * 4 + 1;	// Send 4 bytes for every channel (high + low) + 1 command word
  tx[2] = DAC_CMD_READ;
  tx[3] = 0x00;

  for (i = 1; i <= (NOFFSETS * 2); i++) {// Set the readback commands: 40*4 bytes
    channel_address = 7 + i; //DAC0-7 broadcast, 8-15 offsets
    tx[i * 4] = 0;
    tx[i * 4 + 1] = (DAC_SPECIAL_FUNCTIONS_MODE | DAC_SF_READBACK);
    tx[i * 4 + 2] = X1A_REGISTER + ((channel_address & 0x3E) >> 1);
    tx[i * 4 + 3] = ((channel_address & 0x01) << 7);
  }

  error = spi_transfer(SPI_VIDEO, tx, 4 + 8 * (NOFFSETS), NULL, 0);
  if (error != 0)
    return (error);

  tx[0] = 0x00;			// video pic pic send buffer: 4 bytes
  tx[1] = 0x01;
  tx[2] = CHECK_DAC_READBACK;
  tx[3] = 0x00;
  checksum = 1;
  uint8_t rx_checksum[2];				// checksum is only two bytes
  while (checksum != 0) {				//wait for read back to finish
    lbnl_sleep_ms(100);	// too many interrupts cause the PIC to stall
    error = spi_transfer(SPI_VIDEO, tx, 4, rx_checksum, 2);
    if (error != 0)
      return (error);
    checksum = rx_checksum[0];
    k++;
    if (k >= 500) {	// timeout
      break;
    }
  }

  tx[0] = 0x00;			// clk pic send buffer: 4 bytes
  tx[1] = 0x01;
  tx[2] = PIC_CMD_READ_ADC;
  tx[3] = (NOFFSETS) * 2;	// 16 DAC values + Offset Bias values
  uint8_t rx_receive[(NOFFSETS) * 4] = { 0 };	// words*2 = bytes
  error = spi_transfer(SPI_VIDEO, tx, 4, rx_receive, (NOFFSETS) * 4);// Send
  if (error != 0)
    return (error);

  for (i = 0; i < (NOFFSETS); ++i) {
    dacs[i].address = i;
    binval = rx_receive[i * 2 + 1] & 0xff;
    binval += (rx_receive[i * 2] << 8) & 0x0f00;
    dacs[i].telemetry = biasAdcGains[i] * (binval - biasAdcOffset[i]);
    dacs[i].raw_telemetry = binval;
    binval = rx_receive[i * 2 + 1 + (NOFFSETS) * 2] & 0xff;
    binval += (rx_receive[i * 2 + (NOFFSETS) * 2] << 8) & 0xff00;
    dacs[i].raw_value = binval;
    dacs[i].tvalue = (binval - 4 * OFFSETVALUE_bias) * biasBuffGains[i]
      / 65536 * (4 * VREF_BIAS);
    printf ("off[%d] %f (%d), telem %f %d\n", i, dacs[i].tvalue, dacs[i].raw_value, dacs[i].telemetry, dacs[i].raw_telemetry);
  }
	
  for (i = 0; i < (NOFFSETS); ++i) {
    dacs[i].tvalue = offsets[i].tvalue;
  }

  return (DONE);
}

/**
 * get current Clock values.
 * @param[in] fd is the driver reference
 * @param[out] clocks returns an array of DACs structures
 * @param[out] nclocks returns the number of DACs returned
 **/
int lbnl_controller_get_all_clocks (dref fd, lbnl_clock_t *clocks, u16 *nclocks)
{
  int error = 0;
  int i, adcval, channel_address, checksum, k = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }

  *nclocks = NCLOCKS;			// Number of clocks

  uint8_t tx[NCLOCKS * 8 + 4];// 4 bit for every high/low clock + 4 command bytes
  tx[0] = 0x00;			// clk pic send buffer: 4 bytes
  tx[1] = NCLOCKS * 4 + 1;// Send 4 bytes for every channel (high + low) + 1 command word
  tx[2] = DAC_CMD_READ;
  tx[3] = 0x00;

  for (i = 1; i <= (NCLOCKS * 2); i++) {// Set the readback commands: 40*4 bytes
    channel_address = 7 + i;
    tx[i * 4] = 0;
    tx[i * 4 + 1] = (DAC_SPECIAL_FUNCTIONS_MODE | DAC_SF_READBACK);
    tx[i * 4 + 2] = X1A_REGISTER + ((channel_address & 0x3E) >> 1);
    tx[i * 4 + 3] = ((channel_address & 0x01) << 7);
  }

  error = spi_transfer(SPI_CLOCK, tx, 4 + 8 * NCLOCKS, NULL, 0);
  if (error != 0)
    return (error);

  tx[0] = 0x00;			// clk pic send buffer: 4 bytes
  tx[1] = 0x01;
  tx[2] = CHECK_DAC_READBACK;
  tx[3] = 0x00;
  checksum = 1;
  uint8_t rx_checksum[2];				// checksum is only two bytes
  while (checksum != 0) {				// wait for read back to finish
    lbnl_sleep_ms(100);// too many interrupts cause the PIC to stall
    error = spi_transfer(SPI_CLOCK, tx, 4, rx_checksum, 2);
    if (error != 0)
      return (error);
    checksum = rx_checksum[0];
    k++;
    if (k >= 500) {	// timeout
      break;
    }
  }

  tx[0] = 0x00;			// clk pic send buffer: 4 bytes
  tx[1] = 0x01;
  tx[2] = PIC_CMD_READ_ADC;
  tx[3] = NCLOCKS * 3 + 3;	//plus 3 words from HV
  uint8_t rx_receive[NCLOCKS * 6 + 6] = { 0 };
  error = spi_transfer(SPI_CLOCK, tx, 4, rx_receive, NCLOCKS * 6 + 6);// Send
  if (error != 0)
    return (error);

  // [ADC values][1 word HV][1 word temperature]
  // [DAC high + low values][1 word HV]
  for (i = 0; i < NCLOCKS; ++i) {
    clocks[i].address = i;
    adcval = rx_receive[i * 2 + 1] & 0xff;
    adcval += (rx_receive[i * 2] << 8) & 0x0f00;
    clocks[i].telemetry = clk_adc_gain * (adcval - clk_adc_offset);
    clocks[i].raw_telemetry = adcval;
    adcval = rx_receive[i * 4 + 1 + NCLOCKS * 2 + 4] & 0xff;
    adcval += (rx_receive[i * 4 + NCLOCKS * 2 + 4] << 8) & 0xff00;
    clocks[i].low_raw_value = adcval;
    clocks[i].tlow_value = (adcval - 4 * OFFSETVALUE_clk) * clk_buffergain
      / 65536 * (4 * VREF_CLK);
    adcval = rx_receive[i * 4 + 3 + NCLOCKS * 2 + 4] & 0xff;
    adcval += (rx_receive[i * 4 + 2 + NCLOCKS * 2 + 4] << 8) & 0xff00;
    clocks[i].high_raw_value = adcval;
    clocks[i].thigh_value = (adcval - 4 * OFFSETVALUE_clk) * clk_buffergain
      / 65536 * (4 * VREF_CLK);
  }
  return (DONE);
}

/**
 * get an specific DAC values. This is based on the current controller, which require DAC
 * names. Of course this could be changed to use DAC indexes, etc
 * @param[in] fd is the driver reference
 * @param[in] address of requested dac
 * @param[out] dac returns the DAC structure. If not found, function returns error
 **/
int lbnl_controller_get_dac (dref fd, u16 address, dac_t *dac)
{
  return (DONE);
}

/**
 * set an specific DAC value. This is based on the current controller, which require DAC
 * names. Of course this could be changed to use DAC indexes, etc. Safety measures
 * should be implemented in the caller
 * @param[in] fd is the driver reference
 * @param[in] address is the address of requested dac
 * @param[value] value is the value for the requested DAC
 **/
int lbnl_controller_set_dac_value (dref fd, u16 address, f32 value)
{
  int error = 0;
  i32 dacCode;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }

  if (address > (NDACS - 1))
    return (FAILED);

  //	printf ("lib: setting dac %d to %f\n", address, value);
  if (address == (NDACS - 1)) {	// High voltage DAC (on Clk board)
    //		value = ((value/biasBuffGains[address]));//gain array starts at 0
    //		dacCode = (int)( value * 65536/(4*VREF_BIAS) + 4*OFFSETVALUE_bias ) ;
    //		dacCode = (int)((dacCode - CVALUE_bias + 32768)* (65536/(MVALUE_bias+1)));

    dacCode = (int) (value / clk_hvbuffgain * 65536);	// for bias voltage
    if (dacCode > 0xffff)
      dacCode = 0xffff;				// trim to 16 bit
    if (dacCode < 0)
      dacCode = 0;							// set to zero

    uint8_t tx[] = {	// video pic send buffer
      0x00, 0x02, HV_CMD_WRITE, 0x00, (dacCode >> 8) & 0xff, (dacCode)
      & 0xff, };

    error = spi_transfer(SPI_CLOCK, tx, 6, NULL, 0);
    if (error != 0) {
      return (error);
    }
  } else {
    value = ((value / biasBuffGains[address]));	// gain array starts at 0
    dacCode =
      (int) (value * 65536 / (4 * VREF_BIAS) + 4 * OFFSETVALUE_bias);
    dacCode = (int) ((dacCode - CVALUE_bias + 32768)
		     * (65536 / (MVALUE_bias + 1)));

    if (dacCode > 0xffff)
      dacCode = 0xffff;	// trim to 16 bit
    if (dacCode < 0)
      dacCode = 0;		// set to zero

    uint8_t tx[] = {	// video pic send buffer
      0x00,
      0x03,
      DAC_CMD_WRITE,
      0x00,
      0x00,
      DAC_INPUT_WRITE_MODE + address + 16, // 8 broadcast, 8 offsets
      (dacCode >> 8) & 0xff,
      (dacCode) & 0xff,
    };

    error = spi_transfer(SPI_VIDEO, tx, 8, NULL, 0);
    if (error != 0) {
      return (error);
    }
  }
  return (DONE);

}

/**
 * set the DACs to the values specified in $CONTROLLER_DEFAULTS_DIR/dac_vals
 * @param[fd] fd is the driver reference
 */
int lbnl_controller_set_default_dacs(dref fd) {
  char* defaults_dir = getenv("CONTROLLER_DEFAULTS_DIR");
  if (defaults_dir == NULL) {
    return ENV_VAR_DOESNT_EXIST;
  }
  char* filename = calloc(100, sizeof(char));
  strcat(filename, defaults_dir);
  strcat(filename, "/dac_vals");

  f32 dacvals[NDACS];

  int error = 0;
  error = parse(filename, false, NULL, dacvals);
  if (error != 0) {
    return error;
  }

  u16 i = 0;
  for (; i < NDACS; i++) {
    error = lbnl_controller_set_dac_value(fd, i, dacvals[i]);
    if (error != 0) {
      return error;
    }
  }
  return error;
}

/**
 * set an specific offset value.
 * Offsets are on the video pic and have the addresses 0 - 7.
 * @param[in] fd is the driver reference
 * @param[in] address is the address of requested offset
 * @param[value] value is the value for the requested DAC
 **/
int lbnl_controller_set_offset_value (dref fd, u16 address, f32 value)
{
  int error = 0;
  u16 dacCode;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }

  if (address > NOFFSETS - 1)
    return (FAILED);


  dacCode = (int) (value * 65536 / (4 * VREF_BIAS) + 4 * OFFSETVALUE_bias);
  printf ("lib: setting offset %d to %f (%d)", address, value, dacCode);
  dacCode = (int) ((dacCode - CVALUE_bias + 32768)
		   * (65536 / (MVALUE_bias + 1)));
  //	printf (" (%d)\n", dacCode);
  uint8_t tx[] = {	// video pic send buffer
    0x00,
    0x03,
    DAC_CMD_WRITE,
    0x00,
    0x00,
    DAC_INPUT_WRITE_MODE + address + 8,
    (dacCode >> 8) & 0xff,
    (dacCode) & 0xff,
  };

  error = spi_transfer(SPI_VIDEO, tx, 8, NULL, 0);
  if (error != 0) {
    return (error);
  }
  offsets[address].tvalue = value;
  return (DONE);
}

/**
 * set the offset values to the values specified
 * in $CONTROLLER_DEFAULTS_DIR/offset_vals
 * @param[fd] fd is the driver reference
 */
int lbnl_controller_set_default_offsets(dref fd) {
  char* defaults_dir = getenv("CONTROLLER_DEFAULTS_DIR");
  if (defaults_dir == NULL) {
    return ENV_VAR_DOESNT_EXIST;
  }
  char* filename = calloc(100, sizeof(char));
  strcat(filename, defaults_dir);
  strcat(filename, "/offset_vals");

  f32 offset_vals[NOFFSETS];

  int error = 0;
  error = parse(filename, false, NULL, offset_vals);
  if (error != 0) {
    return error;
  }

  u16 i = 0;
  for (; i < NOFFSETS; i++) {
    error = lbnl_controller_set_offset_value(fd, i, offset_vals[i]);
    if (error != 0) {
      return error;
    }
  }
  return error;
}

/**
 * set an specific clock value. Throws away the HV DAC value.
 * @param[in] fd is the driver reference
 * @param[in] address is the address of requested clock
 * @param[value] value is the value for the requested DAC
 **/
int lbnl_controller_set_clk_value (dref fd, u16 address, f32 high_value, f32 low_value)
{
  int error = 0;
  i32 highCode, lowCode;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  highCode = (int) (high_value / clk_buffergain * 65536 / (4 * VREF_CLK) + 4 * OFFSETVALUE_clk); //gain is always 4.7 for clocks
  highCode = (int) ((highCode - CVALUE_clk + 32768) * (65536 / (MVALUE_clk + 1)));
  if (highCode > 0xffff) highCode = 0xffff;	// trim to 16 bit
  if (highCode < 0) highCode = 0;		// trim to 16 bit
  lowCode = (int) (low_value / clk_buffergain * 65536 / (4 * VREF_CLK) + 4 * OFFSETVALUE_clk); //gain is always 4.7 for clocks
  lowCode = (int) ((lowCode - CVALUE_clk + 32768) * (65536 / (MVALUE_clk + 1)));
  if (lowCode > 0xffff) lowCode = 0xffff;	// trim to 16 bit
  if (lowCode < 0) lowCode = 0;			// trim to 16 bit

  //	printf("Address: %d\tHigh: %x\tLow:\t%x\n", address, highCode, lowCode);

  uint8_t tx[] = {	// clk pic send buffer
    0x00,
    0x05,
    DAC_CMD_WRITE,
    0x00,
    0x00,
    DAC_INPUT_WRITE_MODE + address * 2 + 9, //high value in upper DAC
    (highCode >> 8) & 0xff,
    (highCode) & 0xff,
    0x00,
    DAC_INPUT_WRITE_MODE + address * 2 + 8,
    (lowCode >> 8) & 0xff,
    (lowCode) & 0xff,
  };

  error = spi_transfer(SPI_CLOCK, tx, 12, NULL, 0);
  if (error != 0) {
    return (error);
  }

  return (DONE);
}

/**
 * set the clock values to the values specified
 * in $CONTROLLER_DEFAULTS_DIR/clk_vals
 * @param[fd] fd is the driver reference
 */
int lbnl_controller_set_default_clks(dref fd) {
  char* defaults_dir = getenv("CONTROLLER_DEFAULTS_DIR");
  if (defaults_dir == NULL) {
    return ENV_VAR_DOESNT_EXIST;
  }
  char* filename = calloc(100, sizeof(char));
  strcat(filename, defaults_dir);
  strcat(filename, "/clk_vals");

  f32 clk_vals[NCLOCKS * 2];

  int error = 0;
  error = parse(filename, false, NULL, clk_vals);
  if (error != 0) {
    return error;
  }

  u16 i = 0;
  for (; i < NDACS; i++) {
    error = lbnl_controller_set_clk_value(fd, i, clk_vals[i*2], clk_vals[i*2+1]);
    if (error != 0) {
      return error;
    }
  }

  return error;
}

/**
 * set an specific DAC value. Same as previous, but alternatively the caller
 * passes the complete  DAC structure.This could be useful if the DAC structure
 * has more than the dac valiue and name, but limits, etc
 * @param[in] fd is the driver reference
 * @param[in] dac has the target infoirmation (name/value)
 **/
int lbnl_controller_set_dac_params (dref fd, dac_t dac)
{
  return (DONE);
}

/*Note: we could make the same set of "dacs" command with "clocks", in case we want to
 * differentiate them. A clock level -high and low- could be considered a DAC and be
 * treated no differenly than a bias DAC.
 * The alternative is to make functions as "set_clock_value" that passes a strcuture
 * like clock_t that has the low and high clocl levels (so setting both dacs)
 */

/**
 * write a memory register.
 * @param[in] fd is the driver reference
 * @param[in] address is the register address
 * @param[in] regval has the target infoirmation (name/value)
 **/
int lbnl_controller_write_register (dref fd, u16 type, dptr_t address, data_t regval)

{
  return (DONE);
}

/**
 * reads a memory register.
 * @param[in] fd is the driver reference
 * @param[in] address is the register address
 * @param[out] regval  returns the memory register structure
 **/
int lbnl_controller_read_register (dref fd, u16 type, dptr_t address, data_t *regval)
{
  u32 memaddress=CCD_STATE_ADDR;
  u32 size=1024;
  memory mem_fd;
  int error;

  if (type == TYPE_VIDEOMEM){
    memaddress = VIDEOMEM_ADDR;
    size = VIDEOMEM_SIZE;
  }
  if (type == TYPE_WAVEMEM){
    memaddress = WAVEMEM_ADDR;
    size = WAVEMEM_SIZE;
  }
  if (type == TYPE_GPIO){
    memaddress = GPIO_ADDR;
  }

  error = ccd_mem_open(&mem_fd, memaddress, size);
  *regval = ccd_mem_read(&mem_fd, address*4);
  //	printf ("reg 0x%x %d 0x%x", memaddress, address, *regval);
  error += ccd_mem_close(&mem_fd);

  return (error);
}

/**
 * passes a raw command to the controller. IN the current controller this could be curl command
 * IN the new one this could be an IOCTL code, etc
 * @param[in] fd is the driver reference
 * @param[in] command is the requested command
 * @param[in] response is the command response
 **/
int lbnl_controller_pass (dref fd, char *command, u32 *response)
{
  return (DONE);
}

/**
 * gets digital correlated double sampling parameters
 * @param[in] fd is the driver reference
 * @param[out] cds is the structure response
 **/
int lbnl_controller_get_cds (dref fd, cds_t *cds)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  unsigned address = CCD_STATE_ADDR;
  //	unsigned value;
  unsigned int size = 1024;
  memory ccd_statemachine;

  error = ccd_mem_open(&ccd_statemachine, address, size);
  cds->nsamp_signal = (ccd_mem_read(&ccd_statemachine, 20*4) & 0xffff0000) >> 16;
  cds->nsamp_reset = (ccd_mem_read(&ccd_statemachine, 21*4) & 0xffff0000) >> 16;
  cds->digioff =       ccd_mem_read(&ccd_statemachine, 1*4)  & 0xffff;
  cds->averaging = (ccd_mem_read(&ccd_statemachine, 1*4) & 0xf0000000) >> 28;
  error += ccd_mem_close(&ccd_statemachine);

  return (error);
}

/**
 * sets digital cds parameters. A negative value for a field in the structure
 * may imply that that particular parameter remains untouched
 * @param[in] fd is the driver reference
 * @param[in] cds is the structure with the desired parameters
 **/
int lbnl_controller_set_cds (dref fd, cds_t cds)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  unsigned address = CCD_STATE_ADDR;
  unsigned value = 0x8;		// CCD idle
  unsigned int size = 1024;
  memory ccd_statemachine;

  //	cds.nsamp_reset  -= 3;		// error correction, was calculated
  //	cds.nsamp_signal -= 3;		// with a fixed pattern

  error = ccd_mem_open(&ccd_statemachine, address, size);
  value = (ccd_mem_read(&ccd_statemachine, 20*4) & 0xffff) | (cds.nsamp_signal<<16);
  ccd_mem_write(&ccd_statemachine, 20*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 21*4) & 0xffff) | (cds.nsamp_reset<<16);
  ccd_mem_write(&ccd_statemachine, 21*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 1*4) & 0xffff0000) | (cds.digioff);
  ccd_mem_write(&ccd_statemachine, 1*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 1*4) & 0x0fffffff) | ((cds.averaging & 0xf)<<28);
  ccd_mem_write(&ccd_statemachine, 1*4, value);
  error += ccd_mem_close(&ccd_statemachine);

  return (error);
}
/**
 * set the CDS values to the values specified
 * in $CONTROLLER_DEFAULTS_DIR/cds_vals
 * @param[fd] fd is the driver reference
 */
int lbnl_controller_set_default_cds(dref fd) {
  char* defaults_dir = getenv("CONTROLLER_DEFAULTS_DIR");
  if (defaults_dir == NULL) {
    return ENV_VAR_DOESNT_EXIST;
  }
  char* filename = calloc(100, sizeof(char));
  strcat(filename, defaults_dir);
  strcat(filename, "/cds_vals");

  u16 cds_vals[4]; // size of cds_t struct

  int error = 0;
  error = parse(filename, true, cds_vals, NULL);
  if (error != 0) {
    return error;
  }


  return (DONE);
}

/**
 * gets various waveform delays parameters
 * @param[in] fd is the driver reference
 * @param[out] delays is the structure response
 **/
int lbnl_controller_get_delays (dref fd, delays_t *delay)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  unsigned address = CCD_STATE_ADDR;
  unsigned int size = 1024;
  memory ccd_statemachine;

  error = ccd_mem_open(&ccd_statemachine, address, size);
  delay->settling_signal = (ccd_mem_read(&ccd_statemachine, 20*4) & 0xffff);
  delay->settling_reset = (ccd_mem_read(&ccd_statemachine, 21*4) & 0xffff);
  delay->clock_serial = (ccd_mem_read(&ccd_statemachine, 22*4) & 0xffff);
  delay->clock_sumwell = (ccd_mem_read(&ccd_statemachine, 23*4) & 0xffff);
  delay->clock_reset = (ccd_mem_read(&ccd_statemachine, 24*4) & 0xffff) ;
  delay->clock_parallel = (ccd_mem_read(&ccd_statemachine, 25*4) & 0xffff);
  delay->other1 = (ccd_mem_read(&ccd_statemachine, 26*4) & 0xffff);
  delay->other2 = (ccd_mem_read(&ccd_statemachine, 27*4) & 0xffff);
  delay->other3 = (ccd_mem_read(&ccd_statemachine, 28*4) & 0xffff);
  delay->other4 = (ccd_mem_read(&ccd_statemachine, 29*4) & 0xffff);
  error += ccd_mem_close(&ccd_statemachine);
  printf ("get: o1= %d, o2= %d, o3=%d, o4= %d\n", delay->other1, delay->other2, delay->other3, delay->other4);

  return (DONE);
}

/**
 * sets delays parameters. A negative value for a field in the structure
 * may imply that that particular delay remains untouched
 * @param[in] fd is the driver reference
 * @param[in] delays is the structure with the desired parameters
 **/
int lbnl_controller_set_delays (dref fd, delays_t delay)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  unsigned address = CCD_STATE_ADDR;
  unsigned value = 0x8;		// CCD idle
  unsigned int size = 1024;
  memory ccd_statemachine;
  gbl_delays = delay;
  printf ("o1= %d, o2= %d, o3=%d, o4= %d\n", delay.other1, delay.other2, delay.other3, delay.other4);
  error = ccd_mem_open(&ccd_statemachine, address, size);
  value = (ccd_mem_read(&ccd_statemachine, 20*4) & 0xffff0000) | delay.settling_signal;
  ccd_mem_write(&ccd_statemachine, 20*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 21*4) & 0xffff0000) | delay.settling_reset;
  ccd_mem_write(&ccd_statemachine, 21*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 22*4) & 0xffff0000) | delay.clock_serial;
  ccd_mem_write(&ccd_statemachine, 22*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 23*4) & 0xffff0000) | delay.clock_sumwell;
  ccd_mem_write(&ccd_statemachine, 23*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 24*4) & 0xffff0000) | delay.clock_reset;
  ccd_mem_write(&ccd_statemachine, 24*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 25*4) & 0xffff0000) | delay.clock_parallel;
  ccd_mem_write(&ccd_statemachine, 25*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 26*4) & 0xffff0000) | delay.other1;
  ccd_mem_write(&ccd_statemachine, 26*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 27*4) & 0xffff0000) | delay.other2;
  ccd_mem_write(&ccd_statemachine, 27*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 28*4) & 0xffff0000) | delay.other3;
  ccd_mem_write(&ccd_statemachine, 28*4, value);
  value = (ccd_mem_read(&ccd_statemachine, 29*4) & 0xffff0000) | delay.other4;
  ccd_mem_write(&ccd_statemachine, 29*4, value);
  error += ccd_mem_close(&ccd_statemachine);

  return (error);
}

/**
 * set the delays to the values specified
 * in $CONTROLLER_DEFAULTS_DIR/delays_vals
 * @param[fd] fd is the driver reference
 */
int lbnl_controller_set_default_delays(dref fd) {
  char* defaults_dir = getenv("CONTROLLER_DEFAULTS_DIR");
  if (defaults_dir == NULL) {
    return ENV_VAR_DOESNT_EXIST;
  }
  char* filename = calloc(100, sizeof(char));
  strcat(filename, defaults_dir);
  strcat(filename, "/delay_vals");

  u16 delays_vals[10]; // size of cds_t struct

  int error = 0;
  error = parse(filename, true, delays_vals, NULL);
  if (error != 0) {
    return error;
  }

  delays_t delays;
  delays.settling_signal = delays_vals[0];
  delays.settling_reset = delays_vals[1];
  delays.clock_reset = delays_vals[2];
  delays.clock_sumwell = delays_vals[3];
  delays.clock_reset = delays_vals[4];
  delays.clock_parallel = delays_vals[5];
  delays.other1 = delays_vals[6];
  delays.other2 = delays_vals[7];
  delays.other3 = delays_vals[8];
  delays.other4 = delays_vals[9];

  error = lbnl_controller_set_delays(fd, delays);
  return error;
}



/**
 * sets delays parameters. A negative value for a field in the structure
 * may imply that that particular delay remains untouched
 * @param[in] delays is the structure with the desired parameters
 **/
int lbnl_sleep_ms (long miliseconds)
{
  int error;
  struct timespec sleep_time, rem_time;
  sleep_time.tv_sec = 0;
  sleep_time.tv_nsec = miliseconds * 1000000;
  error = nanosleep(&sleep_time, &rem_time);
  if (error)
    printf("Sleep: %d\t rem time: %ld %ld\n", error, rem_time.tv_sec, rem_time.tv_nsec);
  return (DONE);
}
int lbnl_sleep_us (long microseconds)
{
  int error;
  struct timespec sleep_time, rem_time;
  sleep_time.tv_sec = 0;
  sleep_time.tv_nsec = microseconds * 1000;
  error = nanosleep(&sleep_time, &rem_time);
  if (error)
    printf("Sleep: %d\t rem time: %ld %ld\n", error, rem_time.tv_sec, rem_time.tv_nsec);
  return (DONE);
}
/**
 * update the FPGA bitfile
 * uses cat and avoids any effort
 * Checks if the bitfile exists, path is defined in typedefs.h
 * @param[in] fd is the driver reference
 **/
int lbnl_update_fpga (fd)
{
  int error = 0;
  error = lbnl_check_lock(fd);
  if (error != 0) {
    return (error);
  }
  if (access(BITFILE_PATH, F_OK) != 0) {	// Check if file exists
    return FAILED;
  }

  char command[80];
  sprintf(command, "%s %s %s", "cat ", BITFILE_PATH, " > /dev/xdevcfg");
  error = system(command);
  return (error);
}
