#include "lbnl_typedefs.h"

/* proposed function prototypes. Parameters description on the c code file*/

/*The next functions should probably be implemented by any caller*/
dref lbnl_open (char *params);
int lbnl_close (dref fd);
int lbnl_check_lock (dref fd);
int lbnl_init (dref fd);
int lbnl_ccd_clear_up_or_down (dref fd, int up_or_down);
int lbnl_ccd_erase (dref fd);
int lbnl_ccd_purge (dref fd);
int lbnl_ccd_read_up_or_down (dref fd, u16 image_data[], int up_or_down);
int lbnl_readout_get_data (dref fd, u16 *bufptr);
int lbnl_readout_prepare (dref fd);
int lbnl_readout_discard (dref fd);
int lbnl_ccd_set_erase_params (dref fd, f32 vclk, f32 vsub, i32 slew, i32 delay);
int lbnl_ccd_set_purge_params (dref fd, f32 vclk, i32 delay);
int lbnl_ccd_set_clear_params (dref fd,  i32 clearcols);
int lbnl_ccd_set_size (dref fd, u16 ncols, u16 nrows);
int lbnl_ccd_get_erase_params (dref fd, f32 *vclk, f32 *vsub, i32 *slew, i32 *delay);
int lbnl_ccd_get_purge_params (dref fd, f32 *vclk, i32 *delay);
int lbnl_ccd_get_clear_params (dref fd,  i32 *clearcols);
int lbnl_ccd_get_size (dref fd, u16 *ncols, u16 *nrows);
int lbnl_ccd_idle (dref fd, i8 flag);
int lbnl_readout_get_memfits (dref fd, void *imptr, u16 image_data[]);
int lbnl_readout_get_fits (dref fd, char *impath, u16 image_data[]);
int lbnl_readout_get_status (dref fd, readout_t *readstat);
int lbnl_controller_get_status (dref fd, status_t *contstat);
int lbnl_controller_upload_timing (dref fd, char *timingpath);
int lbnl_controller_set_start_address (dref fd, u16 start_address);
int lbnl_controller_power (dref fd, i8 flag);
int lbnl_controller_master (dref fd, i8 flag);
int lbnl_controller_enable (dref fd, u32 dac_mask, u32 clk_mask);
int lbnl_controller_set_gain (dref fd, u32 gain);
int lbnl_controller_get_temps (dref fd, f32 *temp1, f32 *temp2);
int lbnl_controller_upload_config (dref fd, char *cfgpath);


/*IMPORTANT: the next ones may not be required if the external caller will implement
 * them externally
 */
int lbnl_controller_set_exptime (dref fd, u32 exptime);
int lbnl_controller_get_exptime (dref fd, u32 *exptime);
int lbnl_controller_set_autoshutter (dref fd, i8 autoshutter);
int lbnl_controller_get_autoshutter (dref fd, i8 *autoshutter);
int lbnl_controller_set_shutter (dref fd, i8 shut);
int lbnl_controller_get_shutter (dref fd, i8 *shut);



/*IMPORTANT: the next ones are low level, engineering ones, and they may not be 
 * implemented by all callers -this is, only the engineering-oriented interfaces
 * but not necessarily the run-only clients. Some may require knowledge and safety
 * measures by the caller*/
int lbnl_controller_download_timing (dref fd, u32 *timingarray);
int lbnl_controller_get_ndacs (dref fd, u16 *ndacs);
int lbnl_controller_get_all_dacs (dref fd, dac_t *dacs, u16 *ndacs);
int lbnl_controller_get_noffsets (dref fd, u16 *ndacs);
int lbnl_controller_get_all_offsets (dref fd, dac_t *dacs, u16 *ndacs);
int lbnl_controller_get_nclocks (dref fd, u16 *nclocks);
int lbnl_controller_get_all_clocks(dref fd, lbnl_clock_t *clocks, u16 *nclocks);
int lbnl_controller_get_dac (dref fd, u16 address, dac_t *dac);
int lbnl_controller_set_dac_value (dref fd, u16 address, f32 value);
int lbnl_controller_set_offset_value (dref fd, u16 address, f32 value);
int lbnl_controller_set_clk_value (dref fd, u16 address, f32 high_value, f32 low_value);
int lbnl_controller_set_dac_params (dref fd, dac_t dac);
int lbnl_controller_set_default_dacs (dref fd);
int lbnl_controller_set_default_clks (dref fd);
int lbnl_controller_set_default_offsets (dref fd);
int lbnl_controller_set_default_cds (dref fd);
int lbnl_controller_set_default_delays (dref fd);


/*Note: we could make the same set of "dacs" command with "clocks", in case we want to
 * differentiate them. A clock level -high and low- could be considered a DAC and be
 * treated no differently than a bias DAC.
 * The alternative is to make functions as "set_clock_value" that passes a structure
 * like clock_t that has the low and high clock levels (so setting both dacs)
 */
int lbnl_controller_write_register (dref fd, u16 type, dptr_t address, data_t regval);
int lbnl_controller_read_register (dref fd, u16 type, dptr_t address, data_t *regval);
int lbnl_controller_pass (dref fd, char *command, u32 *response);
int lbnl_controller_get_cds (dref fd, cds_t *cds);
int lbnl_controller_set_cds (dref fd, cds_t cds);
int lbnl_controller_get_delays (dref fd, delays_t *delays);
int lbnl_controller_set_delays (dref fd, delays_t delay);
int lbnl_sleep_ms (long miliseconds);
int lbnl_sleep_us (long microseconds);
int lbnl_update_fpga (dref fd);

/* Testing function prototypes */
int lbnl_test_mode();
