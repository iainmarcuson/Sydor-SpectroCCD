#ifndef LBNL_TYPE_H_
#define LBNL_TYPE_H_

#define	DONE	0
#define OK	0
#define ABORT 10
#define UNTOUCH	-1
#define NOT_OPEN -2
#define WRONG_HANDLE -3
#define LOCKED -4
#define NO_POWER -5
#define TIMEOUT -6

#define FAILED -10
#define SPI_SEND_FAILED -11
#define SPI_ACK_FAILED -12
#define SPI_RECEIVE_FAILED -13

#define DAC_CMD_RESET               0x01
#define DAC_CMD_WRITE               0x02
#define DAC_CMD_CLR                 0x03
#define DAC_CMD_UPDATE              0x04
#define DAC_CMD_READ				0x05

#define DAC_INPUT_WRITE_MODE        0xC0		//0b11<<6
#define DAC_OFFSET_WRITE_MODE       0x80 		//0b10<<6
#define DAC_GAIN_WRITE_MODE         0x04		//0b01<<6
#define DAC_SPECIAL_FUNCTIONS_MODE  0x00		//0b00<<6

#define DAC_SF_WRITE_OFS0			0x02
#define DAC_SF_WRITE_OFS1			0x03
#define DAC_SF_READBACK             0x05

#define X1A_REGISTER				0x00

#define PIC_CMD_READ_ADC            0x10
#define PIC_CMD_SET_SEGMENT         0x11
#define PIC_CMD_SET_GAIN            0x12

#define ADC_CMD_WRITE               0x20
#define HV_CMD_WRITE               	0x20
#define HV_CMD_ERASE               	0x21
#define HV_CMD_PURGE               	0x22
#define CHECK_ERASE_PURGE			0x23
#define CHECK_DAC_READBACK			0x24
#define CHECK_VERSION				0x25

#define TEMPERATURE_CMD_RDATA		0x10
#define TEMPERATURE_CMD_RESET		0x07

#define DACMASK	0x7f3f3f3f
#define CLKMASK 0xfffff
//DAC settings
#define OFFSETVALUE_bias	8192
#define MVALUE_bias			0xfffe
#define CVALUE_bias			32768
#define OFFSETVALUE_clk		8192
#define MVALUE_clk			0xfffe
#define CVALUE_clk			32768
#define VREF_BIAS			2.5
#define VREF_CLK			1.2

#define NDACS		17	//TODO 17
#define NCLOCKS		20	// 20
#define NOFFSETS	8
#define NDELAYS         16
#define EPICS_STRLEN 41

static const float clk_buffergain	=4.77;
static const float clk_hvbuffgain	=190.22;
static const float clk_adc_gain 	=	0.0064;
static const int clk_adc_offset  	=	1866;
static const float clk_hv_adc_gain = 	0.0462;
static const int clk_hv_adc_offset = 	0;
static const float biasAdcGains[] = { 0.0103,0.0103,0.0103,0.0103,  0.00526,0.00526,0.00526,0.00526,  0.01484,0.01484,0.01484,0.01484,  0.01484,0.01484,0.01484,0.01484};
static const int biasAdcOffset[] = {1918,1918,1918,1918,  1790,1790,1790,1790,   1959,1959,1959,1959,   1959,1959,1959,1959};
static const float biasBuffGains[16] = { 4.0,4.0,4.0,4.0,  2.0,2.0,2.0,2.0, 6.0,6.0,6.0,6.0,   6.0,6.0,6.0,6.0 };
static const float offsetBuffGains[8] = { 1.0,1.0,1.0,1.0,  1.0,1.0,1.0,1.0 };

#define  default_read_address 32
#define  default_read_up_address 96
#define  default_read_down_address 160
#define  default_clear_address 224
#define  default_clear_up_address 224
#define  default_clear_down_address 288
#define  default_dummy_address 0

// Default erase and purge parameters (provided by Armin)
#define default_Erase_Vclk_voltage  11.0   // Volts
#define default_Erase_Vsub_voltage 0.0  // Volts
#define default_Erase_slew  100            // ms
#define default_Erase_delay 500           // ms
#define default_Purge_Vclk_voltage  -9.0  // Volts
#define default_Purge_delay  50           // ms
#define default_clearcols  100           // serial shifts per line during clear.

#define LOCKFILE_PATH "/tmp/lockfile"
#define BITFILE_PATH "/root/system_top.bit.bin"

enum CFG_PARAM_TYPE {
	TYPE_I32,
	TYPE_F32
};

typedef int i32;
typedef short i16;
typedef unsigned short u16;
typedef unsigned int u32;
typedef float f32;
typedef char i8;
typedef unsigned int ptr32_t;
typedef int dref;
#ifdef OS64
	typedef unsigned long u64;
	typedef unsigned long ptr_t;
	typedef unsigned int dptr_t;
	typedef unsigned int data_t;
#else
	typedef unsigned long long u64;
//size of the pointer to access memory data in host computer. Kept to 64 bits, even if OS is 32 bits
	typedef unsigned long long ptr_t;
//size of total addresses for accesing controller data. Assumed 32 bits
	typedef unsigned long dptr_t;
//width of the controller registers. Assumed 32 bits
	typedef unsigned long data_t;
#endif

typedef
struct dac_struct {
	u16 address; //since the DAC is not memory mapped, this is an index on video board.
	f32 tvalue;
	f32 telemetry;
	u16 raw_value;
	u16 raw_telemetry;
} dac_t;

typedef
struct lbnl_clock_struct {
	u16 address;//since the DAC is not memory mapped, this is an index on clk board.
	f32 tlow_value;
	f32 thigh_value;
	f32 telemetry;
	u16 raw_telemetry;
	u16 low_raw_value;
	u16 high_raw_value;
} lbnl_clock_t;

typedef
struct cds_struct {
	u16 nsamp_signal;
	u16 nsamp_reset;
	u16 averaging;
	u16 digioff;
} cds_t;

typedef
struct delays_struct {
	u16 settling_signal;
	u16 settling_reset;
	u16 clock_serial;
	u16 clock_sumwell;
	u16 clock_reset;
	u16 clock_parallel;
	u16 other1;
	u16 other2;
	u16 other3;
	u16 other4;
} delays_t;

typedef
struct status_struct {
	i8 power_on;
	i8 ccd_idle;
	u32 clk_mask;
	u32 dac_mask;
} status_t;

typedef
struct readout_struct {
	u16 progress;	/*from 0 to 100%*/
	u16 rows_read;	/*numbers of complete rows read*/
} readout_t;

typedef
union config_param {
	int *i;
	float *f;
} cfg_t;

typedef
struct param_struct {
  //char *desc[EPICS_STRLEN];	/* Array of strings holding user description */
  char (*desc)[EPICS_STRLEN];	/* Array of strings holding user description */
  cfg_t val_array;		/* Pointer to array of parameter values */
  cfg_t lo_array;		/* Pointer to array of low limits for parameters */
  cfg_t hi_array;		/* Pointer to array of high limits for parameters */
  int num_elem;			/* Number of values for this parameter */
  char *format;			/* How to print values for string */
  enum CFG_PARAM_TYPE param_type; /* param_struct can hold multiple types, so keep track */
  int val_len;			  /* How big a string needs to be to hold one value */
  char * (*to_string)(const struct param_struct *param); /* Converts to long string for network transmission*/
} param_t;

#endif
