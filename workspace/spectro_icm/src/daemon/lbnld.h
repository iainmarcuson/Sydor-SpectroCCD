#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <fcntl.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <sys/un.h>
#include <pthread.h>
#include <syslog.h>
#include "../lbnl_typedefs.h"

#define BPP	2
#define MAXCMDWORDS	12
#define MAXRESPWORDS	12

//#include "tcplinux.h"
#define LBNL_SOCKET_STR	"/tmp/lbnld.sock"
#define LBNL_MAX_CONNECT	2
#define	LBNL_SHUTDOWN	99
#define	LBNL_OPEN	1
#define	LBNL_READ	2
#define	LBNL_WRITE	3
#define	LBNL_CLOSE	4
#define LBNL_IMAGE	5
#define	LBNL_IMSIZE	6
#define	LBNL_PROGRESS	7
#define	LBNL_CCD_CLEAR	8
#define	LBNL_CCD_ERASE	9
#define	LBNL_CCD_PURGE	10
#define	LBNL_GET_IMSIZE	11
#define	LBNL_IDLE	12
#define	LBNL_POWER	13
#define	LBNL_ENABLE	14
#define	LBNL_GAIN	15
#define	LBNL_TEMPS	16
#define	LBNL_GET_DACS	17
#define	LBNL_GET_NDACS	18
#define	LBNL_GET_CLKS	19
#define	LBNL_GET_NCLKS	20
#define	LBNL_LOAD_TIM	21
#define	LBNL_DEFAUL_TIM	22
#define	LBNL_SET_CDS	23
#define	LBNL_GET_CDS	24
#define	LBNL_SET_DAC	25
#define	LBNL_FITS	26
#define LBNL_GET_PROG	27
#define LBNL_GET_STAT	28
#define LBNL_SET_DELAY	29
#define LBNL_SET_CLK	30
#define LBNL_SET_OFF	31
#define LBNL_SET_EXPT	32
#define LBNL_EXP_ADD    33
#define	LBNL_GET_OFF	34
#define	LBNL_GET_NOFF	35
#define LBNL_GET_DELAYS	36
#define LBNL_GET_REG	37
#define LBNL_SET_REG	38
#define LBNL_ARTIF_DATA	39
#define LBNL_SET_CPARS	40
#define LBNL_SET_EPARS	41
#define LBNL_SET_PPARS	42
#define LBNL_GET_CPARS	43
#define LBNL_GET_EPARS	44
#define LBNL_GET_PPARS	45
#define LBNL_MASTER		46
#define LBNL_INIT		47
#define LBNL_CTRL_ENABLE	48
#define LBNL_GET_LOGLVL	49
#define LBNL_SET_LOGLVL 50
#define LBNL_LOAD_CFG 51
#define LBNL_FLOAD_TIM	52
#define LBNL_FLOAD_CFG	53
#define LBNL_TIM_DESC 60
#define LBNL_TIM_VALS 61
#define LBNL_OFF_DESC 62
#define LBNL_OFF_VALS 63
#define LBNL_DAC_DESC 64
#define LBNL_DAC_VALS 65
#define LBNL_CLK1_DESC 66
#define LBNL_CLK1_VALS 67
#define LBNL_CLK2_VALS 68
#define LBNL_GET_CFG_ARRAY 70
#define LBNL_GET_CFG_STRING_ARRAY 71
#define LBNL_IMG_ACQ 80
#define LBNL_IMG_READ 81
#define LBNL_IMG_ABORT 82
#define LBNL_GET_ACQ_STATUS 90
#define LBNL_GET_DAC_VALS 100
#define LBNL_GET_ENABLE_VALS 110
#define LBNL_GET_CLK_VALS 140
#define LBNL_GET_EXPT 150
#define LBNL_GET_ELAPSED_TIME 151
#define LBNL_GET_EXPTIME_PROG 152
#define LBNL_GET_OFF_VALS 160
#define LBNL_GET_GAIN 170
#define LBNL_GET_IMG_CNT 180
#define LBNL_SET_IMG_HALF 190
#define LBNL_GET_IMG_HALF 191
#define LBNL_GET_CCDSIZE 200
#define LBNL_SET_SHUTTERMODE 210
#define LBNL_GET_SHUTTERMODE 211
#define LBNL_SET_FAST_MODE 220
#define LBNL_SET_BG 230
#define LBNL_GET_BG 231
#define LBNL_RST_TIMER 240
#define LBNL_RD_TIMER 241
#define LBNL_GET_PIX_SIZE 250
#define LBNL_READ_AUX_PORT 260
#define LBNL_SET_TRIG_MODE 270
#define LBNL_GET_TRIG_MODE 271
#define LBNL_READ_GPIO 280
#define LBNL_READ_HW_TRIG 290
#define LBNL_CFG_NAME 300
#define LBNL_TIM_NAME 310


//XXX FIXME TODO
//Below are aggregated commands.  Make separate commands.
#define LBNL_CMD_RESET 128
#define LBNL_CMD_CONFIG 129
#define LBNL_CMD_ENABLE 130

#ifdef _DEBUG_
#  define pdebug(fmt, args...) printf ("lbnldaemon: " fmt, ## args)
#else
#  define pdebug(fmt, args...)
#endif

typedef unsigned int lbnldata_t;
typedef unsigned int lbnladd_t;

typedef struct {
	int cmd;
	int nw;
	char strmsg[128];
	lbnldata_t data[MAXCMDWORDS];
} cmdstruct_t;

typedef struct {
        char strmsg[128];
        lbnldata_t data[MAXRESPWORDS];
        int status;
} respstruct_t;

typedef struct {
        int status;
        dac_t *dacs;
} dacresp_t;

enum ImgAcqStatus {
	Not_Started = 128,
	Done = 129,
	Aborted = 130,
	No_Exposure = 131, //Used at start of system to indicate no exposure has taken place; Not_Started is part of the acq sequence.
	Clearing = 1,
	Exposing = 2,
	Readout = 3,
	Sending = 4
};

