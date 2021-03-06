#include "lbnld.h"
#include "daemon_defs.h"
#include "../lbnl_prototypes.h"
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "../lbnl_typedefs.h"
#include "../lbnl_params.h"
#include "../lbnl_aux.h"

int got_sigterm=0;
int listenfd;
dref dfd;
unsigned int bsize;
u16 *imbuffer, *up_buffer, *down_buffer;
static int image_num=0;
static u32 cfg_gain;

//Image size
static u32 img_size_x, img_size_y;
static u32 ccd_size_x, ccd_size_y;  //Size of the active area, before overscan
static float pix_width, pix_height; /* Size of a pixel in microns */

//Image acquisition and readout thread variables
static sem_t acq_state;
static pthread_t acq_thread, read_thread, fits_thread;
static enum ImgAcqStatus img_acq_state;
static pthread_mutex_t acq_state_mutex;

/* Trigger mode variables */
typedef enum 
  {
   TRIGGER_INTERNAL = 0,
   TRIGGER_EXTERNAL
  } en_Trig_Mode;
en_Trig_Mode trig_mode;
static pthread_mutex_t trig_mode_mutex;

/* HW Trigger Monitor variables */
static pthread_mutex_t trig_mon_mutex; /* Mutex for trigger status */
static unsigned int hw_trig_detected;  /* Variable holding trigger status */

/* Temperature Read Ability variables */
static pthread_mutex_t temp_ability_mutex; /* Mutex for modifying variable declaring ability ot read temperature */
static unsigned int temp_read_ability;	   /* Variable to hold status of reading temperature ability */
static pthread_mutex_t temp_value_mutex;   /* Mutex for holding temperature */
static const unsigned int TEMP_READ_RESET = 0x01; /* Bit 0 */
static const unsigned int TEMP_READ_TIM = 0x02;	  /* Bit 1 */
static const unsigned int TEMP_READ_CFG = 0x04;	  /* Bit 2 */
static const unsigned int TEMP_READ_ENABLE = 0x08; /* Bit 3 */
static const unsigned int TEMP_READ_DONE = 0x07;   /* OR of above */

/* Temperature Thread Variables */
static float temperature_1;	/* First read temperature from sensor */
static float temperature_2;	/* Second read temperature */
static int temperature_error;	/* Error on last temperature read */
static const float TEMP_INVALID = -1234.5; /* Easily-recognizable invalid temperature */

/* Auxillary port value */
static volatile u32 aux_port_val;
static volatile u32 aux_port_cnt;

//Image variables
static int img_count_reset;	/* Count of images since last reset */

void *pt_take_picture(void *arg);
void *pt_read_picture(void *arg);
void *pt_take_fits(void *arg);
void *thread_hw_trig_mon(void *arg);
void set_shutter(int new_state);
void lbnl_register_params(void);
void *pt_read_temperature(void *arg);

//TODO ultimately can be aborted
int wait_exposure_time();	/* Waits for a exposure time */

extern param_t config_params[5];

/* Enable mask variables to share across threads */
static u32 enable_num;
static u32 enable_dacmask;
static u32 enable_clkmask;

/* Exposure time to share across threads */
static u32 exp_time;		/* Holds the total exposure time */
static u32 exp_time_elapsed;	/* How much of an exposure time has elapsed */
static u32 exp_abort;		/* Flag to abort an exposure during the 
				 exposure time */


// Read-out mode parameters
static u32 b_read_fast; // High to read fast

/* Shutter-mode */
static u32 shutter_mode;	/* TODO Maybe add in shutter status var */
/* See also "Take as BG" options */

static volatile u32 take_as_bg = 0;
static pthread_mutex_t bg_state_mutex;


/* Timer variables */
static struct timespec start_time, curr_time;
static int elapsed_time;

/* Filename variables */
static const int MAX_FILENAME = 80; /* Longest filename */
static const int MAX_FITSLEN = 68;  /* Longest FITS string length */
static char cfg_filename[80]; /* Holds filename only */
static char tim_filename[80]; /* Holds filename only */


/* Globals for normal- and fast-mode config settings */
/* May need to add mutex for coherency */
static cds_t daemon_cds_normal, daemon_cds_fast;
static delays_t daemon_ccd_delay_normal, daemon_ccd_delay_fast;
static u32 b_video_mode;		/* Used for very fast acquisition */

void populate_slow_mode(cds_t *curr_cds, delays_t *curr_delays);
void populate_fast_mode(const cds_t *cds_normal, const delays_t *ccd_delay_normal, cds_t *cds_fast, delays_t *ccd_delay_fast);

/* Exposure Parameters */
struct Exposure_Parameters
{
  int acq_mode;			/* Acquisition mode e.g. continuous */
  int ccd_clear;		/* Whether to clear in continuous mode */
};
typedef struct Exposure_Parameters exp_param_t;

int main(int argc, char **argv)
{
  int              connfd, len;
  socklen_t        clilen, addrlen;
  struct sockaddr_in address,remote;
  pthread_t        tid;
  pthread_t temperature_thread_id;

  // Clear count of aux port reads
  aux_port_cnt = 0;

  /* Clear image counter variable at startup */
  img_count_reset = 0; 

  // Set to regular readout mode
  b_read_fast = 0;

  /* Initialize filenames */
  sprintf(cfg_filename, "Not_Set");
  sprintf(tim_filename, "Not_Set");

  //Register the parameters into the config variable
  lbnl_register_params();

  if ((listenfd = socket(AF_INET,SOCK_STREAM,0)) > 0)
    printf ("socket was created\n");
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(15001);
  if (bind(listenfd,(struct sockaddr *) &address, sizeof(address)) == 0)
    printf("Binding Socket\n");

  sem_init(&acq_state, 0, 1);	/* Only one thread at a time can use the
				   camera */
  listen (listenfd,LBNL_MAX_CONNECT);
  signal(SIGINT, signal_handler);
  signal(SIGPIPE, signal_handler);
  signal(SIGHUP,signal_handler); /* catch hangup signal */
  //XXX FIXME uncomment this signal(SIGTERM,signal_handler); /* catch kill signal */
  addrlen = sizeof(struct sockaddr_in);

  pthread_mutex_init(&trig_mode_mutex, NULL);

  pthread_mutex_init(&acq_state_mutex, NULL);
  pthread_mutex_lock(&acq_state_mutex);
  img_acq_state = No_Exposure;
  pthread_mutex_unlock(&acq_state_mutex);
  
  pthread_mutex_init(&bg_state_mutex, NULL);
  pthread_mutex_lock(&bg_state_mutex);
  take_as_bg = 0;
  pthread_mutex_unlock(&bg_state_mutex);

  pthread_mutex_init(&trig_mon_mutex, NULL);
  pthread_mutex_lock(&trig_mon_mutex);
  hw_trig_detected = 0; 	/* No trigger at start */
  pthread_mutex_unlock(&trig_mon_mutex);
  
  pthread_mutex_init(&temp_value_mutex, NULL);
  pthread_mutex_lock(&temp_value_mutex);
  temperature_1 = TEMP_INVALID;
  temperature_2 = TEMP_INVALID;
  temperature_error = 0;	/* No error to start */
  pthread_mutex_unlock(&temp_value_mutex);

  /* Initialize exposure time -- do this before a mutex lock/unlock for hopeful memory barrier */
  exp_time = 0;

  /* Initialize temperature read ability mutex and status variable */
  pthread_mutex_init(&temp_ability_mutex, NULL);
  pthread_mutex_lock(&temp_ability_mutex);
  temp_read_ability = 0;	/* No steps taken at start */
  pthread_mutex_unlock(&temp_ability_mutex);

  /* Spawn trigger monitor */
  pthread_create(&tid, NULL, &thread_hw_trig_mon, NULL);

  /* Spawn temperature read thread */
  pthread_create(&temperature_thread_id, NULL, &pt_read_temperature, NULL);


  /* Test mode */
  if ((argc>=2) && (strcmp(argv[1], "--test") == 0))
    {
      return lbnl_test_mode();
    }
  while (!got_sigterm) {
    clilen = addrlen;
    printf ("accepting connections\n");
    len = sizeof(remote);
    connfd = accept(listenfd, (struct sockaddr *)&remote, (socklen_t *)&len);
    if (connfd > 0){
      printf ("connected %d\n", connfd);
      pthread_create(&tid, NULL, &thread_main, (void *)connfd);
    } else
      printf ("got wrong connection (%d %d)\n", connfd, errno);
  }
  pthread_join(tid,NULL);
  return(0);
}

int allocate_buffer (int nbytes)
{
  if (bsize >= nbytes)
    return (0);
  if (imbuffer != NULL) {
    free (imbuffer);
    free (up_buffer);
    free (down_buffer);
    bsize = 0;
    imbuffer = NULL;
  }
  //	imbuffer = (unsigned short *) malloc (nbytes);
  imbuffer = (i16 *) malloc (nbytes);
  up_buffer = (i16 *) malloc (nbytes);
  down_buffer = (i16 *) malloc (nbytes);
  //	*imbuffer = 121;
  if ((imbuffer == NULL) || (up_buffer == NULL) || (down_buffer == NULL))
    return (-ENOMEM);
  bsize = nbytes;
  return (0);
}

/* Hardware trigger monitor */
void *thread_hw_trig_mon(void *hw_trig_data)
{
  unsigned int curr_port;
  unsigned int prev_port;	/* Current and previous port values */
  const unsigned int TRIG_REG = 8; /* Read in at reg 8 */
  const unsigned int TRIG_MASK = 0x01; /* Bit zero */
  const unsigned int SLEEP_TIME = 50000; /* Sample at ~20Hz */
  curr_port = 0 & TRIG_MASK;
  prev_port = 0 & TRIG_MASK;		/* Start with trigger "detected" to force full cycle */

  while(1)			/* Loop forever */
    {
      /* XXX TODO See if can pass dfd as zero for no impact, since it does not appear to be used in function */
      prev_port = curr_port;
      /* TODO Add in error checking and persistent memory pointer */
      lbnl_controller_read_gpio(0, TRIG_REG, &curr_port);
      curr_port = curr_port & TRIG_MASK; /* Only look at trigger bit */
      if ((curr_port == 0) && (prev_port)) /* XXX Trigger on negative edge */
	{
	  pthread_mutex_lock(&trig_mon_mutex);
	  hw_trig_detected = 1;
	  pthread_mutex_unlock(&trig_mon_mutex);
	} /* XXX No else; trigger is sticky and reset on read */

      usleep(SLEEP_TIME);		/* Sample again after a sleep */
    }
  
  return NULL;			/* XXX Should never get here. */
}

/*-----------------------------------------------------------------------------
  | thread_main -- main function for threads:
  |
  -----------------------------------------------------------------------------*/
void *thread_main(void *arg)
{
  int  cin, fdin;
  cmdstruct_t message;
  respstruct_t response;
  fdin=(int)arg;
  int ret;
  lbnldata_t val, raddr;
  //  dref dfd;
  int bsize=0;
  int nbytes;
  unsigned short aux1, aux2;
  unsigned short artif_data=0;
  data_t regval;
  f32 faux1, faux2;
  dac_t *dacs;
  f32 *dac_vals;
  cds_t cds;
  readout_t readstat;
  delays_t delay;
  status_t constat;
  lbnl_clock_t *clocks;
  f32 *clk_vals;
  unsigned short ndacs;
  i32 iaux1, iaux2;
  char line[256];
  FILE *name_file;
  int dac_idx;
  int clk_idx;
  int off_idx;
  /* Below used for loading enable data from config file */
  FILE *cfg_file;
  char config_line[256];
  char *data_ptr;
  i32 count_cfg_args;
  int sem_status;
  exp_param_t *exp_args;
  int cfg_line_valid;		/* Determine if a cfg file line 
				   has been parsed properly*/
  //  dacresp_t dacresp;
  int create_thread_ret;	/* Check return status of pthread_create()*/

  do {
    if ((cin=recv(fdin,&message,sizeof(cmdstruct_t), 0))!=sizeof(cmdstruct_t)){
      pdebug ("bad message, connection closed\n");
      printf("received socket data wrong size %d, wanted %d", cin, sizeof(cmdstruct_t));
      close (fdin);
      cin = 0;
    } else {
      //printf ("cmd %d\n", message.cmd);
      switch (message.cmd) {
      case LBNL_OPEN:
	ret = 0;
	if (dfd == 0){
	  if ((dfd=lbnl_open (NULL))<0){
	    printf ("error opening %d\n", dfd);
	    sprintf (response.strmsg, "ERROR opening %d\n",dfd);
	    lbnl_close (dfd);
	    ret = dfd;
	  } else {
	    sprintf (response.strmsg, "DONE");
	  }
	} else {
	  sprintf (response.strmsg, "Already opened");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_CLOSE:
	lbnl_close (dfd);
	printf ("cmd: CLOSE\n");
	response.status = 0;
	sprintf (response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	sleep (1);
	close (fdin);
	fdin = 0;
	cin = 0;

	// Azriel try and reset the driver handle so that new opens can happen
	dfd = 0;
	// end of added block

	break;
      case LBNL_IMSIZE:
	printf ("cmd: IMSIZE\ndata: %d %d\n", message.data[0], message.data[1]);
	if ((ret=lbnl_ccd_set_size (dfd, message.data[0], message.data[1]))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  nbytes = message.data[0] * message.data[1] * BPP;
	  if (nbytes > 0){
	    printf ("allocating buffer for %d bytes\n", nbytes);
	    if ((ret=allocate_buffer (nbytes)) <0)
	      sprintf (response.strmsg, "ERROR allocating buffer %d\n", ret);
	    else
	      sprintf (response.strmsg, "DONE");
	  } else {
	    ret = -22;
	    sprintf (response.strmsg, "ERROR trying to allocat 0 bytes buffer\n");
	  }
	}
	printf ("buffer %s\n", response.strmsg);
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_ARTIF_DATA:
	printf ("cmd: ARTIF_DATA %d\n", message.data[0]);
	artif_data = message.data[0];
	response.status = 0;
	sprintf (response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_IMSIZE:
	printf ("cmd: GET_IMSIZE\n");
	if ((ret=lbnl_ccd_get_size (dfd, &aux1, &aux2))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	response.data[0] = (u32) aux1;
	response.data[1] = (u32) aux2;
	img_size_x = (u32) aux1;
	img_size_y = (u32) aux2;
	printf("Retrieved image size %i, %i.\n", aux1, aux2);
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_REG:
	printf ("cmd: GET_REG %d %d\n", message.data[0], message.data[1]);
	if ((ret=lbnl_controller_read_register (dfd, message.data[0], message.data[1], &regval))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	response.data[0] = (u32) regval;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_SET_CDS:
	printf ("cmd: CDS\ndata:\n");
	cds.nsamp_signal = message.data[0];
	cds.nsamp_reset = message.data[1];
	cds.averaging = message.data[2];
	cds.digioff = message.data[3];
	printf ("nsamp_signal %d, nsamp_reset %d, averaging %d, digioff %d\n", cds.nsamp_signal, cds.nsamp_reset, cds.averaging, cds.digioff);
	if ((ret=lbnl_controller_set_cds (dfd, cds))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  printf ("CDS OK\n");
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_SET_DELAY:
	printf ("cmd: DELAYS\ndata:\n");
	delay.settling_signal = message.data[0];
	delay.settling_reset = message.data[1];
	delay.clock_serial = message.data[2];
	delay.clock_sumwell = message.data[3];
	delay.clock_reset = message.data[4];
	delay.clock_parallel = message.data[5];
	delay.other1 = message.data[6];
	delay.other2 = message.data[7];
	delay.other3 = message.data[8];
	delay.other4 = message.data[9];
	if ((ret=lbnl_controller_set_delays (dfd, delay))<0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  printf ("DELAYS OK\n");
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_CDS:
	printf ("cmd: GET_CDS\n");
	if ((ret=lbnl_controller_get_cds (dfd, &cds))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	response.data[0] = cds.nsamp_signal;
	response.data[1] = cds.nsamp_reset;
	response.data[2] = cds.averaging;
	response.data[3] = cds.digioff;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_DELAYS:
	printf ("cmd: GET_DELAYS\n");
	if ((ret=lbnl_controller_get_delays (dfd, &delay))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	response.data[0] = delay.settling_signal;
	response.data[1] = delay.settling_reset;
	response.data[2] = delay.clock_serial;
	response.data[3] = delay.clock_sumwell;
	response.data[4] = delay.clock_reset;
	response.data[5] = delay.clock_parallel;
	response.data[6] = delay.other1;
	response.data[7] = delay.other2;
	response.data[8] = delay.other3;
	response.data[9] = delay.other4;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_ENABLE_VALS:
	printf("cmd: GET_ENABLE_VALS\n");
	if (1)
	  {
	    u32 enable_return[3];
	    enable_return[0] = enable_num;
	    enable_return[1] = enable_dacmask;
	    enable_return[2] = enable_clkmask;
	    send (fdin, (void *)enable_return, sizeof(enable_return),0);
	  }
	break;
      case LBNL_GET_PROG:
	//printf ("cmd: GET_PROG\n");
	if ((ret=lbnl_readout_get_status (dfd, &readstat))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	response.data[0] = readstat.progress;
	response.data[1] = readstat.rows_read;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_STAT:
	printf ("cmd: GET_STAT\n");
	if ((ret=lbnl_controller_get_status (dfd, &constat))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	iaux1 = (signed int)((signed char) constat.power_on);
	*((i32 *)&response.data[0]) = iaux1;	//Result is signed
	response.data[1] = constat.ccd_idle;
	response.data[2] = constat.clk_mask;
	response.data[3] = constat.dac_mask;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_DEFAUL_TIM:
	printf ("cmd: default_tim\ndata: %s\n",message.strmsg);
	if (strlen(message.strmsg) > 0){
	  //				sprintf (deftimpath, "%s", message.strmsg);
	  sprintf (response.strmsg, "DONE");
	  ret = 0;
	} else {
	  ret = -22;
	  sprintf (response.strmsg, "ERROR %d", ret);
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_LOAD_TIM:
	printf ("cmd: Load_tim\ndata: %s\n",message.strmsg);
	if (strlen(message.strmsg) <= 0)
	  sprintf (message.strmsg, "%s", DEFTIMPATH);
	//			if (strcmp(message.strmsg,"default") == 0)
	//				sprintf (message.strmsg, "%s", deftimpath);
	if ((ret=lbnl_controller_upload_timing (dfd, message.strmsg))!=0){
	  printf ("ERROR uploading %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  printf ("Uploaded OK\n");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_LOAD_CFG:
	printf ("cmd: Load_cfg\ndata: %s\n",message.strmsg);
	if (strlen(message.strmsg) <= 0)
	  sprintf (message.strmsg, "%s", DEFCFGPATH);
	//			if (strcmp(message.strmsg,"default") == 0)
	//				sprintf (message.strmsg, "%s", deftimpath);
	if ((ret=lbnl_controller_upload_config (dfd, message.strmsg))!=0){
	  printf ("ERROR uploading %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  printf ("Uploaded OK\n");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_FLOAD_TIM:
	printf ("cmd: File_Load_tim\n");

	//Open up the file containing the filename
	name_file = fopen(FTIMPATH, "r");
	if (name_file == NULL)	//Handle errors
	  {
	    printf("ERROR opening name file: %s\n", FTIMPATH);
	    response.status = FAILED;
	    ret = response.status;
	    sprintf(response.strmsg, "ERROR %d\n", ret);
	    send (fdin, (void *)&response, sizeof (respstruct_t),0);
	    break;
	  }

	//Read in the first line, where the filename actually is
	if (fgets(line, sizeof(line), name_file) == NULL)
	  {
	    line[0] = '\0';	//Variable is reused, so may contain other data
	    //Thus, it must be nulled out if nothing is read
	  }
	//Strip the ending newline, if there is one
	if (strchr(line,'\n'))
	  {
	    *strchr(line,'\n')='\0';
	  }
	printf("Timing file: %s\n", line);
	//Close the file, since we are done with it
	fclose(name_file);

	/* Now extract the filename proper for reporting in metadata */
	if (1)
	  {
	    char *last_pathsep = NULL;
	    size_t filename_len = 0;
	    last_pathsep = rindex(line, '/'); /* Get last path separator */
	    if (last_pathsep == NULL)	      /* No path, use full name */
	      {
		last_pathsep = line; /* Use the full string */
	      }
	    else		/* There is a path */
	      {
		last_pathsep++;	/* Start after the path separator */
	      }
	    
	    filename_len = strlen(last_pathsep); /* Get length */
	    /* TODO May be useful to report overflow, but not now */

	    bzero(tim_filename, sizeof(tim_filename)); /* Zero out if copy long */
	    strncpy(tim_filename, last_pathsep, MAX_FITSLEN); /* Only copy what will fit */

	  } /* if(1) */
	    
	//Now handle the filename as before
	if (strlen(line) <= 0)
	  sprintf (line, "%s", DEFTIMPATH);
	//			if (strcmp(message.strmsg,"default") == 0)
	//				sprintf (message.strmsg, "%s", deftimpath);
	if ((ret=lbnl_controller_upload_timing (dfd, line))!=0){
	  printf ("ERROR uploading %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  printf ("Uploaded OK\n");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	/* Set ability to read temperature flag; may want this as part of error checking */
	pthread_mutex_lock(&temp_ability_mutex);
	temp_read_ability = temp_read_ability | TEMP_READ_TIM; /* Set flag we just did */
	pthread_mutex_unlock(&temp_ability_mutex);

	break;

      case LBNL_FLOAD_CFG:
	printf ("cmd: File_Load_cfg\n");

	//Open up the file containing the filename
	name_file = fopen(FCFGPATH, "r");
	if (name_file == NULL)	//Handle errors
	  {
	    printf("ERROR opening name file: %s\n", FCFGPATH);
	    response.status = FAILED;
	    ret = response.status;
	    sprintf(response.strmsg, "ERROR %d\n", ret);
	    send (fdin, (void *)&response, sizeof (respstruct_t),0);
	    break;
	  }


	//Read in the first line, where the filename actually is
	if (fgets(line, sizeof(line), name_file) == NULL)
	  {
	    line[0] = '\0';	//Variable is reused, so may contain other data
	    //Thus, it must be nulled out if nothing is read
	  }
	//Strip the ending newline, if there is one
	if (strchr(line,'\n'))
	  {
	    *strchr(line,'\n')='\0';
	  }
	printf("Config file: %s\n", line);
	//Close the file, since we are done with it
	fclose(name_file);
	
	/* Now extract the filename proper for reporting in metadata */
	if (1)
	  {
	    char *last_pathsep = NULL;
	    size_t filename_len = 0;
	    last_pathsep = rindex(line, '/'); /* Get last path separator */
	    if (last_pathsep == NULL)	      /* No path, use full name */
	      {
		last_pathsep = line; /* Use the full string */
	      }
	    else		/* There is a path */
	      {
		last_pathsep++;	/* Start after the path separator */
	      }
	    
	    filename_len = strlen(last_pathsep); /* Get length */
	    /* TODO May be useful to report overflow, but not now */
	    
	    bzero(cfg_filename, sizeof(cfg_filename)); /* Zero out if copy long */
	    strncpy(cfg_filename, last_pathsep, MAX_FITSLEN); /* Only copy what will fit */

	  } /* if(1) */

	//Now handle the filename as before
	if (strlen(line) <= 0)
	  sprintf (line, "%s", DEFCFGPATH);
	//			if (strcmp(message.strmsg,"default") == 0)
	//				sprintf (message.strmsg, "%s", deftimpath);
	if ((ret=lbnl_controller_upload_config (dfd, line))!=0){
	  printf ("ERROR uploading %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  printf ("Uploaded OK\n");
	}

	/* Now parse the file for the enable masks */
	/* First, set to use default masks if no enable entry */
	enable_num = 2;
	cfg_file = fopen(line,"r");
	/* TODO FIXME Add failure checking */
	/* TODO FIXME Handle too-long string */
	while (fgets(config_line, sizeof(config_line), cfg_file))
	  {
	    data_ptr = strstr(config_line, "|");
	    if (!data_ptr)	/* Special functions occur on non-data lines */
	      {
		/* XXX Not thread-safe */
		unsigned int temp_img_size_x;
		unsigned int temp_img_size_y;
		/* TODO Add in handling of error codes in return */
		cfg_line_valid = 0;
		
		count_cfg_args = sscanf(config_line,"Enable: %u %x %x ",
					&enable_num, &enable_dacmask, &enable_clkmask);
		if (count_cfg_args == 3)
		  {
		    /* Correctly parsed Enable line */
		    cfg_line_valid = 1;
		    if (enable_num > 1)
		      {
			/* Set to default */
			enable_dacmask = DACMASK;
			enable_clkmask = CLKMASK;
		      }
		    else if (enable_num == 0)
		      {
			/* Disable masks */
			enable_dacmask = 0;
			enable_clkmask = 0;
		      }
		    else
		      {
			/* Use values unchanged */
		      }

		    /* TODO Add in error support */
		    /* Write the values to the hardware */
		    lbnl_controller_enable(dfd, enable_dacmask, enable_clkmask);
		  }

		//Don't actually read exposure time
		//count_cfg_args = sscanf(config_line, "ExpTime: %*u ", &exp_time);
		//if (count_cfg_args == 1)
		//  {
		//    /* A correctly parsed Exposure Time line*/
		//    cfg_line_valid = 1;
		//  }

		count_cfg_args = sscanf(config_line, "ImgSize: %u %u ", &temp_img_size_x, &temp_img_size_y);
		if (count_cfg_args == 2)
		  {
		    /* Correctly parsed image size */
		    /* TODO Add error handling for set, allocate, and handling zero bytes */
		    cfg_line_valid = 1;
		    int nbytes;
		    img_size_x = temp_img_size_x;
		    img_size_y = temp_img_size_y;
		    lbnl_ccd_set_size(dfd, temp_img_size_x, temp_img_size_y);
		    nbytes = img_size_x * img_size_y * BPP;
		    if (nbytes > 0)
		      {
			allocate_buffer (nbytes);
		      }
		  }

		count_cfg_args = sscanf(config_line, "CCDSize: %u %u ",
					&ccd_size_x, &ccd_size_y);
		if (count_cfg_args == 2)
		  {
		    cfg_line_valid=1;
		  }

		count_cfg_args = sscanf(config_line, "PixSize: %f %f ",
					&pix_width, &pix_height);

		if (count_cfg_args == 2)
		  {
		    cfg_line_valid=1;
		  }
		
		if (!cfg_line_valid)
		  {
		    /* TODO add error handling */
		    printf("Unrecognized config line:\n %s\n", config_line);
		  }
	      }
	  }
	fclose(cfg_file);
	/* Prepare fast and slow structs for later fast mode */
	printf("Populating slow and fast modes.\n");
	populate_slow_mode(&daemon_cds_normal, &daemon_ccd_delay_normal);
	populate_fast_mode(&daemon_cds_normal, &daemon_ccd_delay_normal,
			   &daemon_cds_fast, &daemon_ccd_delay_fast);

	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	/* Set ability to read temperature flag; may want this as part of error checking */
	pthread_mutex_lock(&temp_ability_mutex);
	temp_read_ability = temp_read_ability | TEMP_READ_CFG; /* Set flag we just did */
	pthread_mutex_unlock(&temp_ability_mutex);

	break;

      case LBNL_SET_EXPT:
	printf ("cmd: SET_EXPT\n");
	exp_time = message.data[0];
	response.status = 0;
	sprintf(response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_EXPT:
	printf ("cmd: GET_EXPT\n");
	response.data[0] = exp_time;
	sprintf(response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_GET_ELAPSED_TIME:
	//printf ("cmd: GET_ELAPSED_TIME\n");
	response.data[0] = exp_time_elapsed;
	sprintf(response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_GET_EXPTIME_PROG:
	//printf ("cmd: GET_EXPTIME_PROG\n");
	if (exp_time == 0)
	  {
	    response.data[0] = 100;	/* If no exposure time, say it is 
					 complete. */
	  }
	else
	  {
	    response.data[0] = 100*
	      (((double) exp_time_elapsed)/((double) exp_time)); /* Return percent complete */
	  }
	sprintf(response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
	
	
      case LBNL_TEMPS:
	printf ("cmd: TEMPS\n");
	/* This just reads the variables and passes them on */
	pthread_mutex_lock(&temp_value_mutex);
	faux1 = temperature_1;	/* Copy temperatures */
	faux2 = temperature_2;
	ret = temperature_error; /* Copy status */
	pthread_mutex_unlock(&temp_value_mutex);
	
	/* Populate error message in response */
	if (ret)		/* Error in read */
	  {
	    sprintf(response.strmsg, "ERROR %d\n", ret);
	  }
	else			/* No error in read */
	  {
	    sprintf(response.strmsg, "DONE");
	  }
	/* Set response status */
	response.status = ret;
	
	/* Populate temperatures */
	*((float *) &response.data[0]) =  faux1;
	*((float *) &response.data[1]) =  faux2;
		
	/* Send the response */
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_IDLE:
	printf ("cmd: IDLE %d\n", message.data[0]);
#if 1
	if ((ret=lbnl_ccd_idle (dfd, (i8) message.data[0]))<0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
#endif
	ret = 0;
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_POWER:
	printf ("cmd: POWER %d\n", message.data[0]);
	printf ("dfd %d\n", dfd);
	if ((ret=lbnl_controller_power (dfd, message.data[0])) != 0){
	  printf ("error setting power state");
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	  // lbnl_close (dfd);
	} else {
	  if ( message.data[0] > 0 ){
	    printf ("about to call init\n");
	    if ((ret=lbnl_init (dfd))!=0){
	      sprintf (response.strmsg, "ERROR initializing %d\n",ret);
	    } else {
	      printf ("cmd: POWER/INIT OK (ref %d)\n", dfd);
	      sprintf (response.strmsg, "DONE");
	    }
	  }
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_ENABLE:
	printf ("cmd: ENABLE 0x%x 0x%x\n", message.data[0], message.data[1]);
	if ((ret=lbnl_controller_enable (dfd, message.data[0], message.data[1]))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GAIN:
	printf ("cmd: GAIN %d\n", message.data[0]);
	if ((ret=lbnl_controller_set_gain (dfd, message.data[0]))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	  cfg_gain = message.data[0];
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_GAIN:
	printf ("cmd: GET GAIN %d\n", cfg_gain);
	/* FIXME TODO Add in status setting code. */
	response.data[0] = cfg_gain;
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_EXP_ADD:
	printf ("cmd: EXP_ADD %d\n", message.data[0]);
	if ((ret=lbnl_controller_set_start_address (dfd, message.data[0]))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_CCD_CLEAR:
	printf ("cmd: Clear\n");
	if ((ret=lbnl_ccd_clear_up_or_down (dfd, 1))!=0){
		printf ("ERROR %d\n",ret);
		sprintf (response.strmsg, "ERROR %d\n",ret);
	} else if ((ret=lbnl_ccd_clear_up_or_down (dfd, 2))!=0){
		printf ("ERROR %d\n",ret);
		sprintf (response.strmsg, "ERROR %d\n",ret);
	} else
		sprintf (response.strmsg, "DONE");
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_CCD_ERASE:
	printf ("cmd: Erase\n");
	if ((ret=lbnl_ccd_erase (dfd))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else
	  sprintf (response.strmsg, "DONE");
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_CCD_PURGE:
	printf ("cmd: Purge\n");
	if ((ret=lbnl_ccd_purge (dfd))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR clearing %d\n",ret);
	} else
	  sprintf (response.strmsg, "DONE");
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_IMAGE:
	nbytes = message.data[0];
	printf ("cmd: Image (%d bytes)\n", nbytes);
	response.status = 0;
	printf ("taking image\n");
	usleep (1000);
	if (artif_data==0){
	  if ((ret=lbnl_ccd_read_up_or_down (dfd, imbuffer, 3))!=0){
	    printf ("ERROR acquiring %d\n",ret);
	    sprintf (response.strmsg, "ERROR acquiring %d\n",ret);
	  } else {
	    printf ("image acquired, sending\n");
	    sprintf (response.strmsg, "DONE, image acquiered, sending\n");
	  }
	} else {
	  ret=lbnl_ccd_read_sim (dfd, imbuffer);
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	if (response.status == 0){
	  sleep (1);
	  send (fdin, (char *)imbuffer, nbytes,0);
	}
	break;
      case LBNL_IMG_ABORT:
	response.status = 0;
	printf("cmd: Abort\n");
	exp_abort = 1;		/* Set the abort flag */
	sprintf (response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_FITS:
	response.status = 0;
	printf ("taking image (buf 0x%lx) imbuffer[0] = %d\n", imbuffer, imbuffer[0]);
	usleep (1000);
	//FIXME TODO Handle new img_acq_state values
	if (img_acq_state)
	  {
	    sprintf(response.strmsg, "ACQBUSY");
	    send(fdin, (void *)&response, sizeof(respstruct_t),0);
	    break;
	  }
	img_acq_state = 1;
	pthread_create(&fits_thread, NULL, &pt_take_fits, &fdin);
	sprintf(response.strmsg, "DONE");
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	break;

	if ((ret=lbnl_ccd_read (dfd, imbuffer))!=0){
	  printf ("ERROR acquiring %d\n",ret);
	  sprintf (response.strmsg, "ERROR acquiring %d\n",ret);
	} else {
	  char file_name_full[200];
	  char file_count_ext[200];
	  time_t curr_time_t;
	  struct tm *curr_tm;

	  printf ("image acquired, writing\n");

	  //FIXME TODO Robustify
	  //FIXME XXX TODO Note that this is not thread-safe
	  //Get the current time to timestamp the file
	  curr_time_t = time(NULL);
	  curr_tm = gmtime(&curr_time_t);
	  strftime(file_name_full, sizeof(file_name_full),"/data/image_%Y%m%d_%H%M%S_", curr_tm);

	  //Create the string with the count and the extension
	  sprintf(file_count_ext, "%0.10u.fits", image_num);
	  strcat(file_name_full, file_count_ext);
	  image_num++;	//Increment for next image

	  if ((ret=lbnl_readout_get_fits (dfd, file_name_full, imbuffer)!=0)){
	    //printf ("ERROR writing %d\n",ret);
	  } else {
	    //printf ("DONE\n");
	  }
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_DACS:
	printf ("cmd: GET_DACS\n");
	if ((ret=lbnl_controller_get_ndacs (dfd, &ndacs))<0){
	  printf ("ERROR %d\n",ret);
	} else {
	  dacs = (dac_t *) malloc (ndacs *sizeof (dac_t));
	  if ((ret=lbnl_controller_get_all_dacs (dfd, dacs, &ndacs))!=0){
	    printf ("ERROR %d\n",ret);
	  }
	  //				pdebug ("daemon: dac[5] %d %f\n", dacs[5].address, dacs[5].tvalue);
	}
	response.status = ret;
	sprintf (response.strmsg, "OK");
	//			ret = send (fdin, (void *)&response, sizeof(respstruct_t),0);
	//			printf ("response %d (sent %d)\n", response.status, ret);
	ret = 0;
	if (ret >= 0)
	  //				printf ("sending dacs\n");
	  //				sleep (1);
	  if ((ret = send (fdin, dacs, ndacs*sizeof (dac_t),0)) == -1)
	    printf ("error sending data %d\n", ret);
	free(dacs);		/* Avoid memory leak */
	break;
      case LBNL_GET_DAC_VALS:
	printf ("cmd: GET_DAC_VALS\n");
	if ((ret=lbnl_controller_get_ndacs (dfd, &ndacs))<0){
	  printf ("ERROR %d\n",ret);
	} else {
	  dacs = (dac_t *) malloc (ndacs *sizeof (dac_t));
	  dac_vals = (f32 *) malloc(ndacs * sizeof(f32));
	  
	  if ((ret=lbnl_controller_get_all_dacs (dfd, dacs, &ndacs))!=0){
	    printf ("ERROR %d\n",ret);
	  }
	  //				pdebug ("daemon: dac[5] %d %f\n", dacs[5].address, dacs[5].tvalue);
	}
	response.status = ret;

	/* Copy the actual values to a float array */
	for (dac_idx = 0; dac_idx<ndacs; dac_idx++)
	  {
	    dac_vals[dac_idx] = dacs[dac_idx].telemetry;
	  }

	sprintf (response.strmsg, "OK");
	//			ret = send (fdin, (void *)&response, sizeof(respstruct_t),0);
	//			printf ("response %d (sent %d)\n", response.status, ret);
	ret = 0;
	if (ret >= 0)
	  //				printf ("sending dacs\n");
	  //				sleep (1);
	  //if ((ret = send (fdin, dacs, ndacs*sizeof (dac_t),0)) == -1)
	  if ((ret = send (fdin, dac_vals, ndacs*sizeof (f32),0)) == -1)
	    printf ("error sending data %d\n", ret);
	free(dacs);		/* Avoid memory leak */
	free(dac_vals);
	break;
      case LBNL_GET_NDACS:
	//          printf ("cmd: GET_NDACS\n");
	if ((ret=lbnl_controller_get_ndacs (dfd, &ndacs))<0){
	  printf ("ERROR %d\n",ret);
	}
	response.status = ret;
	response.data[0] = ndacs;
	printf ("resp: %d\n", response.data[0]);
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_SET_DAC:
	faux1 = *((float *) &message.data[1]);
	printf ("cmd: SET_DAC %d %f\n", message.data[0], faux1);
	if ((ret=lbnl_controller_set_dac_value (dfd, (u16) message.data[0], faux1))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_MASTER:
	printf ("cmd: MASTER %hhi\n", (i8) message.data[0]);
	if ((ret=lbnl_controller_master (dfd, (i8) message.data[0]))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;

      case LBNL_INIT:
	printf ("cmd: INIT\n");
	if ((ret=lbnl_init (dfd))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_CTRL_ENABLE:
	printf ("cmd: CTRL ENABLE %i 0x%X 0x%X\n", message.data[0],
		message.data[1], message.data[2]);
	if (message.data[0] > 1)
	  {
	    iaux1 = DACMASK;
	    iaux2 = CLKMASK;
	  }
	else if (message.data[0] == 0)
	  {
	    iaux1 = 0;
	    iaux2 = 0;
	  }
	else
	  {
	    iaux1 = message.data[1];
	    iaux2 = message.data[2];
	  }

	/* Harmonize CTRL_ENABLE and FLOAD_CFG file variables */
	enable_dacmask = iaux1;
	enable_clkmask = iaux2;
	enable_num = message.data[0];
	
	if ((ret=lbnl_controller_enable (dfd, iaux1, iaux2))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_OFF:
	printf ("cmd: GET_OFF\n");
	if ((ret=lbnl_controller_get_noffsets (dfd, &ndacs))<0){
	  printf ("ERROR %d\n",ret);
	} else {
	  dacs = (dac_t *) malloc (ndacs *sizeof (dac_t));
	  if ((ret=lbnl_controller_get_all_offsets (dfd, dacs, &ndacs))!=0){
	    printf ("ERROR %d\n",ret);
	  }
	  pdebug ("daemon: dac[0] %d %f\n", dacs[0].address, dacs[0].tvalue);
	}
	response.status = ret;
	sprintf (response.strmsg, "OK");
	//			ret = send (fdin, (void *)&response, sizeof(respstruct_t),0);
	//			printf ("response %d (sent %d)\n", response.status, ret);
	ret = 0;
	if (ret >= 0)
	  //				printf ("sending dacs\n");
	  //				sleep (1);
	  if ((ret = send (fdin, dacs, ndacs*sizeof (dac_t),0)) == -1)
	    printf ("error sending data %d\n", ret);
	break;

	case LBNL_GET_OFF_VALS:
	printf ("cmd: GET_OFF_VALS\n");
	if ((ret=lbnl_controller_get_noffsets (dfd, &ndacs))<0){
	  printf ("ERROR %d\n",ret);
	} else {
	  dac_vals = (f32 *) malloc (ndacs *sizeof(f32));
	  dacs = (dac_t *) malloc (ndacs *sizeof (dac_t));
	  if ((ret=lbnl_controller_get_all_offsets (dfd, dacs, &ndacs))!=0){
	    printf ("ERROR %d\n",ret);
	  }
	  pdebug ("daemon: dac[0] %d %f\n", dacs[0].address, dacs[0].tvalue);
	}
	response.status = ret;
	sprintf (response.strmsg, "OK");

	for (off_idx = 0; off_idx < ndacs; off_idx++)
	  {
	    dac_vals[off_idx] = dacs[off_idx].telemetry;
	    //dac_vals[off_idx] = dacs[off_idx].tvalue;
	  }

	 if ((ret = send (fdin, dac_vals, ndacs*sizeof (f32),0)) == -1)
	    printf ("error sending data %d\n", ret);
	free(dacs);		/* Avoid memory leak */
	free(dac_vals);
	break;
      case LBNL_GET_NOFF:
	//          printf ("cmd: GET_NOFF\n");
	if ((ret=lbnl_controller_get_noffsets (dfd, &ndacs))<0){
	  printf ("ERROR %d\n",ret);
	}
	response.status = ret;
	response.data[0] = ndacs;
	printf ("resp: %d\n", response.data[0]);
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_SET_OFF:
	faux1 = *((float *) &message.data[1]);
	printf ("cmd: set_offset %d %f\n", message.data[0], faux1);
	if ((ret=lbnl_controller_set_offset_value (dfd, (u16) message.data[0], faux1))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_SET_CLK:
	faux1 = *((float *) &message.data[1]);
	faux2 = *((float *) &message.data[2]);
	printf ("cmd: SET_CLK %d %f %f\n", message.data[0], faux1, faux2);
	if ((ret=lbnl_controller_set_clk_value (dfd, (u16) message.data[0], faux1, faux2))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_CLKS:
	printf ("cmd: GET_CLOCKS\n");
	if ((ret=lbnl_controller_get_nclocks (dfd, &ndacs))<0){
	  printf ("ERROR %d\n",ret);
	} else {
	  clocks = (lbnl_clock_t *) malloc (ndacs *sizeof (lbnl_clock_t));
	  if ((ret=lbnl_controller_get_all_clocks (dfd, clocks, &ndacs))!=0){
	    printf ("ERROR %d\n",ret);
	  }
	  //				pdebug ("daemon: dac[5] %d %f\n", dacs[5].address, dacs[5].tvalue);
	}
	response.status = ret;
	sprintf (response.strmsg, "OK");
	//			ret = send (fdin, (void *)&response, sizeof(respstruct_t),0);
	//			printf ("response %d (sent %d)\n", response.status, ret);
	ret = 0;
	if (ret >= 0)
	  //			printf ("sending clocks\n");
	  //				sleep (1);
	  if ((ret = send (fdin, clocks, ndacs*sizeof (lbnl_clock_t),0)) == -1)
	    printf ("error sending data %d\n", ret);
	free(clocks);
	break;
      case LBNL_GET_NCLKS:
	//                      printf ("cmd: GET_NCLOCKS\n");
	if ((ret=lbnl_controller_get_nclocks (dfd, &ndacs))<0){
	  printf ("ERROR %d\n",ret);
	}
	response.status = ret;
	response.data[0] = ndacs;
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_GET_CLK_VALS:
	printf("cmd: GET_CLK_VALS\n");
	if ((ret=lbnl_controller_get_nclocks (dfd, &ndacs))<0)
	  {
	    printf ("ERROR %d\n",ret);
	  }

	clocks = (lbnl_clock_t *) malloc (ndacs *sizeof (lbnl_clock_t));
	clk_vals = (f32 *) malloc(ndacs *sizeof(f32));
	
	if ((ret=lbnl_controller_get_all_clocks (dfd, clocks, &ndacs))!=0)
	  {
	    printf ("ERROR %d\n",ret);
	  }
	for (clk_idx = 0; clk_idx<ndacs; clk_idx++)
	  {
	    clk_vals[clk_idx] = clocks[clk_idx].telemetry;
	  }
	if ((ret=send (fdin, (void *) clk_vals, ndacs*sizeof(f32),0)) == -1)
	  {
	    printf("error sending data %d\n", ret);
	  }
	free(clocks);
	free(clk_vals);
	break;
      case LBNL_SET_CPARS:
	printf ("cmd: SET_CPARS %d\n", message.data[0]);
	response.status = 0;
	if ((ret=lbnl_ccd_set_clear_params (dfd, message.data[0]))<0){
	  printf ("ERROR %d\n",ret);
	  response.status = ret;
	}
	sprintf (response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_CPARS:
	printf ("cmd: GET_CPARS\n");
	if ((ret=lbnl_ccd_get_clear_params (dfd, &iaux1))<0){
	  printf ("ERROR %d\n",ret);
	}
	response.status = ret;
	response.data[0] = iaux1;
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_SET_EPARS:
	faux1 = *((float *) &message.data[0]);
	faux2 = *((float *) &message.data[1]);
	printf ("cmd: SET_EPARS %f %f %d %d\n", faux1, faux2, message.data[2],message.data[3]);
	response.status = 0;
	if ((ret=lbnl_ccd_set_erase_params (dfd, faux1, faux2, message.data[2], message.data[3]))<0){
	  printf ("ERROR %d\n",ret);
	  response.status = ret;
	}

	sprintf (response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_EPARS:
	printf ("cmd: GET_CPARS\n");
	if ((ret=lbnl_ccd_get_erase_params (dfd, &faux1, &faux2, &iaux1, &iaux2))<0){
	  printf ("ERROR %d\n",ret);
	}
	response.status = ret;
	*((float *) &response.data[0]) =  faux1;
	*((float *) &response.data[1]) =  faux2;
	response.data[2] = iaux1;
	response.data[3] = iaux2;
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_SET_PPARS:
	faux1 = *((float *) &message.data[0]);
	printf ("cmd: SET_PPARS %f %d\n", faux1, message.data[1]);
	response.status = 0;
	if ((ret=lbnl_ccd_set_purge_params (dfd, faux1, message.data[1]))<0){
	  printf ("ERROR %d\n",ret);
	  response.status = ret;
	}

	sprintf (response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
	break;
      case LBNL_GET_PPARS:
	printf ("cmd: GET_PPARS\n");
	if ((ret=lbnl_ccd_get_purge_params (dfd, &faux1, &iaux1))<0){
	  printf ("ERROR %d\n",ret);
	}
	response.status = ret;
	*((float *) &response.data[0]) =  faux1;
	response.data[1] = iaux1;
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_GET_CFG_ARRAY:
	printf ("cmd: GET_CFG_ARRAY %i\n", message.data[0]);
	if (1)
	  {
	    char *data_string;
	    char *numeric_start;
	    f32 *numeric_vals;
	    char *crlf = "\r\n";	//possibly needed for EPICS parsing
	    int data_length;
	    int i,j,k;
	    int nelm;
	    f32 *print_data;
	    char *full_response;

	    data_string = lbnl_param_to_packed(&config_params[message.data[0]],&data_length);
			      
	    memset(response.strmsg,0,sizeof(response.strmsg));
	    strcpy(response.strmsg,"DONE");
	    response.status = 0;
	    response.data[0] = data_length;
	    response.data[1] = config_params[message.data[0]].num_elem; 
			      
	    //DEBUGGING REMOVETHIS
	    printf("GET_CFG_ARRAY: About to send basic response.\n");
			      
	    //XXX Send everything in one packet
	    //send(fdin, (void *)&response, sizeof(respstruct_t),0);
			      
	    //DEBUGGING REMOVETHIS
	    printf("GET_CFG_ARRAY: Sent basic response.\n");
			      
	    //DEBUGGING DIAGNOSTIC
	    nelm = config_params[message.data[0]].num_elem;
	    numeric_vals = calloc(sizeof(f32), nelm*3);
	    numeric_start = data_string+data_length-3*4*nelm;
	    memcpy(numeric_vals, numeric_start, sizeof(f32)*nelm*3);
			      
	    for (i=0; i<(nelm*3); i++)
	      {
		if (config_params[message.data[0]].param_type == TYPE_F32)
		  {
		    printf("%f ",numeric_vals[i]);
		  }
		else
		  {
		    printf("%i ",((i32 *) numeric_vals)[i]);
		  }	
			      	
		fflush(stdout);
	      }
	    //Send all in one packet
	    //send(fdin, (void *)data_string, data_length,0);
			      
	    full_response = calloc(1,sizeof(respstruct_t)+data_length);
	    memcpy(full_response,&response,sizeof(respstruct_t));
	    memcpy(full_response+sizeof(respstruct_t),data_string,data_length);
			     	
	    //Only send packed numeric data due to parser
	    send(fdin, (void *) full_response, sizeof(respstruct_t)+data_length,0); 
			      	
	    free(data_string);
	    free(full_response);
	  }

	break;
			  
      case LBNL_GET_CFG_STRING_ARRAY:
	printf ("cmd: GET_CFG_ARRAY %i\n", message.data[0]);
	if (1)
	  {
	    char *data_string;
	    char *numeric_start;
	    f32 *numeric_vals;
	    char *crlf = "\r\n";	//possibly needed for EPICS parsing
	    int data_length;
	    int i,j,k;
	    int nelm;
	    f32 *print_data;
	    char *full_response;

	    data_string = lbnl_param_to_string(&config_params[message.data[0]]);
	    data_length = strlen(data_string);
			      
	    //Only send packed numeric data due to parser
	    send(fdin, (void *) data_string, data_length,0); 
			      	
	    free(data_string);
			      
	  }

	break;
      case LBNL_IMG_ACQ:
	printf ("cmd: IMG_ACQ \n");
	
	sem_status = sem_trywait(&acq_state);
	if (sem_status) /* Currently acquiring , so don't acquire again */
	  {
	    sprintf( response.strmsg, "ACQBUSY");
	    send(fdin, (void *)&response, sizeof(respstruct_t),0);
	    break;
	  }
	//XXX FIXME How to deal with inconsistent state between semaphore and mutex
	pthread_mutex_lock(&acq_state_mutex);
	img_acq_state = Not_Started;
	pthread_mutex_unlock(&acq_state_mutex);

	exp_args = (exp_param_t *)malloc(sizeof(exp_param_t));
	exp_args->acq_mode = message.data[0];
	exp_args->ccd_clear = message.data[1];

	create_thread_ret = pthread_create(&acq_thread, NULL, &pt_take_picture, exp_args);
	if (create_thread_ret == 0)
	{
		sprintf( response.strmsg, "DONE");
	}
	else
	{
		perror("pthread_create error: ");
		sprintf( response.strmsg, "THREAD_FAIL");
	}
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_IMG_READ:
	printf ("cmd: IMG_READ \n");
	sem_status = sem_trywait(&acq_state);
	if (sem_status)
	  {
	    //FIXME XXX Client code expects a full image, so this may
	    /* Cause problems if read without semaphore */
	    sprintf( response.strmsg, "ACQBUSY");
	    send(fdin, (void *)&response, sizeof(respstruct_t),0);
	    break;
	  }
	pt_read_picture(&fdin);
	//sprintf( response.strmsg, "DONE");
	//send(fdin, (void *)&response, sizeof(respstruct_t),0);
			  
	break;
			  
      case LBNL_SHUTDOWN:
	printf ("received SHUTDOWN\n");
	got_sigterm=1;
	close (fdin);
	fdin = 0;
	cin = 0;
	close (listenfd);
	break;

      case LBNL_CMD_RESET:
	//FIXME TODO Add in error checking
	printf("Reset system");
	lbnl_controller_master(dfd, 0);
	lbnl_controller_power(dfd, 1);
	lbnl_sleep_ms(100);
	lbnl_init(dfd);
	lbnl_ccd_idle(dfd, 1);
	//lbnl_close(dfd);
	response.status=0;
	/* XXX Should probably have a semaphore on below line */
	img_count_reset = 0;
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	/* Set ability to read temperature flag; may want this as part of error checking */
	pthread_mutex_lock(&temp_ability_mutex);
	temp_read_ability = temp_read_ability | TEMP_READ_RESET; /* Set flag we just did */
	pthread_mutex_unlock(&temp_ability_mutex);
	break;
      case LBNL_CMD_ENABLE:
	//FIXME TODO Add error checking and user-determined masking
	lbnl_controller_enable(dfd, DACMASK, CLKMASK);
	response.status=0;
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	/* Set ability to read temperature flag; may want this as part of error checking */
	pthread_mutex_lock(&temp_ability_mutex);
	temp_read_ability = temp_read_ability | TEMP_READ_ENABLE; /* Set flag we just did */
	pthread_mutex_unlock(&temp_ability_mutex);

	break;
      case LBNL_CMD_CONFIG:
	//FIXME TODO Add error checking and user-defined files
	if (1)
	  {
	    FILE *cfg_file;
	    char line[256];
	    int delays[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	    char *DAC_ptr, *CLK_ptr, *OFF_ptr, *TIM_ptr, *data_ptr, *sec_ptr;
	    int addr = -1;			// Start at address 0x0
	    int data = 0;
	    u32 gain = 5;
	    f32 fdata = 0.0;
	    f32 sec = 0;
	    int i;
	    int error = 0, c;
	    int level;
	    // Data structures for DAC and Clock readback
	    dac_t dacs[20];
	    u16 ndacs;
	    lbnl_clock_t clocks[20];
	    u16 nclocks;

	    char *timing_file = getenv("CCDCONTROLLER_TIMING_FILE");
	    if (timing_file == NULL) {
	      timing_file = "/home/ccd/timing.txt";
	    }
	    printf("Using timing file %s\n", timing_file);
	    error = lbnl_controller_upload_timing(dfd, timing_file);
	    printf("Timing: %d\n", error);

	    cfg_file = fopen("/data/ccd.cfg","r");
	    while (fgets(line, sizeof(line), cfg_file))
	      {
		DAC_ptr = strstr(line, "D");
		CLK_ptr = strstr(line, "C");
		OFF_ptr = strstr(line, "O");
		TIM_ptr = strstr(line, "T");
		data_ptr = strstr(line, "|");
		sec_ptr = strstr(line, "/");
		if (data_ptr) {		// Don't scan line if there is no data in it
		  if (DAC_ptr != NULL ) {
		    sscanf(DAC_ptr + 1, "%d", &addr);
		    sscanf(data_ptr + 1, "%f", &fdata);
		    printf("DAC addr %d, data %f\n",addr,fdata);
		    error += lbnl_controller_set_dac_value(dfd, addr, fdata);
		  }
		  if ((CLK_ptr != NULL ) & (sec_ptr != NULL )) {
		    sscanf(CLK_ptr + 1, "%d", &addr);
		    sscanf(data_ptr + 1, "%f", &fdata);
		    sscanf(sec_ptr + 1, "%f", &sec);
		    printf("CLK addr %d, data %f, %f\n",addr,fdata,sec);
		    error += lbnl_controller_set_clk_value(dfd, addr, fdata, sec);
		  }
		  if (OFF_ptr != NULL ) {
		    sscanf(OFF_ptr + 1, "%d", &addr);
		    sscanf(data_ptr + 1, "%f", &fdata);
		    printf("OFF addr %d, data %f\n",addr,fdata);
		    error += lbnl_controller_set_offset_value (dfd, addr, fdata);
		  }
		  if (TIM_ptr != NULL ) {
		    sscanf(TIM_ptr + 1, "%d", &addr);
		    sscanf(data_ptr + 1, "%d", &data);
		    printf("TIM addr %d, data %d\n",addr,data);
		    delays[addr] = data;
		  }
		}
	      }
	    fclose(cfg_file);

	    // Read back current controller settings
	    error = lbnl_controller_get_all_dacs(dfd, dacs, &ndacs);
	    printf("get_all_dacs error code: %d\n", error);

	    for (i=0; i<NDACS; i++){
	      printf("Address: %d:\ttelemetry: %f\ttvalue: %f\n", dacs[i].address, dacs[i].telemetry, dacs[i].tvalue);
	    }

	    error = lbnl_controller_get_all_clocks(dfd, clocks, &nclocks);
	    printf("get_all_clocks error code: %d\n", error);
	    for (i=0; i < NCLOCKS; i++){
	      printf("Address: %d\ttelemetry:\t%f\tthightvalue:\t%f\ttlowtvalue:\t%f\n", clocks[i].address, clocks[i].telemetry, clocks[i].thigh_value, clocks[i].tlow_value);
	    }

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
	    error = lbnl_controller_set_delays(dfd, ccd_delay);
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
	    error = lbnl_controller_set_cds(dfd, cds);
	    printf("set_cds error code: %d\n", error);
	    if(delays[15]==0)
	      gain = 5;
	    else	gain = delays[15];
	    error = lbnl_controller_set_gain(dfd, gain);
	    printf("set gain gain: %d error code: %d\n", gain,error);

	    // Enable controller and set start address (timing file)
	    //	error = lbnl_controller_enable(dfd, DACMASK, CLKMASK);
	    //	printf("enable error code: %i\n", error);
	    error = lbnl_controller_set_start_address(dfd, 32);
	    printf("set_start_address error code: %i\n", error);

	  }

	response.status=0;
	send(fdin, (void *)&response, sizeof(respstruct_t),0);

	break;
	//lbnl_close

      case LBNL_GET_ACQ_STATUS:
	sprintf(response.strmsg, "DONE");
	pthread_mutex_lock(&acq_state_mutex);
	response.data[0] = img_acq_state;
	pthread_mutex_unlock(&acq_state_mutex);
	//printf("img_acq_state is %i.\n", response.data[0]);
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	break;

      case LBNL_GET_IMG_CNT:
	sprintf(response.strmsg, "DONE");
	response.data[0] = img_count_reset;
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_GET_IMG_HALF:
	printf("cmd: LBNL_GET_IMG_HALF: %i\n",b_read_fast);
	sprintf(response.strmsg, "DONE");
	response.data[0] = b_read_fast;
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_SET_IMG_HALF:
	printf("cmd: LBNL_SET_IMG_HALF: %i\n",message.data[0]);
	b_read_fast = message.data[0];
	sprintf(response.strmsg, "DONE");
	response.data[0] = 0;
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_GET_CCDSIZE:
	printf("cmd: LBNL_GET_CCDSIZE: %i %i\n", ccd_size_x, ccd_size_y);
	sprintf(response.strmsg, "DONE");
	response.data[0] = ccd_size_x;
	response.data[1] = ccd_size_y;
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_SET_SHUTTERMODE:
	printf("cmd: LBNL_SET_SHUTTERMODE: %i\n", message.data[0]);
	sprintf(response.strmsg, "DONE"); /* Override on error below */
	if (message.data[0] == SHUTTER_MODE_NORMAL)
	  {
	    shutter_mode = SHUTTER_MODE_NORMAL;
	    /* TODO Set according to exposure state */
	    pthread_mutex_lock(&bg_state_mutex);
	    if (take_as_bg != 0) /* Background, so leave shutter closed */
	      {
		pthread_mutex_unlock(&bg_state_mutex);
		lbnl_controller_set_shutter(dfd, SHUTTER_CLOSE);
	      }
	    else if (b_video_mode) /* Videomode, so open shutter */
	      {
		pthread_mutex_unlock(&bg_state_mutex);
		lbnl_controller_set_shutter(dfd, SHUTTER_OPEN);
	      }
	    else		/* Not taking as background, so set according to state */
	      {
		pthread_mutex_unlock(&bg_state_mutex);
		pthread_mutex_lock(&acq_state_mutex); /* Check state and lock */
		if (img_acq_state == Exposing)	      /* Exposing, so open shutter */
		  {
		    lbnl_controller_set_shutter(dfd, SHUTTER_OPEN); /*Open - 0, Closed - 1  */
		  }
		else 		/* Not exposing, so close */
		  {
		    lbnl_controller_set_shutter(dfd, SHUTTER_CLOSE); /*Open - 0, Closed - 1  */
		  }
		pthread_mutex_unlock(&acq_state_mutex);
	      }
	  }
	else if (message.data[0] == SHUTTER_MODE_OPEN)
	  {
	    shutter_mode = SHUTTER_MODE_OPEN;
	    iaux1 = lbnl_controller_set_shutter(dfd, SHUTTER_OPEN); /* Open shutter */
	  }
	else if (message.data[0] == SHUTTER_MODE_CLOSED)
	  {
	    shutter_mode = SHUTTER_MODE_CLOSED;
	    iaux1 = lbnl_controller_set_shutter(dfd, SHUTTER_CLOSE);
	  }
	else			/* Error condition */
	  {
	    sprintf (response.strmsg, "Invalid shutter mode %i.\n", message.data[0]);
	  }
	if (iaux1)
	  {
	    printf("****************\nError %i on set shutter mode\n**************\n", iaux1);
	  }
	send (fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_GET_SHUTTERMODE:
	//printf("cmd: LBNL_GET_SHUTTERMODE\n");
	sprintf(response.strmsg, "DONE");
	response.data[0] = 0;	/* Clear since get shutter is i8* */
	/* XXX FIXME 2-bit value in code, but lbnl_controller_get_shutter says 0 or 1  */
	/* TODO Add error checking */
	iaux1 = lbnl_controller_get_shutter(dfd, (char *) &response.data[0]);
	if (iaux1)
	  {
	    printf("**************************\nError on Get Shutter Mode\n******************\n");
	  }
	//printf("**********************\nShutter Value: 0x%X\nTake as BG: %i\n********************\n", response.data[0], take_as_bg);
	send(fdin, (void *)&response, sizeof(respstruct_t), 0);
	break;
      case LBNL_SET_FAST_MODE:
	printf("LBNL_FAST_MODE: %i.\n", message.data[0]);
	if (message.data[0] == 0) /* Normal mode */
	  {
	    lbnl_controller_set_cds(dfd, daemon_cds_normal);
	    lbnl_controller_set_delays(dfd, daemon_ccd_delay_normal);
	    b_video_mode = 0;
	  }
	else			/* Fast mode */
	  {
	    lbnl_controller_set_cds(dfd, daemon_cds_fast);
	    lbnl_controller_set_delays(dfd, daemon_ccd_delay_fast);
	    b_video_mode = 1;
	  }
	
	sprintf(response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_SET_BG:
	printf("LBNL_SET_BG: %i\n", message.data[0]);
	pthread_mutex_lock(&bg_state_mutex);
	if (message.data[0])
	  {
	    take_as_bg = 1;
	  }
	else
	  {
	    take_as_bg = 0;
	  }
	pthread_mutex_unlock(&bg_state_mutex);
	sprintf(response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_RST_TIMER:
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	sprintf(response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_RD_TIMER:
	clock_gettime(CLOCK_MONOTONIC, &curr_time);
	elapsed_time = curr_time.tv_sec - start_time.tv_sec;
	response.data[0] = elapsed_time;
	sprintf(response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_GET_BG:
	pthread_mutex_lock(&bg_state_mutex);
	response.data[0] = take_as_bg;
	pthread_mutex_unlock(&bg_state_mutex);
	printf("LBNL_GET_BG: BG value is %i.\n",take_as_bg);
	sprintf(response.strmsg, "DONE");
	send(fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_GET_PIX_SIZE:
	*((float *) &response.data[0]) = pix_width;
	*((float *) &response.data[1]) = pix_height;
	sprintf(response.strmsg, "DONE");
	send(fdin, (void *)&response, sizeof(response),0);
	break;
      case LBNL_READ_AUX_PORT:
    	  aux_port_cnt++;
	lbnl_controller_read_aux(dfd, &aux_port_val);
	sprintf(response.strmsg, "DONE");
	response.data[0] = aux_port_val;
	response.data[1] = aux_port_cnt;
	send(fdin, (void *)&response, sizeof(response),0);
	break;
      case LBNL_SET_TRIG_MODE:
	printf("Set trigger mode to %i.\n", (int) message.data[0]);
	sem_status = sem_trywait(&acq_state);
	if (sem_status) 	/* Currently exposing or reading out, so don't change */
	  {
	    printf("Couldn't set trigger mode.\n");
	    sprintf( response.strmsg, "ACQBUSY");
	    response.data[0] = -1; /* Report error on response */
	  }
	else			/* Can change trigger mode */
	  {
	    pthread_mutex_lock(&trig_mode_mutex);
	    trig_mode = message.data[0]; /* Set trigger mode */
	    response.data[1] = trig_mode; /* Report trigger mode back */
	    response.data[0] = 0; /* Signal success of changing trig mode */
	    pthread_mutex_unlock(&trig_mode_mutex);
	    sem_post(&acq_state); /* Release acq state semaphore */
	    printf("Set trigger mode.\n");
	  }
	send(fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_GET_TRIG_MODE:
	pthread_mutex_lock(&trig_mode_mutex);
	response.data[0] = trig_mode;
	pthread_mutex_unlock(&trig_mode_mutex);
	sprintf(response.strmsg, "DONE");
	send(fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_READ_GPIO:
	iaux1 = message.data[0]; /* Get port number */
	response.data[0] = lbnl_controller_read_gpio(dfd, message.data[0], &response.data[1]);
	sprintf(response.strmsg, "DONE");
	send(fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_READ_HW_TRIG:
	/* Get current value of hardware trigger, then clear it */
	pthread_mutex_lock(&trig_mon_mutex);
	response.data[0] = hw_trig_detected;
	hw_trig_detected = 0;	/* Clear after read */
	pthread_mutex_unlock(&trig_mon_mutex);
	sprintf(response.strmsg, "DONE");
	send(fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_CFG_NAME:
	bzero(response.strmsg, sizeof(response.strmsg)); /* Blank to zero */
	strcpy(response.strmsg, cfg_filename);		 /* Copy in name */
	send(fdin, (void *)&response, sizeof(response), 0);
	break;
      case LBNL_TIM_NAME:
	bzero(response.strmsg, sizeof(response.strmsg)); /* Blank to zero */
	strcpy(response.strmsg, tim_filename);		 /* Copy in name */
	send(fdin, (void *)&response, sizeof(response), 0);
	break;
      default:
	sprintf (response.strmsg, "ERROR unknown cmd %d\n",message.cmd);
	response.status = -EINVAL;
	response.data[0] = response.status;
	send (fdin, (void *)&response, sizeof (response),0);
	break;

      }
    }
  } while (cin>0);

  if (fdin != 0)
    close(fdin);

  printf ("client terminated\n");
  if (dfd != 0) {
    lbnl_close (dfd);
    printf ("cmd: CLOSE\n");
    dfd = 0;
  }
  // sleep (1);
  cin = 0;
  return(NULL);
}

void signal_handler(int sig) {
  switch(sig) {
  case SIGHUP:
    syslog(LOG_INFO, "hangup signal catched");
    break;
  case SIGTERM:
    syslog(LOG_INFO, "terminate signal catched");
    got_sigterm=1;
    break;
  }
}

int lbnl_test_mode()
{
  printf("Entering test mode.\n");
  return 0;
}

void *pt_take_fits(void *arg)
{
  int fdin = *((int *) arg);
  int ret;

  //pthread_mutex_lock(&acq_state);

  //TODO Make thread safe?
  exp_abort = 0;		/* Clear old abort command, if any */
  wait_exposure_time();
  if ((ret=lbnl_ccd_read_up_or_down(dfd, imbuffer, 3))!=0)
    {
      printf ("ERROR acquiring %d\n",ret);
      //sprintf (response.strmsg, "ERROR acquiring %d\n",ret);
    }
  else
    {
      char file_name_full[200];
      char file_count_ext[200];
      time_t curr_time_t;
      struct tm *curr_tm;
      printf ("image acquired, writing\n");
      
      //FIXME TODO Robustify
      //FIXME XXX TODO Note that this is not thread-safe
      //Get the current time to timestamp the file
      curr_time_t = time(NULL);
      curr_tm = gmtime(&curr_time_t);
      strftime(file_name_full, sizeof(file_name_full),"/data/image_%Y%m%d_%H%M%S_", curr_tm);
      
      //Create the string with the count and the extension
      sprintf(file_count_ext, "%0.10u.fits", image_num);
      strcat(file_name_full, file_count_ext);
      image_num++;	//Increment for next image
      
      if ((ret=lbnl_readout_get_fits (dfd, file_name_full, imbuffer))!=0){
	printf ("ERROR writing %d\n",ret);
      } else {
	printf ("DONE\n");
      }
    }
  img_acq_state = 0;
  //pthread_mutex_unlock(&acq_state);
  return NULL;
}
  
void *pt_take_picture(void * arg)
{
  int wait_status = 0;
  exp_param_t *exp_args = (exp_param_t *)arg;
  u16 *half_buffer[2];		/* Point to the upper (0) and 
				   lower (1) buffers */
  int updown_idx;		/* Whether we are iterating over up or down
				   buffer */
  int end_row, half_row;	/* Last and half-rows of image */
  int row_idx, col_idx;		/* Indices for copying from one buffer
				   to imbuffer */
  int read_mode;		       /* How to read out -- calculated once */
  const int READ_DOUBLE = 0;	/* Read up, then down */
  const int READ_INTERLEAVED = 1; /* Read interleaved */
  const int READ_SINGLE_DIRECTION = 2; /* Read only up */
  const int FLIP_UNIDIR = 0;	       /* Whether to perform horiz flip for
					  uni-directional clear.  */
  //TODO FIXME Add clearing logic and update status variable
  //TODO Change behavior based on return from waiting
  //TODO Make abort flag thread safe?

  //Be sure to detach the thread on completion
  pthread_detach(pthread_self());

  /* Assign the buffer pointers */
  half_buffer[0] = up_buffer;
  half_buffer[1] = down_buffer;
  
  /* DEBUGGING REMOVETHIS */
  printf("Acquisition mode %i, clear ccd %i.\n",
  	 exp_args->acq_mode, exp_args->ccd_clear);

  set_shutter(SHUTTER_CLOSE); /* Close shutter, modified automaticall by mode */

  
  /* Set read mode according to chosen menu setting and fast mode setting*/
  read_mode = READ_DOUBLE;	/* Arbitrarily chosen */
  if (b_video_mode)		/* Video mode */
    {
      read_mode = READ_SINGLE_DIRECTION;
    }
  else				/* Normal mode */
    {
      read_mode = b_read_fast;	/* Choose the selected scheme */
    }


  /* Branch depending on Read Mode */
  if (read_mode == READ_DOUBLE)
    {
      for (updown_idx=0; updown_idx<2; updown_idx++) /* 0 Read up, 1 read down */
	{
	  if (exp_args->ccd_clear)
	    {
	      printf("****************\nClearing CCD\n****************\n");
	      pthread_mutex_lock(&acq_state_mutex);
	      img_acq_state = Clearing;
	      pthread_mutex_unlock(&acq_state_mutex);
	      if (updown_idx==0) {
		/* Reading up, so clearing up. */
		lbnl_ccd_clear_up_or_down(dfd, 1);
		
	      } else if (updown_idx==1) {
		/* Reading down, so clearing down */
		lbnl_ccd_clear_up_or_down(dfd, 2);
		
	      }
	    }
	  
	  pthread_mutex_lock(&acq_state_mutex);
	  img_acq_state = Exposing;
	  pthread_mutex_unlock(&acq_state_mutex);
	  set_shutter(SHUTTER_OPEN);
	  wait_status = wait_exposure_time();
	  
	  if (wait_status)
	    {
	      exp_abort = 0;		/* Clear abort flag so we do not trigger
					   on next run.  This also allows an abort on
					   the other steps to trigger here.*/
	      goto end_exp;		/* Exposure aborted */
	    }
	  
	  set_shutter(SHUTTER_CLOSE);

	  pthread_mutex_lock(&acq_state_mutex);
	  img_acq_state = Readout;
	  pthread_mutex_unlock(&acq_state_mutex);
	  if (updown_idx == 0)
	    {
	      lbnl_ccd_read_up_or_down(dfd, half_buffer[updown_idx], 1);
	      /* Read up */
	    }
	  else
	    {
	      lbnl_ccd_read_up_or_down(dfd, half_buffer[updown_idx], 2);
	      /* Read down */
	    }
	  
	  //printf("&&&&&&&&&&&&&&&&&&&&&&Performing iteration %i of updown.\n",updown_idx);
	}
    }
  else if (read_mode == READ_INTERLEAVED) /* Interleaved read */
    {
      if (exp_args->ccd_clear)
	{
	  printf("*************\nClearing CCD\n***************\n");
	  pthread_mutex_lock(&acq_state_mutex);
	  img_acq_state = Clearing;
	  pthread_mutex_unlock(&acq_state_mutex);
	  lbnl_ccd_clear_up_or_down(dfd, 3);
	}

      pthread_mutex_lock(&acq_state_mutex);
      img_acq_state = Exposing;
      pthread_mutex_unlock(&acq_state_mutex);

      set_shutter(SHUTTER_OPEN);
      wait_status = wait_exposure_time();


      if (wait_status)
	{
	  exp_abort = 0;	/* Clear abort flag so we do not trigger on 
				 next run.  This also allows an abort on 
				the other steps to trigger here.*/
	  goto end_exp;
	}
      
      set_shutter(SHUTTER_CLOSE);


      pthread_mutex_lock(&acq_state_mutex);
      img_acq_state = Readout;
      pthread_mutex_unlock(&acq_state_mutex);
      lbnl_ccd_read_up_or_down(dfd, half_buffer[0], 3); /* Read up, into up buffer */
    }
  else 				/* Single direction read, specifically UP */
    {
      if (exp_args->ccd_clear)
	{
	  printf("*****************\nClearing CCD\n**************\n");
	  pthread_mutex_lock(&acq_state_mutex);
	  img_acq_state = Clearing;
	  pthread_mutex_unlock(&acq_state_mutex);
	  lbnl_ccd_clear_up_or_down(dfd, 1);
	}

      pthread_mutex_lock(&acq_state_mutex);
      img_acq_state = Exposing;
      pthread_mutex_unlock(&acq_state_mutex);

      set_shutter(SHUTTER_OPEN);

      wait_status = wait_exposure_time();

      if (wait_status)
	{
	  exp_abort = 0;	/* Clear abort flag so we do not trigger on 
				 next run.  This also allows an abort on
				the other steps to trigger here.*/
	  goto end_exp;
	}

      set_shutter(SHUTTER_CLOSE);
      pthread_mutex_lock(&acq_state_mutex);
      img_acq_state = Readout;
      pthread_mutex_unlock(&acq_state_mutex);
      lbnl_ccd_read_up_or_down(dfd, half_buffer[0], 1); /* Read up, into up buffer */
    }

  /* Assemble the image according to the read mode */

  // Blank out destination array
  for (row_idx=0; row_idx<img_size_y; row_idx++)
    {
      for (col_idx =0; col_idx<img_size_x; col_idx++)
	{
	  int buf_pixel;
	  buf_pixel = row_idx*img_size_x+col_idx;
	  imbuffer[buf_pixel]=0;
	}
    }

  /* Top half of the image is always the same */

  end_row = img_size_y;
  half_row = end_row/2;

  
  /* Copy upper half */
  for (row_idx = 0; row_idx < half_row; row_idx ++)
    {
      for (col_idx = 0; col_idx < img_size_x; col_idx++)
	{
	  int buffer_pix;
	  
	  buffer_pix = (row_idx*img_size_x) + col_idx;
	  imbuffer[buffer_pix] = up_buffer[buffer_pix];
	}
      //printf("#################\nCopying row from upper half.\n###################\n");
    }
  

  /* Now branch based on read mode  */
  if (read_mode == READ_DOUBLE)
    {
      /* Assemble the image from the bottom half of the down buffer */
      for (row_idx = half_row; row_idx < end_row; row_idx++)
	{
	  for (col_idx = 0; col_idx < img_size_x; col_idx++)
	    {
	      int buffer_pix;
	      
	      buffer_pix = (row_idx*img_size_x) + col_idx;
	      imbuffer[buffer_pix] = down_buffer[buffer_pix];
	    }
	  //printf("@@@@@@@@@@@@@@@@@@@@\nCopying row from lower half.\n@@@@@@@@@@@@@@@@@@@@@@@@\n");
	}
    }
  else if (read_mode == READ_INTERLEAVED) /* Interleaved read */
    {
      /* Copy bottom half of image from bottom half of up buffer, since data
	 were all read into up_buffer */
      for (row_idx = half_row; row_idx < end_row; row_idx++)
	{
	  for (col_idx = 0; col_idx < img_size_x; col_idx++)
	    {
	      int buffer_pix;
	      buffer_pix = (row_idx*img_size_x)+col_idx;
	      imbuffer[buffer_pix] = up_buffer[buffer_pix];
	    }
	}
    }
  else 				/* Unidirectional read */
    {
      /* Clone upper half into lower half */

      int upper_row_idx, bottom_row_idx;
      int upper_col_idx, bottom_col_idx;
      int half_col = img_size_x/2;
      int overscan_offset;
      
      overscan_offset = (half_row-ccd_size_y);
      
      if (FLIP_UNIDIR)
	{
	  for (row_idx=0; row_idx<half_row; row_idx++)
	    {
	      upper_row_idx = row_idx; //Copy from top half to bottom half, with
	      //offset for overscan
	      bottom_row_idx = ((row_idx+overscan_offset)%half_row)+half_row;
	      /* Copy lower-left to lower right */
	      for (upper_col_idx = 0; upper_col_idx < half_col; upper_col_idx++)
		{
		  int upper_buffer_pix, bottom_buffer_pix;
		  bottom_col_idx = img_size_x-1-upper_col_idx;
		  upper_buffer_pix = upper_row_idx*img_size_x+upper_col_idx;
		  bottom_buffer_pix = bottom_row_idx*img_size_x+bottom_col_idx;
		  
		  imbuffer[bottom_buffer_pix] = up_buffer[upper_buffer_pix];
		}
	      /* Copy lower-right to lower left */
	      for (upper_col_idx = half_col; upper_col_idx < img_size_x; upper_col_idx++)
		{
		  int upper_buffer_pix, bottom_buffer_pix;
		  bottom_col_idx = half_col-1-(upper_col_idx-half_col);
		  upper_buffer_pix = upper_row_idx*img_size_x+upper_col_idx;
		  bottom_buffer_pix = bottom_row_idx*img_size_x+bottom_col_idx;
		  
		  imbuffer[bottom_buffer_pix] = up_buffer[upper_buffer_pix];
		}
	    }
	}
      else			/* Not flipping horizontally */
	{
	  for (row_idx=0; row_idx<half_row; row_idx++)
	    {
	      upper_row_idx = row_idx; //Copy from top half to bottom half, with
	      //offset for overscan
	      bottom_row_idx = ((row_idx+overscan_offset)%half_row)+half_row;
	      /* Copy lower-left to lower left */
	      for (upper_col_idx = 0; upper_col_idx < half_col; upper_col_idx++)
		{
		  int upper_buffer_pix, bottom_buffer_pix;
		  //bottom_col_idx = img_size_x-1-upper_col_idx;
		  bottom_col_idx = upper_col_idx;
		  upper_buffer_pix = upper_row_idx*img_size_x+upper_col_idx;
		  bottom_buffer_pix = bottom_row_idx*img_size_x+bottom_col_idx;
		  
		  imbuffer[bottom_buffer_pix] = up_buffer[upper_buffer_pix];
		}
	      /* Copy lower-right to lower right */
	      for (upper_col_idx = half_col; upper_col_idx < img_size_x; upper_col_idx++)
		{
		  int upper_buffer_pix, bottom_buffer_pix;
		  //bottom_col_idx = half_col-1-(upper_col_idx-half_col);
		  bottom_col_idx = upper_col_idx;
		  upper_buffer_pix = upper_row_idx*img_size_x+upper_col_idx;
		  bottom_buffer_pix = bottom_row_idx*img_size_x+bottom_col_idx;
		  
		  imbuffer[bottom_buffer_pix] = up_buffer[upper_buffer_pix];
		}
	    }
	
	}
    }

    
        
  /* Increment image counter */
  img_count_reset++;
  
  /* TODO Add state changes as we progress through multi-stage read */
  pthread_mutex_lock(&acq_state_mutex);
  img_acq_state = Done;
  pthread_mutex_unlock(&acq_state_mutex);
    
 end_exp:
  set_shutter(SHUTTER_CLOSE);	/* Can jump here after exposure aborted, so need to close shutter */
  sem_post(&acq_state);

  free(arg);
  return NULL;
}

void *pt_read_picture(void *arg)
{
  int fdin = *((int *) arg);
  //FIXME TODO Update status variable as progress is made
  pthread_mutex_lock(&acq_state_mutex);
  img_acq_state = Sending;
  pthread_mutex_unlock(&acq_state_mutex);

  int curr_bytes_sent, total_bytes_sent=0;

  //Be sure to detach the thread on completion
  pthread_detach(pthread_self());


  //XXX TODO Robustify Eliminate magic numbers
  while (total_bytes_sent < (img_size_x*img_size_y*sizeof(imbuffer[0])))
    {
      int start_byte, stop_byte;
      //const int NUM_TO_SEND = 2085120;
      const int NUM_TO_SEND = 25000000;
     
      start_byte = total_bytes_sent;
      stop_byte = start_byte + NUM_TO_SEND-1;
      //printf("Byte %i: %hx\n", start_byte, imbuffer[start_byte]);
      if (stop_byte >= (img_size_x*img_size_y*sizeof(imbuffer[0])))
	{
	  stop_byte = (img_size_x*img_size_y*sizeof(imbuffer[0])-1);
	}
      printf("Start byte: %u, Stop byte %u, Max byte %u.\n",
	     start_byte, stop_byte,
	     img_size_x*img_size_y*sizeof(imbuffer[0])-1);
      
      curr_bytes_sent = send(fdin, ((char *)imbuffer)+start_byte, stop_byte-start_byte+1, 0);
      printf("Sent %li bytes, %li total.\n", curr_bytes_sent, total_bytes_sent);
      if (curr_bytes_sent == -1)
	{
	  perror("");
	  break;
	}
      total_bytes_sent += curr_bytes_sent;

      //usleep(500000);

    }
  
  pthread_mutex_lock(&acq_state_mutex);
  img_acq_state = Done;
  pthread_mutex_unlock(&acq_state_mutex);

  sem_post(&acq_state);
  return NULL;
}

int checkerboard()
{
  /* Create a checkerboard pattern in imbuffer */
  int i,j,k;
  for (i=0; i<4200; i++)
    {
      for (j=0; j<4200; j++)
	{
	  int x_mod, y_mod, pix_loc;
	  x_mod = (i/50)%2;
	  y_mod = (j/50)%2;
	  pix_loc = i*4200+j;
	  if (x_mod ^ y_mod)
	    {
	      /* make black */
	      imbuffer[pix_loc] = 0;
	    }
	  else
	    {
	      /* make white */
	      imbuffer[pix_loc] = 65535;
	    }
	}
    }
  return 0;
}

/* TODO Add capability of interrupting timer */
int wait_exposure_time()
{
  struct timeval start_time, curr_time, diff_time;

  /* If in video mode, we want to return early */
  if (b_video_mode)
    {
      if (exp_abort)
	{
	  pthread_mutex_lock(&acq_state_mutex);
	  img_acq_state = Aborted;
	  pthread_mutex_unlock(&acq_state_mutex);
	  return 1;		/* Return img_acq aborted */
	}
      else
	{
	  return 0;
	}
    }

  gettimeofday(&start_time, NULL);

  while (1)
    {
      usleep(100000);
      gettimeofday(&curr_time, NULL);
      timersub(&curr_time, &start_time, &diff_time);
      exp_time_elapsed = diff_time.tv_sec;

      if (exp_abort)
	{
	  pthread_mutex_lock(&acq_state_mutex);
	  img_acq_state = Aborted;
	  pthread_mutex_unlock(&acq_state_mutex);
	  return 1;		/* Return img_acq aborted */
	}
      if (exp_time_elapsed >= exp_time)
	{
	  break;
	}
    }

  return 0;			/* All went well */
}

/* Set shutter, taking into account shutter mode and "Take as BG" mode */
void set_shutter(int new_state)
{
  pthread_mutex_lock(&bg_state_mutex);
  printf("###########################\nSHUTTERMODE: %i\n:TakeAsBG: %i\nAcqState: %i\n###################\n",shutter_mode, take_as_bg, img_acq_state);
  pthread_mutex_unlock(&bg_state_mutex);

  if (shutter_mode == SHUTTER_MODE_OPEN || shutter_mode == SHUTTER_MODE_CLOSED) /* Forcing mode */
    {
      /* Do nothing */
      return;
    }

  pthread_mutex_lock(&bg_state_mutex);
  if (take_as_bg)		/* Taking background image */
    {
      pthread_mutex_unlock(&bg_state_mutex);
      /* Close the shutter */
      lbnl_controller_set_shutter(dfd, SHUTTER_CLOSE);
      return;
    }
  else if (b_video_mode)
    {
      pthread_mutex_unlock(&bg_state_mutex);
      /* Open the shutter */
      lbnl_controller_set_shutter(dfd, SHUTTER_OPEN);
    }
  else				/* Normal mode and not bg */
    {
      pthread_mutex_unlock(&bg_state_mutex);
      lbnl_controller_set_shutter(dfd, new_state);
      return;
    }

  
}


void populate_slow_mode(cds_t *curr_cds, delays_t *curr_delays)
{
  lbnl_controller_get_cds(dfd, curr_cds);
  lbnl_controller_get_delays(dfd, curr_delays);
  return;
}

void populate_fast_mode(const cds_t *cds_normal, const delays_t *ccd_delay_normal, cds_t *cds_fast, delays_t *ccd_delay_fast)
{
  *cds_fast = *cds_normal;
  *ccd_delay_fast = *ccd_delay_normal; /* Copy over existing values */

  ccd_delay_fast->clock_parallel = 14400; /* Set delay parameter to new value */
  
  cds_fast->averaging = 5;
  cds_fast->digioff = 2000;
  cds_fast->nsamp_reset = 32;
  cds_fast->nsamp_signal = 32;

  return;
}

void *pt_read_temperature(void *arg)
{
  unsigned int b_temp_ready;
  unsigned int temp_ready_value;
  float temp_1_aux, temp_2_aux;	/* Auxilliary variables for holding temperatures */
  int lbnl_ret;

  /* Loop forever, once per second approx */
  while (1)
    {
      printf("Temp monitor thread reading temperature.\n");
      fflush(stdout);
      /* See if we can read the temperature */
      pthread_mutex_lock(&temp_ability_mutex);
      if (temp_read_ability == TEMP_READ_DONE) /* All flags set, no extras */
	{
	  b_temp_ready = 1;		/* Set so we can read */
	}
      else				/* All flags not set */
	{
	  b_temp_ready = 0;		/* We cannot read */
	}
      pthread_mutex_unlock(&temp_ability_mutex);
      
      /* Branch on ability to read temperature */
      
      if (b_temp_ready)		/* Can read temperature, so talk SPI */
	{
	  lbnl_ret = lbnl_controller_get_temps(dfd, &temp_1_aux, &temp_2_aux); /* Read temperature */
	  if (lbnl_ret)		/* Error */
	    {
	      /* Report invalid temperatures and error code */
	      pthread_mutex_lock(&temp_value_mutex);
	      temperature_1 = TEMP_INVALID;
	      temperature_2 = TEMP_INVALID;
	      temperature_error = lbnl_ret;
	      pthread_mutex_unlock(&temp_value_mutex);
	    }
	  else			/* No error in read */
	    {
	      pthread_mutex_lock(&temp_value_mutex);
	      temperature_1 = temp_1_aux;
	      temperature_2 = temp_2_aux;
	      temperature_error = 0;
	      pthread_mutex_unlock(&temp_value_mutex);
	    }
	}
      else			/* Can't read temperature */
	{
	  pthread_mutex_lock(&temp_value_mutex);
	  temperature_1 = TEMP_INVALID; /* Report invalid temperatures */
	  temperature_2 = TEMP_INVALID;
	  temperature_error = 0; /* No error, since no SPI communications */
	  pthread_mutex_unlock(&temp_value_mutex);
	}

      usleep(1000000);		/* Sleep 1 second, for sampling */
    }
}
      
	      
