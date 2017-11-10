#include "lbnld.h"
#include "daemon_defs.h"
#include "../lbnl_prototypes.h"
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include "../lbnl_typedefs.h"
#include "../lbnl_params.h"

int got_sigterm=0;
int listenfd;
dref dfd;
unsigned int bsize;
u16 *imbuffer;
static image_num=0;
static u32 cfg_gain;

//Image size
static u32 img_size_x, img_size_y;

//Image acquisition and readout thread variables
static sem_t acq_state;
static pthread_t acq_thread, read_thread, fits_thread;
static enum ImgAcqStatus img_acq_state;
static pthread_mutex_t acq_state_mutex;

void *pt_take_picture(void *arg);
void *pt_read_picture(void *arg);
void *pt_take_fits(void *arg);

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

  pthread_mutex_init(&acq_state_mutex, NULL);
  pthread_mutex_lock(&acq_state_mutex);
  img_acq_state = Not_Started;
  pthread_mutex_unlock(&acq_state_mutex);
  
  /* Test mode */
  if ((argc>=2) && (strcmp(argv[1], "--test") == 0))
    {
      return lbnl_test_mode();
    }
  while (!got_sigterm) {
    clilen = addrlen;
    printf ("accepting conenctions\n");
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
    bsize = 0;
    imbuffer = NULL;
  }
  //	imbuffer = (unsigned short *) malloc (nbytes);
  imbuffer = (i16 *) malloc (nbytes);
  //	*imbuffer = 121;
  if (imbuffer == NULL)
    return (-ENOMEM);
  bsize = nbytes;
  return (0);
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

  //  dacresp_t dacresp;

  do {
    if ((cin=recv(fdin,&message,sizeof(cmdstruct_t), 0))!=sizeof(cmdstruct_t)){
      pdebug ("bad message, connection closed\n");
      printf("received socket data wrong size %d, wanted %d", cin, sizeof(cmdstruct_t));
      close (fdin);
      cin = 0;
    } else {
      printf ("cmd %d\n", message.cmd);
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
	printf ("cmd: GET_PROG\n");
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
		/* REMOVETHIS  DEBUGGING */
		printf("Special line is:\n %s\n",config_line);
		count_cfg_args = sscanf(config_line,"Enable: %u %x %x ",
					&enable_num, &enable_dacmask, &enable_clkmask);
		/* REMOVETHIS DEBUGGING */
		printf("Read %i arguments.\n", count_cfg_args);
		printf("Enable params %u 0X%X 0x%X\n",
		       enable_num, enable_dacmask, enable_clkmask); 

		/* Parse exposure time */
		count_cfg_args = sscanf(config_line, "ExpTime: %u ", &exp_time);
		/* REMOVETHIS DEBUGGING */
		/*if ((count_cfg_args == 1) || 1)
		  {
		    printf("Exposure time is %u.\n", exp_time);
		    }*/
	      }
	  }
	fclose(cfg_file);
	response.status = ret;
	send (fdin, (void *)&response, sizeof (respstruct_t),0);
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
	printf ("cmd: GET_ELAPSED_TIME\n");
	response.data[0] = exp_time_elapsed;
	sprintf(response.strmsg, "DONE");
	send (fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_GET_EXPTIME_PROG:
	printf ("cmd: GET_EXPTIME_PROG\n");
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
	if ((ret=lbnl_controller_get_temps (dfd, &faux1, &faux2))!=0){
	  printf ("ERROR %d\n",ret);
	  sprintf (response.strmsg, "ERROR %d\n",ret);
	} else {
	  sprintf (response.strmsg, "DONE");
	}
	response.status = ret;
	//			faux1 = 22.5;
	//			faux2 = 23.5;
	*((float *) &response.data[0]) =  faux1;
	*((float *) &response.data[1]) =  faux2;
	printf ("temp1 %f, temp2 %f\n", faux1, faux2);
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
	if ((ret=lbnl_ccd_clear (dfd))!=0){
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
	  if ((ret=lbnl_ccd_read (dfd, imbuffer))!=0){
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
	    return ret;
	  }

	clocks = (lbnl_clock_t *) malloc (ndacs *sizeof (lbnl_clock_t));
	clk_vals = (f32 *) malloc(ndacs *sizeof(f32));
	
	if ((ret=lbnl_controller_get_all_clocks (dfd, clocks, &ndacs))!=0)
	  {
	    printf ("ERROR %d\n",ret);
	    return ret;
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

	pthread_create(&acq_thread, NULL, &pt_take_picture, exp_args);
	sprintf( response.strmsg, "DONE");
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
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
	break;
      case LBNL_CMD_ENABLE:
	//FIXME TODO Add error checking and user-determined masking
	lbnl_controller_enable(dfd, DACMASK, CLKMASK);
	response.status=0;
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
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
	printf("img_acq_state is %i.\n", response.data[0]);
	send(fdin, (void *)&response, sizeof(respstruct_t),0);
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
  if ((ret=lbnl_ccd_read(dfd, imbuffer))!=0)
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
  //TODO FIXME Add clearing logic and update status variable
  //TODO Change behavior based on return from waiting
  //TODO Make abort flag thread safe?

  /* DEBUGGING REMOVETHIS */
  printf("Acquisition mode %i, clear ccd %i.\n",
  	 exp_args->acq_mode, exp_args->ccd_clear);

  if ((exp_args->acq_mode != 2) || ((exp_args->acq_mode == 2) && (exp_args->ccd_clear)))
    {
      printf("****************\nClearing CCD\n****************\n");
      pthread_mutex_lock(&acq_state_mutex);
      img_acq_state = Clearing;
      pthread_mutex_unlock(&acq_state_mutex);
      lbnl_ccd_clear(dfd);
    }

  pthread_mutex_lock(&acq_state_mutex);
  img_acq_state = Exposing;
  pthread_mutex_unlock(&acq_state_mutex);
  wait_status = wait_exposure_time();

  if (wait_status)
    {
      exp_abort = 0;		/* Clear abort flag so we do not trigger
				 on next run.  This also allows an abort on
				the other steps to trigger here.*/
      goto end_exp;		/* Exposure aborted */
    }

  pthread_mutex_lock(&acq_state_mutex);
  img_acq_state = Readout;
  pthread_mutex_unlock(&acq_state_mutex);
  lbnl_ccd_read(dfd, imbuffer);

  pthread_mutex_lock(&acq_state_mutex);
  img_acq_state = Done;
  pthread_mutex_unlock(&acq_state_mutex);

 end_exp:
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
