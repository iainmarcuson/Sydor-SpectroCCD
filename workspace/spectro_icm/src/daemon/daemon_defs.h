typedef
struct address_struct
{
        lbnladd_t     address;
        lbnladd_t     value;
} simadd_t;

#define MAXADDS		4096
/*---------------------------------------------------------------------------
| Definitions
|----------------------------------------------------------------------------*/

int lksfd;
pthread_mutex_t lbnllock          = PTHREAD_MUTEX_INITIALIZER;


/*-----------------------------------------------------------------------------
| Function Prototypes
|----------------------------------------------------------------------------*/

int   main(int argc, char **argv);
void *thread_main(void *arg);
void  signal_handler(int signo);
void  exit_cleanly(void);

#define DEFTIMPATH	"/tmp/timing.txt"
#define DEFFITSPATH	"/tmp/image.fits"
#define DEFCFGPATH	"/data/ccd.cfg"

#define FTIMPATH	"/data/timfilename.txt"
#define FCFGPATH	"/data/cfgfilename.txt"

/*-----------------------------------------------------------------------------
| Externals
|----------------------------------------------------------------------------*/
                                                               /* tcplinux.c */
extern int   tcp_listen(int port);
extern int   Tcp_listen(const char *host,const char *serv,socklen_t *addrlenp);
extern int   connect_to_server(char *host, int port);
extern int   Poll(int fd, int to);
extern void  ms_pause(int ms);
