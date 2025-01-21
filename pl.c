// ka9q-radio PL tone decoder
// Reads multicast PCM audio (mono only right now)
// Copyright Jan 2019 Phil Karn, KA9Q
#define _GNU_SOURCE 1
#include <assert.h>
#include <errno.h>
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <locale.h>
#include <signal.h>
#include <getopt.h>
#include <sysexits.h>

#include "filter.h"
#include "misc.h"
#include "multicast.h"
#include "rtp.h"
#include "osc.h"

// Global config variables
#define MAX_MCAST 20          // Maximum number of multicast addresses

static const float Kaiser_beta = 11;

static const int PL_blockrate = 5;    // PL Integration time 200 msec
//static const int PL_blockrate = 50;    // PL Integration time 20 msec
//static const int DTMF_blockrate = 20; // PL Integration time 50 ms
// Shift PL filter output down by PL_Shift to straddle DC and allow lower sample rate
static float const PL_Shift = 150;    // -83 to +104.1 Hz
static const float PL_samprate = 500; // Nyquist rate 250 Hz
static const float Filter_time = .200; // 200 ms
//static const float Filter_time = .0200; // 20 ms

// Command line params
const char *App_path;
int Verbose;                  // Verbosity flag
int Mcast_ttl = 10;           // our multicast output is frequently routed
static char *Mcast_address_text[MAX_MCAST];


// Group 1 is generated by 100 * (1.03515)^n, n=0...27
// 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3, 131.8, 136.5,
// 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8,
// 199.5, 206.5, 213.8, 221.3, 229.1, 237.1, 245.5, 254.1

// Group 2 - ?
// 159.8, 165.5, 171.3, 177.3, 183.5, 189.9, 196.6, 203.5, 210.7, 218.1,
// 225.7, 233.6, 241.8, 250.3/4

// Group 3
//      67.0,  69.3,  71.9,  74.4,  77.0,  79.7,  82.5,  85.4,  88.5,  91.5,
//     94.8,  97.4,

// Not in Icom 706MKIIG
// 150.0, 213.8, 221.3, 237.1, 245.5, 

// All the tones from various groups, including special NATO 150 Hz tone
static float PL_tones[] = {
     67.0,  69.3,  71.9,  74.4,  77.0,  79.7,  82.5,  85.4,  88.5,  91.5,
     94.8,  97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3,
    131.8, 136.5, 141.3, 146.2, 150.0, 151.4, 156.7, 159.8, 162.2, 165.5,
    167.9, 171.3, 173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6,
    199.5, 203.5, 206.5, 210.7, 213.8, 218.1, 221.3, 225.7, 229.1, 233.6,
    237.1, 241.8, 245.5, 250.3, 254.1
};

#define N_tones ((int)(sizeof(PL_tones)/sizeof(PL_tones[0])))

#if 0
static float DTMF_low_tones[] = { 697, 770, 852, 941 };
static float DTMF_high_tones[] = { 1209, 1336, 1477, 1633 };

static char DTMF_matrix[4][4] = {   // indexed by [low][high]
  { '1', '2', '3', 'A' },			  
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' },
};
#endif

// Global variables
static int Nfds;
static struct session *Sessions;

struct session {
  struct session *prev;       // Linked list pointers
  struct session *next; 
  int type;                 // input RTP type (10,11)
  
  struct sockaddr sender;
  char const *source;

  struct rtp_state rtp_state_in; // RTP input state

  int samprate;
  int pl_blocksize;
  int dtmf_blocksize;

  complex float pl_integrators[N_tones];
  struct osc pl_osc[N_tones];
  float strongest_tone_energy;
  int strongest_tone_index;

  float dtmf_tot_energy;
  complex float dtmf_low_integrators[4];
  complex float dtmf_high_integrators[4];  
  struct osc dtmf_low_osc[4];
  struct osc dtmf_high_osc[4];

  int pl_audio_count;          // Number of samples integrated so far
  int dtmf_audio_count;        // Number of samples integrated so far

  char current_dtmf_digit;
  float current_pl_tone;
  struct filter_in filter_in;
  int in_cnt;
  struct filter_out pl_filter_out;
};

static void closedown(int);
static struct session *lookup_session(const struct sockaddr *,uint32_t);
static struct session *create_session(struct sockaddr const *r,uint32_t,uint16_t,uint32_t);
static int close_session(struct session *);
static float process_pl(struct session *sp,complex float samp);
#if 0
static char process_dtmf(struct session *sp,complex float samp);
#endif

static struct option Options[] =
  {
   {"iface", required_argument, NULL, 'A'},
   {"pcm-in", required_argument, NULL, 'I'},
   {"ttl", required_argument, NULL, 'T'},
   {"verbose", no_argument, NULL, 'v'},
   {"Version", no_argument, NULL, 'V'},
   {NULL, 0, NULL, 0},
};

static char Optstring[] = "A:I:T:vV";

int main(int argc,char * const argv[]){
  App_path = argv[0];

  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != -1){
    switch(c){
    case 'A':
      Default_mcast_iface = optarg;
      break;
    case 'I':
      if(Nfds == MAX_MCAST){
	fprintf(stdout,"Too many multicast addresses; max %d\n",MAX_MCAST);
      } else 
	Mcast_address_text[Nfds++] = optarg;
      break;
    case 'T':
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    case 'v':
      Verbose++;
      break;
    case 'V':
      VERSION();
      exit(EX_OK);
    default:
      break;
    }
  }
  setlinebuf(stdout); // see results quickly when grepping

  // Also accept groups without -I option
  for(int i=optind; i < argc; i++){
    if(Nfds == MAX_MCAST){
      fprintf(stdout,"Too many multicast addresses; max %d\n",MAX_MCAST);
    } else 
      Mcast_address_text[Nfds++] = argv[i];
  }
  // Set up multicast
  if(Nfds == 0){
    fprintf(stdout,"Must specify PCM source group(s)\n");
    exit(1);
  }

  // Set up multicast input, create mask for select()
  fd_set fdset_template; // Mask for select()
  FD_ZERO(&fdset_template);
  int max_fd = 2;        // Highest number fd for select()
  int input_fd[Nfds];    // Multicast receive sockets

  for(int i=0;i<Nfds;i++){
    input_fd[i] = setup_mcast_in(Mcast_address_text[i],NULL,0,0);
    if(input_fd[i] == -1){
      fprintf(stdout,"Can't set up input %s\n",Mcast_address_text[i]);
      continue;
    }
    if(input_fd[i] > max_fd)
      max_fd = input_fd[i];
    FD_SET(input_fd[i],&fdset_template);
  }

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  while(true){
    // Wait for traffic to arrive
    fd_set fdset = fdset_template;
    int const s = select(max_fd+1,&fdset,NULL,NULL,NULL);
    if(s < 0 && errno != EAGAIN && errno != EINTR) break;
    if(s == 0) continue; // Nothing arrived; probably just an ignored signal

    for(int fd_index = 0;fd_index < Nfds;fd_index++){
      if(input_fd[fd_index] == -1 || !FD_ISSET(input_fd[fd_index],&fdset)) continue;

      // Receive PCM in RTP/UDP/IP
      struct sockaddr sender;
      uint8_t buffer[PKTSIZE];
      socklen_t socksize = sizeof(sender);
      int size = recvfrom(input_fd[fd_index],buffer,sizeof(buffer),0,&sender,&socksize);
      if(size == -1){
	if(errno != EINTR){ // Happens routinely
	  perror("recvfrom");
	  usleep(1000);
	}
	continue;
      }
      if(size <= RTP_MIN_SIZE){
	usleep(500); // Avoid tight loop
	continue; // Too small to be valid RTP
      }
      // RTP header to host format
      struct rtp_header rtp_hdr;
      uint8_t const *dp = ntoh_rtp(&rtp_hdr,buffer);
      size -= (dp - buffer);
      if(rtp_hdr.pad){
	// Remove padding
	size -= dp[size-1];
	rtp_hdr.pad = 0;
      }
      if(size <= 0) continue; // Bogus RTP header?
      
      // Detect and handle stereo?
      int const samprate = samprate_from_pt(rtp_hdr.type);
      if(samprate == 0)	continue;
      
      struct session *sp = lookup_session(&sender,rtp_hdr.ssrc);
      if(sp == NULL){
	sp = create_session(&sender,rtp_hdr.ssrc,rtp_hdr.seq,rtp_hdr.timestamp);
	if(sp == NULL){
	  fprintf(stdout,"No room!!\n");
	  continue;
	}
	fprintf(stdout,"new ssrc %u, samprate %'d Hz\n",rtp_hdr.ssrc,samprate);
	sp->type = rtp_hdr.type;
	sp->samprate = samprate;

	// Set up input side of audio baseband filter
	// 4800 samples @ 24 kHz = 200 ms
	int const Filter_block = roundf(Filter_time * sp->samprate);
	create_filter_input(&sp->filter_in,Filter_block,Filter_block+1,REAL);

	// Set up PL tone detector
	sp->pl_blocksize = PL_samprate / PL_blockrate;
	// Set up PL tone steps and phasors
	for(int n=0; n < N_tones; n++){
	  sp->pl_integrators[n] = 0;
	  set_osc(&sp->pl_osc[n],(PL_tones[n] - PL_Shift)/PL_samprate,0);
	}

	//  200 ms @ 1500 Hz = 300 samples x 2 = 600 point FFT, 2.5 Hz bins, rotate by 10 hz increments
	int pl_Filter_block = roundf(PL_samprate * Filter_time);
	create_filter_output(&sp->pl_filter_out,&sp->filter_in,NULL,pl_Filter_block,COMPLEX);
	// Pass 50-300 Hz
	// Kaiser beta = 11; kaiser alpha = 11/pi = 3.5; first null @ sqrt(1+alpha^2) = 3.64 bins * 5 Hz = 18.2 Hz
	set_filter(&sp->pl_filter_out,(50. - PL_Shift)/PL_samprate,(300. - PL_Shift)/PL_samprate,Kaiser_beta);
      }
      int sampcount = size / sizeof(int16_t);
      int const samples_skipped = rtp_process(&sp->rtp_state_in,&rtp_hdr,sampcount);
      if(samples_skipped < 0) continue;

      
      int16_t const *sampp = (int16_t *)dp;
      while(sampcount-- > 0){
	// For each sample, run the local oscillators and integrators
	float const samp = SCALE16 * (int16_t)ntohs(*sampp++);
	if(put_rfilter(&sp->filter_in,samp) == 0)
	  continue;

	int const Rotate = 2 * (PL_Shift * Filter_time);
	execute_filter_output(&sp->pl_filter_out,Rotate);
	// Process for PL tone
	for(int n=0; n < sp->pl_filter_out.olen; n++){
	  float const pl_tone = process_pl(sp,sp->pl_filter_out.output.c[n]);
	  if(pl_tone > 0){
#if 0
	    printf("ssrc %u: PL %.1f Hz\n",sp->rtp_state_in.ssrc,pl_tone);
#endif
	    sp->current_pl_tone = pl_tone;
	  }
	}
      }
#if 0
	
      char const dtmf_digit = process_dtmf(sp,samp);
      if(dtmf_digit == -1)
	continue;
      if(dtmf_digit != sp->current_dtmf_digit){
#if 0
	printf("ssrc %u: DTMF %c\n",sp->rtp_state_in.ssrc,dtmf_digit);
#endif
	sp->current_dtmf_digit = dtmf_digit;
      }
#endif
    }
  }
}

static struct session *lookup_session(const struct sockaddr *sender,const uint32_t ssrc){
  struct session *sp;
  for(sp = Sessions; sp != NULL; sp = sp->next){
    if(sp->rtp_state_in.ssrc == ssrc && address_match(&sp->sender,sender)){
      // Found it
      if(sp->prev != NULL){
	// Not at top of bucket chain; move it there
	if(sp->next != NULL)
	  sp->next->prev = sp->prev;

	sp->prev->next = sp->next;
	sp->prev = NULL;
	sp->next = Sessions;
	Sessions = sp;
      }
      return sp;
    }
  }
  return NULL;
}
// Create a new session, partly initialize
static struct session *create_session(struct sockaddr const *sender,uint32_t ssrc,uint16_t seq,uint32_t timestamp){
  struct session *sp;

  if((sp = calloc(1,sizeof(*sp))) == NULL)
    return NULL; // Shouldn't happen on modern machines!
  
  // Initialize entry
  sp->source = formatsock(sender,false);
  memcpy(&sp->sender,sender,sizeof(struct sockaddr));
  sp->rtp_state_in.ssrc = ssrc;
  sp->rtp_state_in.seq = seq;
  sp->rtp_state_in.timestamp = timestamp;

  // Put at head of bucket chain
  sp->next = Sessions;
  if(sp->next != NULL)
    sp->next->prev = sp;
  Sessions = sp;
  return sp;
}

static int close_session(struct session *sp){
  if(sp == NULL)
    return -1;
  
  // Remove from linked list
  if(sp->next != NULL)
    sp->next->prev = sp->prev;
  if(sp->prev != NULL)
    sp->prev->next = sp->next;
  else
    Sessions = sp->next;
  FREE(sp);
  return 0;
}
static void closedown(int s){
  (void)s; // unused
  while(Sessions != NULL)
    close_session(Sessions);

  exit(0);
}

// Look for PL tone after each integration interval
static float process_pl(struct session * const sp,complex float const samp){

  for(int n=0; n < N_tones; n++)
    sp->pl_integrators[n] += conjf(samp) * step_osc(&sp->pl_osc[n]);

  if(++sp->pl_audio_count < sp->pl_blocksize)
    return -1; // Not done integrating

  sp->pl_audio_count = 0;
  // NBFM nominal bandwidth is 16 kHz, so a (slow) deviation of +/- 8 kHz will give 0 dB audio
  // PL deviation is nominally > 600 Hz or -22.5 dB 
  // Should calculate this analytically from specified minimum tone deviation (500 Hz?) and audio path gain
  sp->strongest_tone_energy = 0.005 * sp->pl_blocksize; // mininum tone energy in block
  sp->strongest_tone_index = -1;
  for(int n=0; n < N_tones; n++){
    float const energy = cnrmf(sp->pl_integrators[n]);
    if(energy > sp->strongest_tone_energy){
      sp->strongest_tone_energy = energy;
      sp->strongest_tone_index = n;
    }
    sp->pl_integrators[n] = 0;
  }
  if(sp->strongest_tone_index == -1)
    return 0; // No tone found
  float const pl_tone = PL_tones[sp->strongest_tone_index];
  printf("ssrc %u: tone %.1f Hz %.1f dB\n",sp->rtp_state_in.ssrc,pl_tone,power2dB(sp->strongest_tone_energy/sp->pl_blocksize));
  return pl_tone;
}

#if 0
// Look for DTMF digit after each integration interval
static char process_dtmf(struct session *sp,complex float samp){
  sp->dtmf_tot_energy += samp * samp;
  for(int n=0; n < 4; n++){
    sp->dtmf_low_integrators[n] += conjf(samp) * step_osc(&sp->dtmf_low_osc[n]);
    sp->dtmf_high_integrators[n] += conjf(samp) * step_osc(&sp->dtmf_high_osc[n]);
  }
  if(++sp->dtmf_audio_count < sp->dtmf_blocksize)
    return -1;

  sp->dtmf_audio_count = 0;
  const float min_tone_level = 0.1 * sp->dtmf_blocksize; // Each tone must be above -10 dBFS
  
  int low_tone_index = -1;
  float low_tone_snr = 0;
  float low_tone_energy = 0; // Set this to a minimum threshold
  {
    float total_energy = 0;
    for(int n=0; n < 4; n++){
      float const energy = cnrmf(sp->dtmf_low_integrators[n]);
      sp->dtmf_low_integrators[n] = 0;
      total_energy += energy;
      if(energy >= low_tone_energy){
	low_tone_energy = energy;
	low_tone_index = n;
      }
    }
    low_tone_snr = low_tone_energy / (total_energy - low_tone_energy);
    if(low_tone_energy < min_tone_level || low_tone_snr < 10) // 10 dB
      low_tone_index = -1; // Not good enough
  }
  int high_tone_index = -1;
  float high_tone_snr = 0;
  float high_tone_energy = 0; // Set this to a minimum threshold
  {
    float total_energy = 0;
    for(int n=0; n < 4; n++){
      float const energy = cnrmf(sp->dtmf_high_integrators[n]);
      sp->dtmf_high_integrators[n] = 0;
      total_energy += energy;
      if(energy >= high_tone_energy){
	high_tone_energy = energy;
	high_tone_index = n;
      }
    }
    high_tone_snr = high_tone_energy / (total_energy - high_tone_energy);
    if(high_tone_energy < min_tone_level || high_tone_snr < 10) // 10 dB
      high_tone_index = -1;
  }
  char result = 0;
  if(low_tone_index != -1 && high_tone_index != -1)
    result = DTMF_matrix[low_tone_index][high_tone_index];

#if 1
  if(result != sp->current_dtmf_digit){
    low_tone_energy /= sp->dtmf_blocksize; // scale to per sample
    high_tone_energy /= sp->dtmf_blocksize;
    printf("DTMF debug ssrc %u %c low=(%.0f, abs %.1f dB snr %.1f) high=(%.0f, abs %.1f dB, snr %.1f)\n",
	   sp->rtp_state_in.ssrc, result,
	   DTMF_low_tones[low_tone_index],power2dB(low_tone_energy),power2dB(low_tone_snr),
	   DTMF_high_tones[high_tone_index],power2dB(high_tone_energy),power2dB(high_tone_snr));
#endif
  sp->dtmf_tot_energy = 0;
  return result;
}
#endif
