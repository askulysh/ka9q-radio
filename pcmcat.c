// THIS PROGRAM IS OBSOLETE - Use 'pcmrecord --stdout' instead

// Receive and stream PCM RTP data to stdout
// Should emit .wav format by default to encode sample rate & parameters for subsequent encoding
// Revised Aug 2023 to more cleanly handle sender restarts
// Copyright 2023 Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <locale.h>
#include <errno.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netdb.h>
#include <time.h>
#include <sysexits.h>
#include <linux/filter.h>

#include "misc.h"
#include "multicast.h"
#include "rtp.h"

struct pcmstream {
  uint32_t ssrc;            // RTP Sending Source ID
  int type;                 // RTP type (10,11,20)
  
  struct sockaddr sender;
  char const *source;
  int framesize;            // Bytes per timestamp increment

  long long bytes_received;
  struct rtp_header last_header;
  int last_size;
};

// Command line params
static char const *Mcast_address_text;
static int Quiet;
const char *App_path;
int Verbose;
int Byteswap = -1; // automaticall determine from payload type

static int Input_fd = -1;
static struct pcmstream Pcmstream;
static uint32_t Ssrc; // Requested SSRC

static int init(struct pcmstream *pc,struct rtp_header const *rtp,struct sockaddr const *sender);

// Attach a classic BPF filter that passes only datagrams whose RTP SSRC field
// (bytes 8-11 of UDP payload) matches ssrc.
// SO_ATTACH_FILTER on SOCK_DGRAM presents UDP header + payload to BPF, so the
// RTP SSRC sits at offset 16: 8 (UDP header) + 8 (RTP fields before SSRC).
static void attach_ssrc_bpf_filter(int fd, uint32_t ssrc){
  struct sock_filter filter[] = {
    BPF_STMT(BPF_LD  | BPF_W   | BPF_ABS, 16),         /* load 4 bytes at udp[8]+rtp[8] = RTP SSRC */
    BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K,   ssrc, 0, 1), /* if match, fall through; else skip  */
    BPF_STMT(BPF_RET | BPF_K,             0xFFFF),      /* accept                             */
    BPF_STMT(BPF_RET | BPF_K,             0),           /* drop                               */
  };
  struct sock_fprog prog = {
    .len    = sizeof(filter) / sizeof(filter[0]),
    .filter = filter,
  };
  if(setsockopt(fd, SOL_SOCKET, SO_ATTACH_FILTER, &prog, sizeof(prog)) != 0)
    perror("SO_ATTACH_FILTER");
}

int main(int argc,char *argv[]){
  App_path = argv[0];
  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt(argc,argv,"qhs:b:V")) != EOF){
    switch(c){
    case 'V':
      VERSION();
      exit(EX_OK);
    case 'v':
      Verbose++;
      break;
    case 'q':
      Quiet++;
      break;
    case 's':
      Ssrc = strtol(optarg,NULL,0);
      break;
    case 'b':
      Byteswap = strtol(optarg,NULL,0);
      break;
    case 'h':
    default:
      fprintf(stderr,"Usage: %s [-h] [-v] [-q] -s ssrc [-b 0|1] mcast_address\n",argv[0]);
      fprintf(stderr,"       hex ssrc requires 0x prefix\n");
      exit(1);
    }
  }
  if(optind != argc-1){
    fprintf(stderr,"mcast_address not specified\n");
    exit(EX_USAGE);
  }
  if(Ssrc == 0){
    fprintf(stderr,"SSRC (-s) is required\n");
    exit(EX_USAGE);
  }
  Mcast_address_text = argv[optind];

  // Set up multicast input
  Input_fd = setup_mcast_in(Mcast_address_text,NULL,0,0);
  if(Input_fd == -1){
    fprintf(stderr,"Can't set up input from %s\n",
	    Mcast_address_text);
    exit(EX_USAGE);
  }
  attach_ssrc_bpf_filter(Input_fd, Ssrc);

  // audio input thread
  // Receive audio multicasts, multiplex into sessions, send to output
  while(true){
    struct sockaddr sender;
    socklen_t socksize = sizeof(sender);
    uint8_t buffer[PKTSIZE];

    // Gets all packets to multicast destination address, regardless of sender IP, sender port, dest port
    int size = recvfrom(Input_fd,buffer,sizeof(buffer),0,&sender,&socksize);
    if(size == -1){
      if(errno != EINTR){ // Happens routinely
	perror("recvmsg");
	usleep(1000);
      }
      continue;
    }
    if(size < RTP_MIN_SIZE)
      continue; // Too small to be valid RTP

    struct rtp_header rtp;
    uint8_t const *dp = ntoh_rtp(&rtp,buffer);

    size -= dp - buffer;
    if(rtp.pad){
      // Remove padding
      size -= dp[size-1];
      rtp.pad = 0;
    }
    if(size <= 0)
      continue;

    if(Pcmstream.ssrc == 0){
      // First packet on stream, initialize
      init(&Pcmstream,&rtp,&sender);
      
      if(!Quiet){
	fprintf(stderr,"New session from %u@%s, payload type %d\n",
		Pcmstream.ssrc,
		Pcmstream.source,
		rtp.type);
      }
    }

    if(!address_match(&sender,&Pcmstream.sender) || getportnumber(&Pcmstream.sender) != getportnumber(&sender)){
      // Source changed, the sender restarted
      init(&Pcmstream,&rtp,&sender);
      if(!Quiet){
	fprintf(stderr,"Session restart from %u@%s\n",
		Pcmstream.ssrc,
		Pcmstream.source);
      }
    }
    if(!rtp.marker){
      // Change in sequence number from last RTP packet
      int seq_change = (int16_t)(rtp.seq - Pcmstream.last_header.seq);

      if(seq_change == 1){
	// Normal case: next expected packet in sequence
	if(rtp.timestamp != Pcmstream.last_header.timestamp){
	  // There's no marker, this packet is in sequence after the last one, we now know bytes per timestamp count
	  int new_framesize = Pcmstream.last_size / (int32_t)(rtp.timestamp - Pcmstream.last_header.timestamp);
	  if(new_framesize != Pcmstream.framesize){
	    Pcmstream.framesize = new_framesize;
	    if(!Quiet){
	      fprintf(stderr,"%d bytes/Timestamp count\n",Pcmstream.framesize);
	    }
	  }
	}
      } else if(seq_change > 1){
	// Something got dropped. Emit some padding if it's not too much and we know the framesize
	// This will get invoked on the first packet, but nothing will happen because Pcmstream.framesize == 0
	int time_step = (int32_t)(rtp.timestamp - Pcmstream.last_header.timestamp) - Pcmstream.last_size;
	if(!Quiet && Pcmstream.framesize != 0)
	  fprintf(stderr,"dropped packet, expected seq %d, got seq %d, lost %d frames\n",
		  (int16_t)(Pcmstream.last_header.seq+1),rtp.seq,
		  time_step);

	if(Pcmstream.framesize != 0 && time_step >= 0 && time_step < 48000){  // arbitrary, make this a parameter
	  char zeroes[Pcmstream.framesize * time_step];
	  memset(zeroes,0,sizeof(zeroes));
	  fwrite(zeroes,1,sizeof(zeroes),stdout);
	}
      } else {
	// Else drop duplicate or old out of sequence - should buffer these under user control
	if(!Quiet)
	  fprintf(stderr,"Discarding old packet, expected seq %d, got seq %d, timestamp %ul, size %d bytes, %d frames\n",
		  (int16_t)(Pcmstream.last_header.seq+1), rtp.seq, rtp.timestamp, size,size*Pcmstream.last_size);
	goto done;
      }
    }
    if(Byteswap == true){
      // Byte swap incoming buffer
      int16_t *sdp = (int16_t *)dp;
      if(!Quiet){
	if(size & 1){
	  fprintf(stderr,"size %d not even!\n",size);
	}
      }
      int sampcount = size / 2;

      for(int i=0; i < sampcount; i++)
	sdp[i] = ntohs(sdp[i]);
      fwrite(sdp,sizeof(*sdp),sampcount,stdout);
    } else
      fwrite(dp,size,1,stdout);

    fflush(stdout);
  done:;
    Pcmstream.bytes_received += size;
    Pcmstream.last_header = rtp;
    Pcmstream.last_size = size;
  }
  exit(0); // Not reached
}
static int init(struct pcmstream *pc,struct rtp_header const *rtp,struct sockaddr const *sender){
  // First packet on stream, initialize
  pc->ssrc = rtp->ssrc;
  pc->type = rtp->type;
  pc->framesize = 0; // unknown
  if(Byteswap == -1){
    // Hardwired test for statically assigned big-endian 16-bit PCM payload types
    // The ideal way is to look at the data encoding in the status stream, which we'll eventually do
    if(pc->type == 10 || pc->type == 11)
      Byteswap = true;
    else
      Byteswap = false;
  }
  
  memcpy(&pc->sender,sender,sizeof(pc->sender)); // Remember sender
  pc->source = formatsock(&pc->sender,false);
  return 0;
}

