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
#include <signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "misc.h"
#include "multicast.h"
#include "rtp.h"
#include "status.h"

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
static int Quiet = 0;
const char *App_path;
int Verbose;
char const *Iface;

char const *Radio = NULL;
static int Input_fd = -1;
static struct pcmstream Pcmstream;
static struct sockaddr_in Control_address;
static volatile int cli_sock = -1;

static pthread_t command_thread;
uint16_t port = 1234;
static uint32_t Ssrc;
double freq = 0;
int samp_rate = 0;
int keep = 1;
int do_exit = 0;

struct command
{
        unsigned char cmd;
        unsigned int param;
}__attribute__((packed));

static void sighandler(int)
{
        fprintf(stderr, "Signal caught, exiting!\n");
	keep = 0;
	do_exit = 1;
}

int send_rtl_header(int s)
{
	struct { /* structure size must be multiple of 2 bytes */
		char magic[4];
		uint32_t tuner_type;
		uint32_t tuner_gain_count;
	} dongle_info;

	memcpy(&dongle_info.magic, "RTL0", 4);
	dongle_info.tuner_type = htons(0);
	dongle_info.tuner_gain_count = htons(0);

	int r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
	if (sizeof(dongle_info) != r) {
		printf("failed to send dongle information\n");
		return -1;
	}

	return r;
}

static int tcp_server()
{
	int sockfd;
	struct sockaddr_in servaddr;

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		perror("socket creation failed...\n");
		return -1;
	}
	bzero(&servaddr, sizeof(servaddr));

	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(port);

	if (bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr))) {
		perror("socket bind failed...\n");
		close(sockfd);

		return -1;
	}
	if (listen(sockfd, 5) != 0) {
		perror("Listen failed...\n");
		close(sockfd);

		return -1;
	}

	return sockfd;
}

int accept_cli(int srv_sock)
{
	int cli_sock;
	struct sockaddr_in cli;
	unsigned int len;

	len = sizeof(cli);
	cli_sock = accept(srv_sock, (struct sockaddr *)&cli, &len);
	if (cli_sock < 0) {
		perror("server accept failed...\n");
		return cli_sock;
	} else {
		printf("incoming connection from %s\n",
				inet_ntoa(cli.sin_addr));
		fflush(NULL);
	}

	send_rtl_header(cli_sock);

	return cli_sock;
}

static void close_stream(int sock)
{
	uint8_t cmd_buffer[PKTSIZE];
	uint8_t *bp = cmd_buffer;

	*bp++ = 1; // Generate command packet
	encode_int(&bp,COMMAND_TAG, arc4random());
	encode_int(&bp,OUTPUT_SSRC, Ssrc);
	encode_double(&bp, RADIO_FREQUENCY, 0);
	encode_eol(&bp);
	int cmd_len = bp - cmd_buffer;
	if (sendto(sock, cmd_buffer, cmd_len, 0, &Control_address,
				sizeof(Control_address)) != cmd_len)
		perror("command send");
}

static void *command_worker(void *)
{
	int Control_sock = -1;

        int left, received = 0;
        fd_set readfds;
        struct command cmd={0, 0};
        struct timeval tv= {1, 0};
        int r = 0;
        uint32_t tmp;
	uint8_t cmd_buffer[PKTSIZE];

	char iface[1024];
	resolve_mcast(Radio, &Control_address, DEFAULT_STAT_PORT,
			iface, sizeof(iface), 0);

	char const *ifc = (Iface != NULL) ? Iface : iface;
	Control_sock = output_mcast(&Control_address, ifc, 2, 0);
	if (Control_sock == -1) {
		fprintf(stdout,
			"Can't open cmd socket to radio control channel %s\n",
			strerror(errno));
		exit(EX_IOERR);
	}

        while(1) {
		if (cli_sock <= 0) {
			sleep(1);
			continue;
		}
                cmd.cmd = 0xff;
                left=sizeof(cmd);
                while (left > 0) {
                        FD_ZERO(&readfds);
                        FD_SET(cli_sock, &readfds);
                        tv.tv_sec = 1;
                        tv.tv_usec = 0;
                        r = select(cli_sock+1, &readfds, NULL, NULL, &tv);
                        if(r) {
                                received = recv(cli_sock,
						(char*)&cmd+(sizeof(cmd)-left),
						left, 0);
                                left -= received;
                        }
                        if (received == -1 || cli_sock <= 0 || do_exit) {
                                printf("comm recv bye\n");
				close_stream(Control_sock);

				if (!keep)
					return NULL;
				break;
                        }
                }
		uint32_t param = ntohl(cmd.param);
                switch(cmd.cmd) {
                case 0x01:
			if (param == 52000000 || param < 1000000) {
				printf("invalid freq %d\n", param);
				fflush(NULL);
				continue;
			}
                        printf("set freq %d\n", param);
			freq = param;
                        break;
                case 0x02:
			if (param > 5000000 || param < 10000) {
				printf("invalid sample rate %d\n", param);
				fflush(NULL);
				continue;
			}
                        printf("set sample rate %d\n", param);
			samp_rate = param;
                        break;
                case 0x03:
                        printf("set gain mode %d\n", ntohl(cmd.param));
//			encode_int(&bp, AGC_ENABLE, (cmd.param == 0));
                        break;
                case 0x04:
                        printf("set gain %d\n", param);
//			encode_float(&bp, GAIN, param);
//			encode_int(&bp, AGC_ENABLE, false);
                        break;
                case 0x05:
                        printf("set freq correction %d\n", ntohl(cmd.param));
                        continue;
                        break;
                case 0x06:
                        tmp = ntohl(cmd.param);
                        printf("set if stage %d gain %d\n", tmp >> 16, (short)(tmp & 0xffff));
                        continue;
                        break;
                case 0x08:
                        printf("set agc mode %d\n", ntohl(cmd.param));
                        continue;
                        break;
                case 0x0d:
                        printf("set tuner gain by index %d\n", ntohl(cmd.param));
//                        set_gain_by_index(dev, ntohl(cmd.param));
                        continue;
                        break;
                default:
                        continue;
                }
		fflush(NULL);
		if (freq != 0 && samp_rate != 0) {
			uint8_t *bp = cmd_buffer;
			*bp++ = 1; // Generate command packet
			encode_int(&bp, COMMAND_TAG, arc4random());
			encode_int(&bp, OUTPUT_SSRC, Ssrc);
			encode_string(&bp, PRESET, "rmd_iq", 6);
			encode_int(&bp, OUTPUT_ENCODING, U8);
			encode_int(&bp, AGC_ENABLE, 1);
			encode_double(&bp, RADIO_FREQUENCY, freq);
			encode_int(&bp, OUTPUT_SAMPRATE, samp_rate);
			encode_float(&bp, LOW_EDGE,  -(int)samp_rate/2);
			encode_float(&bp, HIGH_EDGE,  -(int)samp_rate/2);
			encode_eol(&bp);
			int cmd_len = bp - cmd_buffer;
			if (sendto(Control_sock, cmd_buffer, cmd_len, 0,
				&Control_address,
				sizeof Control_address) != cmd_len)
				perror("command send");
		}
        }
}

static int init(struct pcmstream *pc,struct rtp_header const *rtp,
		struct sockaddr const *sender)
{
	// First packet on stream, initialize
	pc->ssrc = rtp->ssrc;
	pc->type = rtp->type;
	pc->framesize = 0; // unknown

	memcpy(&pc->sender,sender,sizeof(pc->sender)); // Remember sender
	pc->source = formatsock(&pc->sender,false);

	return 0;
}

int data_worker(int Input_fd, int output)
{
	while (true) {
		struct sockaddr sender;
		socklen_t socksize = sizeof(sender);
		uint8_t buffer[PKTSIZE];

		// Gets all packets to multicast destination address,
		// regardless of sender IP, sender port, dest port, ssrc
		int size = recvfrom(Input_fd, buffer, sizeof(buffer),
				0, &sender, &socksize);
		if (do_exit) {
			close(output);
			break;
		}

		if (size == -1) {
			perror("recvmsg");
			fflush(NULL);
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
		if (size <= 0)
			continue;

		if (rtp.ssrc == 0 || (Ssrc != 0 && rtp.ssrc != Ssrc)) {
			printf("not for us: %d\n", rtp.ssrc);
			fflush(NULL);
			// Ignore unwanted or invalid SSRCs
			continue;
		}

		if (Pcmstream.ssrc == 0){
			// First packet on stream, initialize
			init(&Pcmstream,&rtp,&sender);

			if(!Quiet){
				fprintf(stderr,"New session from %u@%s, payload type %d\n",
						Pcmstream.ssrc,
						Pcmstream.source,
						rtp.type);
			}
		} else if(rtp.ssrc != Pcmstream.ssrc)
			continue; // unwanted SSRC, ignore

		if (!address_match(&sender,&Pcmstream.sender) ||
		    getportnumber(&Pcmstream.sender) != getportnumber(&sender)){
			// Source changed, the sender restarted
			init(&Pcmstream,&rtp,&sender);
			if(!Quiet){
				fprintf(stderr,"Session restart from %u@%s\n",
						Pcmstream.ssrc,
						Pcmstream.source);
			}
		}
		if (!rtp.marker) {
			// Change in sequence number from last RTP packet
			int seq_change = (int16_t)(rtp.seq - Pcmstream.last_header.seq);

			if (seq_change == 1) {
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
			} else if (seq_change > 1) {
				// Something got dropped. Emit some padding if it's not too much and we know the framesize
				// This will get invoked on the first packet, but nothing will happen because Pcmstream.framesize == 0
				int time_step = (int32_t)(rtp.timestamp - Pcmstream.last_header.timestamp) - Pcmstream.last_size;
				if (Pcmstream.framesize != 0)
					fprintf(stderr,"dropped packet, expected seq %d, got seq %d, lost %d frames\n",
							(int16_t)(Pcmstream.last_header.seq+1),rtp.seq,
							time_step);

				if (Pcmstream.framesize != 0 &&
				    time_step >= 0 && time_step < 48000) {
					// arbitrary, make this a parameter
					char zeroes[Pcmstream.framesize * time_step];
					memset(zeroes,0,sizeof(zeroes));
					fwrite(zeroes,1,sizeof(zeroes),stdout);
				}
			} else {
				// Else drop duplicate or old out of sequence - should buffer these under user control
				fprintf(stderr,"Discarding old packet, expected seq %d, got seq %d, timestamp %ul, size %d bytes, %d frames\n",
					(int16_t)(Pcmstream.last_header.seq+1),
					rtp.seq, rtp.timestamp,
					size,size*Pcmstream.last_size);
				goto done;
			}
		}
		int r = send(output, dp, size, 0);
		if (r < 0) {
			perror("write:");
			return r;
		}

done:
		Pcmstream.bytes_received += size;
		Pcmstream.last_header = rtp;
		Pcmstream.last_size = size;
	}

	return 0;
}

int main(int argc,char *argv[])
{
	pthread_attr_t attr;
	App_path = argv[0];
	setlocale(LC_ALL,getenv("LANG"));

	int c;
	while ((c = getopt(argc,argv,"qhi:p:r:s:V")) != EOF){
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
		case 'i':
			Iface = optarg;
			break;
		case 'p':
			port = strtol(optarg, NULL, 0);
			break;
		case 'r':
			Radio = optarg;
			break;
		case 's':
			Ssrc = strtol(optarg, NULL, 0);
			break;
		case 'h':
		default:
			fprintf(stderr,
				"Usage: %s [-h] [-v] [-q] [-s ssrc] mcast_address\n",
				argv[0]);
			fprintf(stderr,"       hex ssrc requires 0x prefix\n");
			exit(1);
		}
	}
	if (optind != argc-1) {
		fprintf(stderr,"mcast_address not specified\n");
		exit(1);
	}

	Mcast_address_text = argv[optind];

	int srv_sock = tcp_server();

	if (srv_sock < 0)
		return srv_sock;

	cli_sock = accept_cli(srv_sock);

	// Set up multicast input
	struct sockaddr_in saddr;
	Input_fd = setup_mcast_in(Mcast_address_text,
			(struct sockaddr *)&saddr, 0, 0);
	if (Input_fd == -1) {
		fprintf(stderr,"Can't set up input from %s\n",
				Mcast_address_text);
		close(srv_sock);
		close(cli_sock);

		return -1;
	}
	printf("multicast input: %s\n",
			inet_ntoa(saddr.sin_addr));
	fflush(NULL);

	int n = 1 << 20; // 1 MB
	if (setsockopt(Input_fd, SOL_SOCKET,SO_RCVBUF, &n,sizeof(n)) == -1)
		perror("setsockopt");

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&command_thread, NULL, command_worker, NULL);
	pthread_attr_destroy(&attr);

	struct sigaction sigact, sigign;

        sigact.sa_handler = sighandler;
        sigemptyset(&sigact.sa_mask);
        sigact.sa_flags = 0;
        sigign.sa_handler = SIG_IGN;
        sigaction(SIGINT, &sigact, NULL);
        sigaction(SIGTERM, &sigact, NULL);
        sigaction(SIGQUIT, &sigact, NULL);
        sigaction(SIGPIPE, &sigign, NULL);

	int rc = 0;

	while (1) {
		data_worker(Input_fd, cli_sock);
		cli_sock = -1;
		if (!keep)
			break;

		cli_sock = accept_cli(srv_sock);
		if (cli_sock < 0) {
			rc = cli_sock;
			break;
		}

	};

	close(srv_sock);
	close(Input_fd);

	return rc;
}

