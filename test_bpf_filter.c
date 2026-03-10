// Test for BPF SSRC socket filter used in pcmcat.
//
// Verifies that SO_ATTACH_FILTER with an SSRC-matching BPF program admits only
// UDP datagrams whose RTP SSRC field (bytes 8-11 of payload, at BPF offset 16:
// 8 bytes UDP header + 8 bytes RTP before SSRC) equals the filter value, and
// silently drops all others.
//
// Packets are built with hton_rtp() mirroring the real sender path in audio.c.
// Runs entirely on the loopback interface; no multicast or external tools needed.

#define _GNU_SOURCE 1
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdbool.h>
#include <linux/filter.h>
#include "rtp.h"

#define TEST_SSRC 0xDEADBEEFU

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
  if(setsockopt(fd, SOL_SOCKET, SO_ATTACH_FILTER, &prog, sizeof(prog)) != 0){
    perror("SO_ATTACH_FILTER");
    _exit(1);
  }
}

static void send_rtp_pkt(int fd, struct sockaddr_in *dst, uint32_t ssrc){
  struct rtp_header rtp;
  memset(&rtp, 0, sizeof(rtp));
  rtp.ssrc = ssrc;
  uint8_t packet[RTP_MIN_SIZE];
  hton_rtp(packet, &rtp);
  if(sendto(fd, packet, sizeof(packet), 0, (struct sockaddr *)dst, sizeof(*dst)) < 0)
    perror("sendto");
}

int main(void){
  // Receiver socket bound to loopback on an OS-assigned port
  int recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(recv_fd < 0){ perror("socket recv"); return 1; }

  struct sockaddr_in addr = {
    .sin_family      = AF_INET,
    .sin_port        = 0,
    .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
  };
  if(bind(recv_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0){
    perror("bind"); return 1;
  }
  socklen_t alen = sizeof(addr);
  getsockname(recv_fd, (struct sockaddr *)&addr, &alen);

  attach_ssrc_bpf_filter(recv_fd, TEST_SSRC);

  struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 }; // 200 ms timeout
  setsockopt(recv_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // Sender socket
  int send_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(send_fd < 0){ perror("socket send"); return 1; }

  // Send: wrong SSRC, correct SSRC, wrong SSRC
  send_rtp_pkt(send_fd, &addr, 0x11111111U);
  send_rtp_pkt(send_fd, &addr, TEST_SSRC);
  send_rtp_pkt(send_fd, &addr, 0x22222222U);

  // Drain receiver and check results
  int count = 0;
  int pass  = 1;
  uint8_t buf[64];

  for(;;){
    ssize_t n = recv(recv_fd, buf, sizeof(buf), 0);
    if(n < 0)
      break; // timeout or error — stop draining
    count++;
    if(n < RTP_MIN_SIZE){
      fprintf(stderr, "FAIL: received packet too short (%zd bytes)\n", n);
      pass = 0;
      continue;
    }
    struct rtp_header rtp;
    ntoh_rtp(&rtp, buf);
    if(rtp.ssrc != TEST_SSRC){
      fprintf(stderr, "FAIL: expected SSRC 0x%08x, got 0x%08x\n", TEST_SSRC, rtp.ssrc);
      pass = 0;
    }
  }

  if(count != 1){
    fprintf(stderr, "FAIL: expected 1 packet through filter, got %d\n", count);
    pass = 0;
  }

  close(recv_fd);
  close(send_fd);

  puts(pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
