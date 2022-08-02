#ifndef   get_ip_address_string_h
#define   get_ip_address_string_h

#include <stdio.h>
#include <errno.h>
#include <string.h> /* for strncpy */
#include <unistd.h> /* for close */
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>


// IPv4のIPアドレスを取得
void get_ip_address_string(const char* if_name, char* result_ip_address) {
  int ret;
  int fd;
  struct ifreq ifr;

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  ifr.ifr_addr.sa_family = AF_INET;
  strncpy(ifr.ifr_name, if_name, IFNAMSIZ-1);
  ret = ioctl(fd, SIOCGIFADDR, &ifr);
  if(ret == 0) {
    strcpy(result_ip_address, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
  } else {
    if(errno == 19) {
      // fprintf(stdout, "DEBUG: get_ip_address_string: Failed to ioctl. [%s] No such device.\n", if_name);
    } else {
      // fprintf(stdout, "DEBUG: get_ip_address_string: Failed to ioctl. [%s] errno[%d]\n", if_name, errno);
    }
  }
  close(fd);
}

#endif // get_ip_address_string_h
