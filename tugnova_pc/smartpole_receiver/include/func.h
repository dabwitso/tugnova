/**
 * @brief 車載器制御関数群
 * @details ITS車載無線機とUDP/IPを変換する処理を行うための制御関数群
 * @file func.h
 * @author 大藪
 * @date 2021/8/27
 */

#ifndef INCLUDE_MY_FUNC
#define INCLUDE_MY_FUNC

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/signal.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>

#define BAUDRATE B115200 //!< STS端末を接続しているシリアルの通信速度

#ifdef DUMMY_STS
#define MODEMDEVICE "/dev/com4" //!< STS端末を接続しているデバイス
#else
#define MODEMDEVICE "/dev/com4" //!< STS端末を接続しているデバイス
#endif

#define _POSIX_SOURCE 1 /* POSIX 準拠のソース */
#define FALSE 0 //!< FALSE定義
#define TRUE 1 //!< TRUE定義

#define UDP_RECV_PORT 30001 //!< UDP受信ポート
#define UDP_RECV_IP   "192.168.2.50" //!< 受信IPアドレス
#define UDP_SEND_PORT 30000 //!< UDP送信先ポート
#define UDP_SEND_IP   "192.168.2.100" //!< 送信先IPアドレス
#define BUF_SIZ  1024 //!< 受信バッファデータ長（UDPとUARTのデータ長）

void uart_open(int *ret);
void sts_connect(int uart_fd);
void udp_open(int *recv,int *send);
void init(int *uart_fd, int *udp_fd, int *udp_sock);
void radio_recv(int uart_fd,int udp_sock);
void udp_recv(int uart_fd,int udp_fd);


#endif
