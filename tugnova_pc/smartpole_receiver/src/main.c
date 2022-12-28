/**
 * @brief 車載器制御ソフト
 * @details ITS車載無線機とUDP/IPを変換する処理を行うソフト
 * @file main.c
 * @author 大藪
 * @date 2021/8/27
 */

#include "func.c"

/**
 * @brief メイン関数
 * @details UDPとSTS端末双方からの入力待ちを行い、受信すると
 *
 */
void main(int argc,char *argv[]){
     int uart_fd,udp_fd;
     int udp_sock;
     fd_set fds, readfds;
     int maxfd;
     int n;
     struct timeval tv;
     
     /* 初期化と初期接続完了 */
     init(&uart_fd,&udp_fd,&udp_sock);
   
     /*Select前処理*/
     /* fd_setの初期化します */
     FD_ZERO(&readfds);
     
     /* selectで待つ読み込みソケットとしてuart_fdを登録します */
     FD_SET(uart_fd, &readfds);
     /* selectで待つ読み込みソケットとしてudp_fd を登録します */
     FD_SET(udp_fd , &readfds);
     
     /* 10秒でタイムアウトするようにします */
     tv.tv_sec = 10;
     tv.tv_usec = 0;
     
     
     if (uart_fd > udp_fd) {
	  maxfd = uart_fd;
     } else {
	  maxfd = udp_fd;
     }
     
     while(1){
	  /*
	   *    読み込み用fd_setの初期化
	   *    selectが毎回内容を上書きしてしまうので、毎回初期化します
	   *         */
	  memcpy(&fds, &readfds, sizeof(fd_set));
	  tv.tv_sec = 10;
	  tv.tv_usec = 0;
	  
	  /* fdsに設定されたソケットが読み込み可能になるまで待ちます */
	  n = select(maxfd+1, &fds, NULL, NULL, &tv);
	  
	  /* タイムアウトの場合にselectは0を返します */
	  if (n == 0) {
	       printf("10sec timeout\n");
	  }
	  
	  /* STSに読み込み可能データがある場合 */
	  if (FD_ISSET(uart_fd, &fds)) {
#ifdef DEBUG
	       printf("hit! uart_fd\n");
#endif
	       radio_recv(uart_fd,udp_sock);
	  }
	  
	  /* Etherに読み込み可能データがある場合 */
	  if (FD_ISSET(udp_fd, &fds)) {
#ifdef DEBUG
	       printf("hit! udp_fd\n");
#endif
	       //printf("udp_recv\n");
	       udp_recv(uart_fd,udp_fd);
	  }
     }
}



