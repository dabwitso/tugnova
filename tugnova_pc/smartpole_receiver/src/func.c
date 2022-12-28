/**
 * @brief 車載器制御関数群
 * @details ITS車載無線機とUDP/IPを変換する処理を行うための制御関数群
 * @file func.c
 * @author 大藪
 * @date 2021/8/27
 */



#include "func.h"

/**
 * @brief STS端末と接続するためのシリアル(UART)デバイスを開く関数
 *
 * @param[out] ret 開いたシリアルデバイスのファイルディスクリプタ変数
 */
void uart_open(int *ret){
     int fd, c, res, i;             /* fd:ファイルディスクリプタ res:受け取った文字数 */
     struct termios oldtio, newtio; /* 通信ポートを制御するためのインターフェイス */

     if((fd=open(MODEMDEVICE, O_RDWR | O_NOCTTY))== -1){
	  /* O_RDWR:読み書き両用 O_NOCTTY:tty制御をしない */	  
	  perror(MODEMDEVICE);
	  exit(-1);
     }
     printf("uart fd=%08x\n",fd);
     
     tcgetattr(fd, &oldtio);          /* 現在のシリアルポートの設定を退避させる */
     bzero(&newtio, sizeof(newtio));  /* 新しいポートの設定の構造体をクリア */
     
     newtio.c_cflag= (BAUDRATE | CS8 | CLOCAL | CREAD);
     /* CRTSCTS:フロー制御有り CS8:8ビット、ノンパリティ、ストップビット１
      *       CLOCAL:モデムの状態信号を無視 CREAD:受信可能にする */
     
     newtio.c_iflag=IGNPAR;          /* IGNPAR:パリティエラーの文字は無視 */
     
     newtio.c_oflag=0;               /* rawモード */
     newtio.c_lflag=0;               /* 非カノニカル入力 */
     
     newtio.c_cc[VTIME]=0;           /* キャラクタ間タイマは未使用 */
     newtio.c_cc[VMIN]=4;            /* MC文字受け取るまでブロックする */
     
     tcflush(fd,TCIFLUSH);           /* ポートのクリア */
     tcsetattr(fd, TCSANOW, &newtio); /* ポートの設定を有効にする */

     *ret=fd;
}



/**
 * @brief STS端末の起動処理
 * 
 * @details STS端末が起動するための起動要求を送信し、起動要求OKを待つ関数。失敗するとアプリ停止。
 *
 * @param[in] uart_fd STS端末のシリアルデバイスのファイルディスクリプタ変数
 */
void sts_connect(int uart_fd){
     unsigned short port_id=8500;
     unsigned short data_size=12;
     unsigned char  info_id=0x11;
     unsigned char  command=1;
     unsigned short send_msg_port_id_v2v=8501;
     unsigned short recv_msg_port_id_v2v=8511;
     unsigned short recv_msg_port_id_v2i=8512;
     unsigned short reserved_6 = 8513;
     unsigned char  reserved_7 = 0;
     unsigned char  reserved_8 = 0;
     
     unsigned char msg1[16];
     
     unsigned char  recv_buf[BUF_SIZ];
     unsigned short recv_port_id;
     unsigned short recv_data_size;
     unsigned char  recv_info_id;
     unsigned char  recv_command;
     unsigned char  recv_ret;
     
     int n,m;
     int len;
     
     msg1[ 0]=((port_id>>8)&0xff);
     msg1[ 1]=((port_id>>0)&0xff);
     msg1[ 2]=((data_size>>8)&0xff);
     msg1[ 3]=((data_size>>0)&0xff);
     msg1[ 4]=info_id;
     msg1[ 5]=command;
     msg1[ 6]=((send_msg_port_id_v2v>>8)&0xff);
     msg1[ 7]=((send_msg_port_id_v2v>>0)&0xff);
     msg1[ 8]=((recv_msg_port_id_v2v>>8)&0xff);
     msg1[ 9]=((recv_msg_port_id_v2v>>0)&0xff);
     msg1[10]=((recv_msg_port_id_v2i>>8)&0xff);
     msg1[11]=((recv_msg_port_id_v2i>>0)&0xff);
     msg1[12]=((reserved_6>>8)&0xff);
     msg1[13]=((reserved_6>>0)&0xff);
     msg1[14]=reserved_7;
     msg1[15]=reserved_8;
     
     /*STS起動メッセージ送信*/
     write(uart_fd,msg1,16);
     
     for(n=0;n<128;n++){
	  memset(recv_buf,0,BUF_SIZ);
	  len=read(uart_fd,recv_buf,4);

#ifdef DEGUB
	  printf("len = %d\n",len);
	  for(m=0;m<32;m++)printf("%02x ",recv_buf[m]);
	  printf("\n");
#endif
	  recv_port_id  =recv_buf[0]*256+recv_buf[1];
	  recv_data_size=recv_buf[2]*256+recv_buf[3];

	  printf("try %d : portID=%d   size=%d\n",n,recv_port_id,recv_data_size);

	  if(recv_data_size >= 1024){
	       printf("受信メッセージ長異常 %d -> ",recv_data_size);
	       recv_data_size=(recv_data_size&0x03ff);
	       printf("%d\n",recv_data_size);
	  }
	  //memset(recv_buf,0,BUF_SIZ);
	  len=read(uart_fd,recv_buf,recv_data_size);
	
#ifdef DEGUB
	  printf("len = %d\n",len);
	  for(m=0;m<32;m++)printf("%02x ",recv_buf[m]);
	  printf("\n");
#endif
	
	  if(recv_port_id==8500 && recv_data_size == 3){
	       recv_info_id=recv_buf[0];
	       recv_command=recv_buf[1];
	       recv_ret    =recv_buf[2];
	       if(recv_info_id == 0x11 && recv_command==0x81 && recv_ret == 0){
		    printf("起動完了\n");
		    return;
	       }else{
		    printf("起動異常 %02x,%02x,%02x\n",recv_info_id,recv_command,recv_ret);
		    sleep(1);
		    write(uart_fd,msg1,16);
	       }
	  }else{
	       printf("別メッセージ portID=%d  len=%d\n",recv_port_id,recv_data_size);
	  }
     }
     printf("起動できず\n");
     exit(1);
}

struct sockaddr_in addr2; //!< 送信先アドレス変数

/**
 * @brief UDP/IP通信用ソケットオープン関数
 * 
 * @details UDP/IP用のソケットを作る関数。IPやポートはヘッダーファイルで設定。
 *
 * @param[out] recv UDP/IP受信用ファイルディスクリプタ格納変数
 * @param[out] send UDP/IP送信用ファイルディスクリプタ格納変数
 */
void udp_open(int *recv,int *send){
     int sock1,sock2;
     struct sockaddr_in addr1;//,addr2;
     
     /* 受信ソケットを1つ作ります */
     sock1 = socket(AF_INET, SOCK_DGRAM, 0);
     
     addr1.sin_family = AF_INET;     
     addr1.sin_addr.s_addr = inet_addr(UDP_RECV_IP);
     
     /* ポート番号を設定します */
     addr1.sin_port = htons(UDP_RECV_PORT);
     
     /* bindします */
     bind(sock1, (struct sockaddr *)&addr1, sizeof(addr1));     
     *recv=sock1;

     printf("udp recv fd=%08x\n",sock1);

     
     /*送信ソケット*/
     sock2 = socket(AF_INET, SOCK_DGRAM, 0);
     
     addr2.sin_family = AF_INET;
     addr2.sin_port = htons(UDP_SEND_PORT);
     addr2.sin_addr.s_addr = inet_addr(UDP_SEND_IP);
     
     *send=sock2;     
     printf("udp send fd=%08x\n",sock2);
}



FILE *rlog; //!<UART受信ログファイル格納変数
FILE *slog; //!<UART送信ログファイル格納変数
/* シグナルハンドラ *
void sig_handler(int signo){
     switch(signo){
     case SIGINT:
     case SIGKILL:
	  fclose(rlog);
	  fclose(slog);
	  exit(0);
     }
}*/


/**
 * @brief 初期化関数
 * 
 * @details UARTとUDP/IPのデバイスオープンと、ログファイルオープンを実施
 *
 * @param[out] uart_fd STS端末のシリアルデバイスのファイルディスクリプタ変数
 * @param[out] udp_fd UDP/IP受信用ファイルディスクリプタ格納変数
 * @param[out] udp_sock UDP/IP送信用ファイルディスクリプタ格納変数
 */
void init(int *uart_fd, int *udp_fd, int *udp_sock){
     
     char rlogname[128];
     char slogname[128];
     
     time_t now = time(NULL);
     struct tm *pnow = localtime(&now); //現在時刻を取得
     
     /*uart open*/
     uart_open(uart_fd);
     
     /*udp open*/
     udp_open(udp_fd,udp_sock);

//#ifndef DUMMY_STS
     /*STS connct open*/
     sts_connect(*uart_fd);
//#endif
     
     sprintf(rlogname,"log/rlog_%04d%02d%02d-%02d%02d%02d.csv",
	    (pnow->tm_year+1900),(pnow->tm_mon+1),
	    pnow->tm_mday,pnow->tm_hour,pnow->tm_min,pnow->tm_sec);
     sprintf(slogname,"log/slog_%04d%02d%02d-%02d%02d%02d.csv",
	    (pnow->tm_year+1900),(pnow->tm_mon+1),
	    pnow->tm_mday,pnow->tm_hour,pnow->tm_min,pnow->tm_sec);
     
     rlog=fopen(rlogname,"w");
     if(rlog == NULL){
	  printf("ファイルオープンエラー %s\n",rlogname);
	  exit(errno);
     }
     slog=fopen(slogname,"w");     
     if(slog == NULL){
	  printf("ファイルオープンエラー %s\n",slogname);
	  fclose(rlog);
	  exit(errno);
     }
     
     /* シグナルハンドラの設定 *
     if (signal(SIGINT, sig_handler) == SIG_ERR) {
	  printf("\ncan't catch SIGINT\n");
     }
     if (signal(SIGKILL, sig_handler) == SIG_ERR) {
	  printf("\ncan't catch SIGKILL\n");
     }*/
}


/**
 * @brief ITS無線を受信し、UDP/IPを送信する関数
 *
 * @param[in] uart_fd 受信元STS端末のシリアルデバイスファイルディスクリプタ
 * @param[in] udp_sock 送信先UDPソケット
 */
void radio_recv(int uart_fd,int udp_sock){
     unsigned char  recv_buf[BUF_SIZ];
     
     unsigned short port_id;
     unsigned short data_size;
     
     double rssi_rf1,rssi_rf2;
     unsigned int app_data_len;
     int len;
     int n;
     
     
     struct timespec ts;
     struct tm pnow;
     
     int ret = clock_gettime(CLOCK_REALTIME, &ts);     
     localtime_r(&ts.tv_sec, &pnow); //現在日時
     
     len=read(uart_fd,recv_buf,4);
     port_id  =recv_buf[0]*256+recv_buf[1];
     data_size=recv_buf[2]*256+recv_buf[3];
     fprintf(rlog,"%04d/%02d/%02d %02d:%02d:%02d.%09ld,",
	     (pnow.tm_year+1900),(pnow.tm_mon+1),
	     pnow.tm_mday,pnow.tm_hour,pnow.tm_min,pnow.tm_sec,ts.tv_nsec);
     for(n=0;n<4;n++){fprintf(rlog,"%02x",recv_buf[n]);}
     
     if(data_size >= 1024){
	  printf("受信メッセージ長異常 %d -> ",data_size);
	  data_size=(data_size & 0x03ff);
	  printf("%d\n",data_size);
     }
     len=read(uart_fd,recv_buf,data_size);
     for(n=0;n<len;n++){fprintf(rlog,"%02x",recv_buf[n]);}
     fprintf(rlog,",\n");
     fflush(rlog);
#ifdef DEBUG
     printf("read uart %d\n",len);
#endif
     if( port_id == 8511 ){
	  rssi_rf1=((recv_buf[0]&0x01)*(-256)+recv_buf[1])/2;
	  rssi_rf2=((recv_buf[2]&0x01)*(-256)+recv_buf[3])/2;
	  app_data_len = (recv_buf[22]*256+recv_buf[23]);
	  if( sendto(udp_sock,&recv_buf[24],app_data_len,0,(struct sockaddr *)&addr2, sizeof(addr2)) == -1){
	       printf("UDP送信エラー\n");
	  }
     }else if( port_id == 8512 ){
	  rssi_rf1=((recv_buf[0]&0x01)*(-256)+recv_buf[1])/2;
	  rssi_rf2=((recv_buf[2]&0x01)*(-256)+recv_buf[3])/2;
	  app_data_len = (recv_buf[26]*256+recv_buf[27]);
	  if( sendto(udp_sock,&recv_buf[28],app_data_len,0,(struct sockaddr *)&addr2, sizeof(addr2)) == -1){
	       printf("UDP送信エラー\n");
	  }
#ifdef DUMMY_STS
     }else if( port_id < 0x4000){
	  if( sendto(udp_sock,&recv_buf[0],32,0,(struct sockaddr *)&addr2, sizeof(addr2)) == -1){
	       printf("UDP送信エラー\n");
	  }
#endif	  
     }else{
	  printf("不明なメッセージ : %d \n",port_id);
     }     
}


/**
 * @brief UDP/IPからデータ受信し、ITS無線へ送信する関数
 *
 * @param[in] udp_fd 受信元UDPソケット
 * @param[in] uart_fd 送信先STS端末のシリアルデバイスファイルディスクリプタ
 */
void udp_recv(int uart_fd,int udp_fd){
     unsigned char  recv_buf[BUF_SIZ];
     unsigned char  send_buf[BUF_SIZ];
     int recv_len;
     struct sockaddr address; 
     int address_length;
     
     unsigned short port_id=8501;
     unsigned short data_size;
     unsigned short msg_data_size;
     
     struct timespec ts;
     struct tm pnow;
     
     int ret;
     int n;
     
     /*UDP受信処理*/
     //memset(recv_buf,0,BUF_SIZ);
     //recv_len=recvfrom(udp_fd,recv_buf,BUF_SIZ,0,&address,&address_length);
#ifdef DEBUG
     printf("recv");
#endif
     recv_len=recv(udp_fd,recv_buf,BUF_SIZ,0);
     
#ifdef DEBUG
     printf(" len=%d\n",recv_len);
#endif
     if(recv_len > 0 && recv_len < 100){
	  data_size=recv_len+18;
	  msg_data_size=data_size-4;
#ifdef DEBUG
	  printf("data_size=%d msg_data_size=%d\n",data_size,msg_data_size);
#endif
	  
	  /*UART送信処理*/
	  send_buf[ 0]=(port_id>>8)  &0xff;
	  send_buf[ 1]=(port_id>>0)  &0xff;
	  send_buf[ 2]=(msg_data_size>>8)&0xff;
	  send_buf[ 3]=(msg_data_size>>0)&0xff;
	  send_buf[ 4]=(0<<2)+0; //なし:(0<<2)+0  あり(2<<2)+3
	  send_buf[ 5]=0; //sec info 2
	  send_buf[ 6]=0; //sec info 3
	  send_buf[ 7]=0; //sec info 3
	  send_buf[ 8]=0; //sec info 3
	  send_buf[ 9]=0; //sec info 3
	  send_buf[10]=0; //sec info 3
	  send_buf[11]=0; //sec info 3
	  send_buf[12]=0; //sec info 3
	  send_buf[13]=0; //sec info 3
	  send_buf[14]=0xff; //sec info 3
//	  send_buf[15]=0x70; //app info 
	  send_buf[15]=0xb0; //app info 
	  send_buf[16]=(recv_len>>8)&0xff;
	  send_buf[17]=(recv_len>>0)&0xff;

#ifdef DEBUG
	  printf("memcpy ");
#endif
	  memcpy(&send_buf[18],recv_buf,recv_len);
#ifdef DEBUG
	  printf("write ");
#endif
	  write(uart_fd,send_buf,data_size);

#ifdef DEBUG
	  printf("sts send data : len = %d\n",data_size);
	  for(n=0;n<data_size;n++){
	       printf(" %02x",send_buf[n]);
	  }
	  printf("\n");
#endif	  
	  ret= clock_gettime(CLOCK_REALTIME, &ts);     
	  localtime_r(&ts.tv_sec, &pnow); //現在日時
#ifdef DEBUG
	  printf("%04d/%02d/%02d %02d:%02d:%02d.%09ld,",
		 (pnow.tm_year+1900),(pnow.tm_mon+1),
		 pnow.tm_mday,pnow.tm_hour,pnow.tm_min,pnow.tm_sec,ts.tv_nsec);
	  for(n=0;n<data_size;n++){printf("%02x",send_buf[n]);}	  
#endif      
	  fprintf(slog,"%04d/%02d/%02d %02d:%02d:%02d.%09ld,",
		  (pnow.tm_year+1900),(pnow.tm_mon+1),
		  pnow.tm_mday,pnow.tm_hour,pnow.tm_min,pnow.tm_sec,ts.tv_nsec);
	  for(n=0;n<data_size;n++){fprintf(slog,"%02x",send_buf[n]);}
	  fprintf(slog,",\n");
	  fflush(slog);
     }else{
	  printf("UDP受信メッセージ異常 recv_len=%d\n",recv_len);
     }
}
