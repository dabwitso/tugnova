#ifndef UDP_CONNECTOR_H_
#define UDP_CONNECTOR_H_

#include <ros/ros.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <time.h>


namespace Udp_ns{
class UdpConnector{ //class for receiving and publishing the car info, subscribing and sending the control input
public:
    UdpConnector(int _port);
    ~UdpConnector();
    void run();
    
protected:       
    int port;
    int socket_var;
    struct sockaddr_in sock_addr;
    struct timeval tv;
    
    void init();
};

class UdpSender : public UdpConnector{
public:
    UdpSender(std::string _destination, unsigned short _port);
    ~UdpSender();
    void run();
    
private:
    void init();

    std::string destination;
};

class UdpReceiver : public UdpConnector{
public:
    UdpReceiver(unsigned short _port);
    ~UdpReceiver();
    void run();
};

};

#endif //UDP_CONNECTOR_H_
