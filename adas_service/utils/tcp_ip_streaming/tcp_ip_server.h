#ifndef TCP_IP_SERVER_H
#define TCP_IP_SERVER_H

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp> //for cv::Mat datatype
#include <opencv2/imgcodecs.hpp> //for encode 

#define PORT 8080

class TcpIpServer
{
private:
    int server_fd; // socket file descriptor
    int client_socket; //place holder for client binding
    struct sockaddr_in address;
    int opt = 1;
    
public:
    TcpIpServer();
    void startServer();
    cv::Mat getNewFrame();
    void closeConnection();
};

#endif // TCP_IP_SERVER_H
