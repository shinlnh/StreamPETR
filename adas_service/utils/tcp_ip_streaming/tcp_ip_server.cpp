#include "tcp_ip_streaming/tcp_ip_server.h"
#include "common.h"

TcpIpServer::TcpIpServer()
{

}

void TcpIpServer::startServer()
{
    int addrlen = sizeof(this->address);

    // Create server socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        ERROR("socket status code = %d", server_fd);
        exit(EXIT_FAILURE);
    }

    // Options
    if (setsockopt(server_fd, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Bind Socket
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0){
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Listen (blocking)
    // Waiting for a client to bind with
    if (listen(server_fd, 5) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    sockaddr_in clientAddress;
    socklen_t clientAddrLength = sizeof(clientAddress);
    // Accept
    if ((client_socket  =  accept(
                        server_fd,
                        (struct sockaddr *)&clientAddress,
                        &clientAddrLength)
                        ) < 0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    INFO("Accepted connection from: %s", inet_ntoa(clientAddress.sin_addr));
}

cv::Mat TcpIpServer::getNewFrame()
{
    // Receive the size of the compressed data (4 bytes)
    uint32_t msg_size;
    ssize_t bytes_received = recv(client_socket, &msg_size, sizeof(msg_size), 0);

    if (bytes_received != sizeof(msg_size)) 
    {
        ERROR("Error receiving TCP/IP message size");
        return {};
    }

    // Receive the compressed frame data
    std::vector<uint8_t> frame_data(msg_size);
    ssize_t total_bytes_received = 0;

    while (total_bytes_received < msg_size) 
    {
        ssize_t bytes = recv(client_socket, frame_data.data() + total_bytes_received, 
                                msg_size - total_bytes_received, 0);

        if (bytes < 0) 
        {
            ERROR("Error receiving image frame data");
            return {};
        }

        total_bytes_received += bytes;
    }
    
    // Decode the compressed frame data using OpenCV and return
    cv::Mat frame = cv::imdecode(frame_data, cv::IMREAD_COLOR);
    return frame;
}

void TcpIpServer::closeConnection()
{
    // closing the connected socket
    close(client_socket);
    // closing the listening socket
    shutdown(server_fd, SHUT_RDWR);
    INFO("Shut down TCP/IP connection section");
    return;
}