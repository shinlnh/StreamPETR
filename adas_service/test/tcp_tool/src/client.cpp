#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
#include <thread>
#include <chrono>

#include "../include/control_msg.h"
#include "common.h"
#define PORT 8080
#define PORT2 8081

void listenThread()
{
	int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8081
    if (setsockopt(server_fd, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT2);

    // Forcefully attaching socket to the port 8081
    if (bind(server_fd, (struct sockaddr *)&address,
             sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }

	new_socket   = 	accept(
					server_fd,
					(struct sockaddr *)&address,
					(socklen_t *)&addrlen);
	
    if (new_socket < 0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    while (true)
    {
        // Receive data
        char buffer[sizeof(control_msg)];
        int bytes_received = recv(new_socket, buffer, sizeof(buffer), 0);

        // Check for receive errors
        if (bytes_received <= 0)
        {
            continue;
        }

        // Deserialize the received data to control_msg struct
        control_msg received_msg;
        memcpy(&received_msg, buffer, sizeof(received_msg));

        // Access the fields of the received control_msg struct
        float throttle = received_msg.throttle;
        float steer = received_msg.steer;
    }

    // closing the connected socket
    close(new_socket);
    // closing the listening socket
    shutdown(server_fd, SHUT_RDWR);
	return;
}

void sendThread()
{
    int status, client_fd;
	struct sockaddr_in serv_addr;

	// Creating the socket
	if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
	{
		INFO("Socket creation error");
		return;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);

	// Convert IPv4 and IPv6 addresses from text to binary form.
	if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)
		<= 0) 
	{
		INFO("Invalid address/ Address not supported");
		return;
	}

	// Establish connection with server.
	status   = 	connect (
				client_fd, 
				(struct sockaddr*)&serv_addr,
				sizeof(serv_addr)
				);

	if (status < 0) 
	{
		INFO("Connection Failed ");
		return;
	}


	cv::VideoCapture cap("videos/test_video.mp4");

	// Check if the video file was opened successfully
    if (!cap.isOpened()) 
	{
        INFO("Error opening video file");
        return;
    }

	cv::Mat img;

	while (cap.read(img)) 
	{
		if(img.empty())
		{
			INFO("Could not read the image");
			return;
		}

		img = (img.reshape(0,1)); // Serialize the image

		int  imgSize = img.total() * img.elemSize();
    std::this_thread::sleep_for (std::chrono::milliseconds(50));
		send(client_fd, img.data, imgSize, 0);

	}
	// Close the connected socket.
	close(client_fd);

}

int main(int argc, char const* argv[])
{
	std::thread listenerThread(listenThread);
    std::thread senderThread(sendThread);
	
    listenerThread.join();
    senderThread.join();
	return 0;
}
