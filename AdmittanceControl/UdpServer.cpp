// UDP Server Headers
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "UdpServer.h"


UDPServer::UDPServer(std::string addr, int port): _port(port), _addr(addr){

	// Initialize socket
	_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (_sockfd < 0){
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}

	// Reset and fill server address structs
	memset(&_servaddr, 0, sizeof(_servaddr));
	memset(&_cliaddr, 0, sizeof(_cliaddr));

	_servaddr.sin_family = AF_INET;
	inet_pton(AF_INET, _addr.c_str(), &_servaddr.sin_addr);
	_servaddr.sin_port = htons(_port);

	// Bind the socket with the server address
	if (bind(_sockfd, (const struct sockaddr *)&_servaddr,sizeof(_servaddr)) < 0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
}

void UDPServer::ConnectIfNecessary(){
    /*
    select(2) system call monitors if there is an incoming messge from the client
    If there is, select(2) will return a number greater than 0.  We will then connect
    run ConnectClient() which reads in the message and gets the client's ip info
    so the server can send messages to it
    */
    FD_ZERO(&_readset);
    FD_SET(_sockfd, &_readset);
    int ret = select(_sockfd+1, &_readset, NULL, NULL, &_select_timeout);
    if (ret > 0){
        ConnectClient();
    }
}

std::string UDPServer::GetAddress(){
    // return server's ip address
	return _addr;
}

int UDPServer::GetPort(){
    // return server's port
	return _port;
}

bool UDPServer::IsConnected() const{
    // return server-client connection status
    return _connected;
}

void UDPServer::ConnectClient(){
    /*
    recvfrom() system call reads the message into the buffer and fills the _cliaddr
    struct with the client's ip info.  The message in the buffer is not important.
    We only do this so we get the client's ip address info
    */
	char buffer[_maxlen];
	int n;
	n = recvfrom(_sockfd, (char *)buffer, _maxlen,
				MSG_WAITALL, ( struct sockaddr *) &_cliaddr, &_cliaddrlen);
	_connected = true;
}

template<typename T>
void UDPServer::Send(T* data, int ndata){
    // Connect/Reconnect to client if necessary (get's clients ip address info)
    ConnectIfNecessary();
    // Send data if the server is connected to a client
	if (_connected){
		sendto(_sockfd, (const T *) data, sizeof(data)*ndata,
  				 MSG_DONTWAIT, (const struct sockaddr *) &_cliaddr, _cliaddrlen);
	}
}

void UDPServer::Send(double* data, int ndata){
    // Connect/Reconnect to client if necessary (get's clients ip address info)
    ConnectIfNecessary();
    // Send data if the server is connected to a client
	if (_connected){
		sendto(_sockfd, (const double *) data, sizeof(data)*ndata,
  				 MSG_DONTWAIT, (const struct sockaddr *) &_cliaddr, _cliaddrlen);
	}
}
