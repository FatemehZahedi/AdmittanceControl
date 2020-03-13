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

class UDPServer{
private:
    // UDP/IP socket info
	std::string _addr;
	int _port;
    int _sockfd;

    // Server/Client address info
	struct sockaddr_in _servaddr;
	struct sockaddr_in _cliaddr;
	socklen_t _cliaddrlen = sizeof(_cliaddr);

    // Recv/send limit
    const int _maxlen = 1024;

    // Connection State
    bool _connected = false;

    // select(2) related data
    fd_set _readset;
    struct timeval _select_timeout = {.tv_sec = 0, .tv_usec = 0};

public:
    // Class Methods
	UDPServer(std::string addr, int port);
	std::string GetAddress();
	int GetPort();
	bool IsConnected() const;
	void ConnectClient();
    void ConnectIfNecessary();
	template<typename T>
	void Send(T* data, int ndata);
	void Send(double* data, int ndata);
};
