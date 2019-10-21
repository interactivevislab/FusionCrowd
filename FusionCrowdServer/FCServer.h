#pragma once

#include <WinSock2.h>


class FCServer
{
private:
	SOCKET _serverSocket;
	SOCKET _acceptSocket;

	const static size_t _bufferSize = 512;
	char _receiveBuffer[_bufferSize];

public:
	FCServer();
	~FCServer();

	void Start();
	void Bind(const char* ipAdress, short port);
	void Listen();
	void Accept();
	void Send(const char* data);
	char* Receive();
	void Shutdown();
};