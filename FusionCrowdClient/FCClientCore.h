#pragma once

#include <WinSock2.h>


class FCClientCore
{
private:
	SOCKET _clientSocket;

	const static size_t _bufferSize = 512;
	char _receiveBuffer[_bufferSize];

public:
	FCClientCore();
	~FCClientCore();

	void Start();
	void Connect(const char* ipAdress, short port);
	void Send(const char* data);
	char* Receive();
	void Shutdown();
};