#pragma once

#include <WinSock2.h>

namespace FusionCrowdWeb {

	class FCServerCore
	{
	private:
		SOCKET _serverSocket;
		SOCKET _clientSocket;

		size_t _bufferSize = 512;
		char *_receiveBuffer = new char[_bufferSize]; // _bufferSize + 1 ?

	public:
		FCServerCore();
		~FCServerCore();

		void Start();
		void Bind(const char* ipAdress, short port);
		void Listen();
		void Accept();
		void Disconnect();

		void SendString(const char* data);
		void Send(const char* data, size_t dataSize);
		const char* ReceiveString(); 
		const char* Receive(size_t dataSize);

		void Shutdown();
	};

}