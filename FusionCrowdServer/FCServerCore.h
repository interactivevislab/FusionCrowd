#pragma once

#include <WinSock2.h>


namespace FusionCrowdWeb
{
	class FCServerCore
	{
	public:
		FCServerCore();
		~FCServerCore();

		void Start();
		void Bind(const char* inIpAdress, short inPort);
		void Listen();
		void Accept();
		void Disconnect();

		void SendString(const char* inData);
		void Send(const char* inData, size_t inDataSize);
		const char* ReceiveString(); 
		const char* Receive(size_t inDataSize);

		void Shutdown();

	private:
		SOCKET _serverSocket;
		SOCKET _clientSocket;

		size_t _bufferSize = 512;
		char *_receiveBuffer = new char[_bufferSize + 1];
	};
}
