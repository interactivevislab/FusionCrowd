#pragma once

#include <WinSock2.h>

namespace FusionCrowdWeb {

	class FCClientCore
	{
	private:
		SOCKET _clientSocket;

		size_t _bufferSize = 512;
		char *_receiveBuffer = new char[_bufferSize]; // _bufferSize + 1 ?

	public:
		FCClientCore();
		~FCClientCore();

		void Start();
		void Connect(const char* ipAdress, short port);

		void SendString(const char* data);
		void Send(const char* data, size_t dataSize);
		const char* ReceiveString();
		const char* Receive(size_t dataSize);

		void Shutdown();
	};

}