#pragma once

#include "FcClientApi.h"
#include <WinSock2.h>


namespace FusionCrowdWeb
{
	class FC_CLIENT_API FCClientCore
	{
	public:
		FCClientCore();
		~FCClientCore();

		void Start();
		void Connect(const char* inIpAdress, short inPort);

		void SendString(const char* inData);
		void Send(const char* inData, size_t inDataSize);
		const char* ReceiveString();
		const char* Receive(size_t inDataSize);

		void Shutdown();

	private:
		SOCKET _clientSocket;

		size_t _bufferSize = 512;
		char *_receiveBuffer = new char[_bufferSize + 1];
	};
}