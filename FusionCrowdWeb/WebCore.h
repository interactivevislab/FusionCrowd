#pragma once

#include "FusionCrowdWebApi.h"

#include <WinSock2.h>


namespace FusionCrowdWeb
{
	class FC_WEB_API WebCore
	{
	public:
		WebCore(bool inIsServer);

		void Start();
		void Shutdown();

		void Disconnect();

		void Send(const char* inData, size_t inDataSize);
		void SendString(const char* inData);

		const char* Receive(size_t inDataSize);
		const char* ReceiveString();

		static void CheckSocket(SOCKET inSocket, const char* inErrorMessage);
		static void CheckWsResult(int inResult, const char* inErrorMessage);

	protected:
		SOCKET OwnSocket;
		SOCKET AnotherSocket;

		sockaddr_in GetSocketAddress(const char* inIpAdress, short inPort);

	private:
		SOCKET& _dataSocket;

		size_t _bufferSize = 512;
		char *_receiveBuffer = new char[_bufferSize + 1];
	};
}
