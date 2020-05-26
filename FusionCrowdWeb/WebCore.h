#pragma once

#include "FusionCrowdWebApi.h"

#include "WebMessage.h"

#include <utility>
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

		void Send(WebCode inWebCode, const char* inData, size_t inDataSize);
		std::pair <WebCode, const char*> Receive();

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
