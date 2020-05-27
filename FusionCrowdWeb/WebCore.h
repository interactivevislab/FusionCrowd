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
		void Initialize();
		void Finalize();

		static void GlobalStartup();
		static void GlobalCleanup();

	protected:
		SOCKET OwnSocket;

		static sockaddr_in GetSocketAddress(const char* inIpAdress, short inPort);

		void Send(SOCKET inDestSocket, WebCode inWebCode, const char* inData, size_t inDataSize);
		std::pair<WebCode, const char*> Receive(SOCKET inSrcSocket);

		static void CheckSocket(SOCKET inSocket, const char* inErrorMessage);
		static void CheckWsResult(int inResult, const char* inErrorMessage);

	private:
		size_t _bufferSize = 512;
		char *_receiveBuffer = new char[_bufferSize + 1];
	};
}
