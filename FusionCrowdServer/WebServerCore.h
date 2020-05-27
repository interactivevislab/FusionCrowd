#pragma once

#include "WebCore.h"

#include <map>


namespace FusionCrowdWeb
{
	class WebServerCore : public WebCore
	{
	public:
		void Start(const char* inIpAdress, short inPort);
		int Accept();
		void Disconnect(int inClientId);

		void Send(int inClientId, WebCode inWebCode, const char* inData, size_t inDataSize);
		std::pair<WebCode, const char*> Receive(int inClientId);

	private:
		std::map<int, SOCKET> _clientsSockets;
		int _freeSocketId = 0;

		SOCKET GetClientSocket(int inClientId);
	};
}
