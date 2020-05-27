#include "WebClientCore.h"

#include "WsException.h"


namespace FusionCrowdWeb
{
	void WebClientCore::Connect(const char* inIpAdress, short inPort)
	{
		auto address = GetSocketAddress(inIpAdress, inPort);
		auto result = connect(OwnSocket, (SOCKADDR*)&address, sizeof(address));
		if (result == SOCKET_ERROR)
		{
			throw WsException("Connection failed");
		}
	}


	void WebClientCore::Disconnect()
	{
		auto result = shutdown(OwnSocket, 2);
		CheckWsResult(result, "Disconnect failed");
	}


	void WebClientCore::Send(WebCode inWebCode, const char* inData, size_t inDataSize)
	{
		WebCore::Send(OwnSocket, inWebCode, inData, inDataSize);
	}


	std::pair<WebCode, const char*> WebClientCore::Receive()
	{
		return WebCore::Receive(OwnSocket);
	}
}
