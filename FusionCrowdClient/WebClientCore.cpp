#include "WebClientCore.h"

#include "WsException.h"


namespace FusionCrowdWeb
{
	WebClientCore::WebClientCore() : WebCore(false)
	{
	}


	void WebClientCore::Connect(const char* inIpAdress, short inPort)
	{
		auto address = GetSocketAddress(inIpAdress, inPort);
		auto result = connect(OwnSocket, (SOCKADDR*)&address, sizeof(address));
		if (result == SOCKET_ERROR)
		{
			throw WsException("Connection failed");
		}
	}
}
