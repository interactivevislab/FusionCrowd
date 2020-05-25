#include "WebServerCore.h"

#include "WsException.h"


namespace FusionCrowdWeb
{
	WebServerCore::WebServerCore() : WebCore(true)
	{
	}


	void WebServerCore::Bind(const char* inIpAdress, short inPort)
	{
		auto address = GetSocketAddress(inIpAdress, inPort);
		auto result = bind(OwnSocket, (SOCKADDR*)&address, sizeof(address));
		if (result == SOCKET_ERROR)
		{
			throw WsException("Binding Failed");
		}
	}


	void WebServerCore::Listen()
	{
		auto result = listen(OwnSocket, 1);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Listen Fun Failed");
		}
	}


	void WebServerCore::Accept()
	{
		sockaddr_in address;
		int addressSize = sizeof(address);
		AnotherSocket = accept(OwnSocket, (SOCKADDR*)&address, &addressSize);
		if (AnotherSocket == INVALID_SOCKET)
		{
			throw WsException("Accept Failed");
		}
	}
}
