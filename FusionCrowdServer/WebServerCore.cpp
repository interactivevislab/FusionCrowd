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
		CheckWsResult(result, "Binding failed");
	}


	void WebServerCore::Listen()
	{
		auto result = listen(OwnSocket, 1);
		CheckWsResult(result, "Listening failed");
	}


	void WebServerCore::Accept()
	{
		sockaddr_in address;
		int addressSize = sizeof(address);
		AnotherSocket = accept(OwnSocket, (SOCKADDR*)&address, &addressSize);
		CheckSocket(AnotherSocket, "Accept failed");
	}
}
