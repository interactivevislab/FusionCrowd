#include "WebServerCore.h"

#include "WsException.h"


namespace FusionCrowdWeb
{
	void WebServerCore::Start(const char* inIpAdress, short inPort)
	{
		auto address = GetSocketAddress(inIpAdress, inPort);
		auto result = bind(OwnSocket, (SOCKADDR*)&address, sizeof(address));
		CheckWsResult(result, "Binding failed");

		result = listen(OwnSocket, SOMAXCONN);
		CheckWsResult(result, "Listening failed");
	}


	int WebServerCore::Accept()
	{
		sockaddr_in address;
		int addressSize = sizeof(address);
		auto clientSocket = accept(OwnSocket, (SOCKADDR*)&address, &addressSize);
		CheckSocket(clientSocket, "Accept failed");

		_clientsSockets.insert({ _freeSocketId, clientSocket });

		return _freeSocketId++;
	}


	void WebServerCore::Disconnect(int inClientId)
	{
		auto clientSocket = GetClientSocket(inClientId);

		auto result = closesocket(clientSocket);
		CheckWsResult(result, "Disconnect failed");

		_clientsSockets.erase(inClientId);
	}


	void WebServerCore::Send(int inClientId, WebCode inWebCode, const char* inData, size_t inDataSize)
	{
		WebCore::Send(GetClientSocket(inClientId), inWebCode, inData, inDataSize);
	}


	std::pair<WebCode, const char*> WebServerCore::Receive(int inClientId)
	{
		return WebCore::Receive(GetClientSocket(inClientId));
	}


	SOCKET WebServerCore::GetClientSocket(int inClientId)
	{
		auto clientSocketData = _clientsSockets.find(inClientId);
		if (clientSocketData == _clientsSockets.end())
		{
			throw FcWebException("Wrong client id");
		}

		return clientSocketData->second;
	}
}
