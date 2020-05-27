#include "WebCore.h"

#include "WsException.h"

#include "Ws2tcpip.h"


namespace FusionCrowdWeb
{
	void WebCore::GlobalStartup()
	{
		WSADATA winsockData;
		auto result = WSAStartup(MAKEWORD(2, 2), &winsockData);
		if (result != 0)
		{
			throw FcWebException("WSAStartUp failed");
		}
	}


	void WebCore::GlobalCleanup()
	{
		auto result = WSACleanup();
		CheckWsResult(result, "WSACleanup failed");
	}


	void WebCore::Initialize()
	{
		OwnSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		CheckSocket(OwnSocket, "TCP socket creation failed");
	}


	void WebCore::Finalize()
	{
		delete[] _receiveBuffer;

		auto result = closesocket(OwnSocket);
		CheckWsResult(result, "Closing socket failed");
	}


	sockaddr_in WebCore::GetSocketAddress(const char* inIpAdress, short inPort)
	{
		sockaddr_in address;
		address.sin_family = AF_INET;
		InetPton(AF_INET, inIpAdress, &(address.sin_addr.s_addr));
		address.sin_port = htons(inPort);

		return address;
	}


	void WebCore::Send(SOCKET inDestSocket, WebCode inWebCode, const char* inData, size_t inDataSize)
	{
		WebMessageHead messageHead = { inWebCode, inDataSize };
		auto bytesSended = send(inDestSocket, reinterpret_cast<const char*>(&messageHead), sizeof(WebMessageHead), 0);
		CheckWsResult(bytesSended, "Sending failed");

		int totalBytesSended = 0;
		int bytesleft = static_cast<int>(inDataSize);
		while (totalBytesSended < inDataSize)
		{
			bytesSended = send(inDestSocket, inData + totalBytesSended, bytesleft, 0);
			CheckWsResult(bytesSended, "Sending failed");
			totalBytesSended += bytesSended;
			bytesleft -= bytesSended;
		}
	}


	std::pair<WebCode, const char*> WebCore::Receive(SOCKET inSrcSocket)
	{
		WebMessageHead messageHead;
		auto bytesRecieved = recv(inSrcSocket, reinterpret_cast<char*>(&messageHead), sizeof(WebMessageHead), 0);
		CheckWsResult(bytesRecieved, "Receive data failed");

		if (_bufferSize < messageHead.MessageLength)
		{
			_bufferSize = 3 * _bufferSize / 2;
			_bufferSize = (_bufferSize > messageHead.MessageLength) ? _bufferSize : messageHead.MessageLength;
			delete[] _receiveBuffer;
			_receiveBuffer = new char[_bufferSize];
		}

		int totalBytesRecieved = 0;
		int bytesleft = static_cast<int>(messageHead.MessageLength);
		while (totalBytesRecieved < messageHead.MessageLength)
		{
			auto bytesRecieved = recv(inSrcSocket, _receiveBuffer + totalBytesRecieved, bytesleft, 0);
			CheckWsResult(bytesRecieved, "Receive data failed");
			totalBytesRecieved += bytesRecieved;
			bytesleft -= bytesRecieved;
		}

		return std::make_pair(messageHead.WebCode, _receiveBuffer);
	}


	void WebCore::CheckSocket(SOCKET inSocket, const char* inErrorMessage)
	{
		if (inSocket == INVALID_SOCKET)
		{
			throw WsException(inErrorMessage);
		}
	}


	void WebCore::CheckWsResult(int inResult, const char* inErrorMessage)
	{
		if (inResult == SOCKET_ERROR)
		{
			throw WsException(inErrorMessage);
		}
	}
}
