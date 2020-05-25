#include "WebCore.h"

#include "WsException.h"

#include "Ws2tcpip.h"


namespace FusionCrowdWeb
{
	WebCore::WebCore(bool inIsServer) : _dataSocket(inIsServer ? AnotherSocket : OwnSocket)
	{
	}


	void WebCore::Start()
	{
		WSADATA winsockData;
		auto result = WSAStartup(MAKEWORD(2, 2), &winsockData);
		if (result != 0)
		{
			throw FcWebException("WSAStartUp failed");
		}

		OwnSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (OwnSocket == INVALID_SOCKET)
		{
			throw WsException("TCP socket creation failed");
		}
	}


	void WebCore::Shutdown()
	{
		delete[] _receiveBuffer;

		auto result = closesocket(OwnSocket);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Closing socket failed");
		}

		result = WSACleanup();
		if (result == SOCKET_ERROR)
		{
			throw WsException("WSACleanup failed");
		}
	}


	sockaddr_in WebCore::GetSocketAddress(const char* inIpAdress, short inPort)
	{
		sockaddr_in address;
		address.sin_family = AF_INET;
		InetPton(AF_INET, inIpAdress, &(address.sin_addr.s_addr));
		address.sin_port = htons(inPort);

		return address;
	}


	void WebCore::Disconnect()
	{
		auto result = closesocket(_dataSocket);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Disconnect failed");
		}
	}


	void WebCore::Send(const char* inData, size_t inDataSize)
	{
		int totalBytesSended = 0;
		int bytesleft = static_cast<int>(inDataSize);

		while (totalBytesSended < inDataSize)
		{
			auto bytesSended = send(_dataSocket, inData + totalBytesSended, bytesleft, 0);
			if (bytesSended == SOCKET_ERROR)
			{
				throw WsException("Sending failed");
			}
			totalBytesSended += bytesSended;
			bytesleft -= bytesSended;
		}
	}


	void WebCore::SendString(const char* inData)
	{
		int dataSize = static_cast<int>(strlen(inData)) + 1;
		auto result = send(_dataSocket, inData, dataSize, 0);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Sending failed");
		}
	}


	const char* WebCore::Receive(size_t inDataSize)
	{
		if (_bufferSize < inDataSize)
		{
			_bufferSize = 3 * _bufferSize / 2;
			_bufferSize = (_bufferSize > inDataSize) ? _bufferSize : inDataSize;
			delete[] _receiveBuffer;
			_receiveBuffer = new char[_bufferSize];
		}

		int totalBytesRecieved = 0;
		int bytesleft = static_cast<int>(inDataSize);

		while (totalBytesRecieved < inDataSize)
		{
			auto bytesRecieved = recv(_dataSocket, _receiveBuffer + totalBytesRecieved, bytesleft, 0);
			if (bytesRecieved == SOCKET_ERROR)
			{
				throw WsException("Receive data failed");
			}
			totalBytesRecieved += bytesRecieved;
			bytesleft -= bytesRecieved;
		}
		return _receiveBuffer;
	}


	const char* WebCore::ReceiveString()
	{
		auto result = recv(_dataSocket, _receiveBuffer, static_cast<int>(_bufferSize), 0);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Receive data failed");
		}
		return _receiveBuffer;
	}
}
