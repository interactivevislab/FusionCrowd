#include "FCClientCore.h"

#include <exception>
#include "WsException.h"
#include "Ws2tcpip.h"

#pragma comment(lib, "Ws2_32.lib")


namespace FusionCrowdWeb
{
	FCClientCore::FCClientCore()
	{
		WSADATA winsockData;
		auto result = WSAStartup(MAKEWORD(2, 2), &winsockData);
		if (result != 0)
		{
			throw std::exception("WSAStartUp Failed");
		}
	}


	FCClientCore::~FCClientCore()
	{
		auto result = WSACleanup();
		if (result == SOCKET_ERROR)
		{
			//throw WsException("CleanUp Fun Failed");	//need to log instead
		}
	}


	void FCClientCore::Start()
	{
		_clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (_clientSocket == INVALID_SOCKET)
		{
			throw WsException("TCP Client Socket Creation Failed");
		}
	}


	void FCClientCore::Connect(const char* inIpAdress, short inPort)
	{
		sockaddr_in serverAddress;
		serverAddress.sin_family = AF_INET;
		InetPton(AF_INET, inIpAdress, &(serverAddress.sin_addr.s_addr));
		serverAddress.sin_port = htons(inPort);

		auto result = connect(_clientSocket, (SOCKADDR*)&serverAddress, sizeof(serverAddress));
		if (result == SOCKET_ERROR)
		{
			throw WsException("Connection Failed");
		}
	}


	void FCClientCore::SendString(const char* inData)
	{
		int dataSize = static_cast<int>(strlen(inData)) + 1;
		auto result = send(_clientSocket, inData, dataSize, 0);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Sending Failed");
		}
	}


	void FCClientCore::Send(const char* inData, size_t inDataSize)
	{
		int totalBytesSended = 0;
		int bytesleft = static_cast<int>(inDataSize);

		while (totalBytesSended < inDataSize)
		{
			auto bytesSended = send(_clientSocket, inData + totalBytesSended, bytesleft, 0);
			if (bytesSended == SOCKET_ERROR)
			{
				throw WsException("Sending Failed");
			}
			totalBytesSended += bytesSended;
			bytesleft -= bytesSended;
		}
	}


	const char* FCClientCore::ReceiveString()
	{
		auto result = recv(_clientSocket, _receiveBuffer, static_cast<int>(_bufferSize), 0);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Receive Data Failed");
		}
		return _receiveBuffer;
	}


	const char* FCClientCore::Receive(size_t inDataSize)
	{
		auto minSize = inDataSize + 1;
		if (_bufferSize < minSize)
		{
			_bufferSize = 3 * _bufferSize / 2;
			_bufferSize = (_bufferSize > minSize) ? _bufferSize : minSize;
			delete[] _receiveBuffer;
			_receiveBuffer = new char[_bufferSize];
		}

		int totalBytesRecieved = 0;
		int bytesleft = static_cast<int>(inDataSize);

		while (totalBytesRecieved < inDataSize)
		{
			auto bytesRecieved = recv(_clientSocket, _receiveBuffer + totalBytesRecieved, bytesleft, 0);
			if (bytesRecieved == SOCKET_ERROR)
			{
				throw WsException("Receive Data Failed");
			}
			totalBytesRecieved += bytesRecieved;
			bytesleft -= bytesRecieved;
		}
		_receiveBuffer[inDataSize] = '\0';

		return _receiveBuffer;
	}


	void FCClientCore::Shutdown()
	{
		auto result = closesocket(_clientSocket);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Closing Socket Failed");
		}
	}

}