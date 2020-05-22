#include "FCServerCore.h"

#include <exception>
#include "WsException.h"
#include "Ws2tcpip.h"

#pragma comment(lib, "Ws2_32.lib")


namespace FusionCrowdWeb
{
	FCServerCore::FCServerCore()
	{
		WSADATA winsockData;
		auto result = WSAStartup(MAKEWORD(2, 2), &winsockData);
		if (result != 0)
		{
			throw std::exception("WSAStartUp Failed");
		}
	}


	FCServerCore::~FCServerCore()
	{
		delete[] _receiveBuffer;
		auto result = WSACleanup();
		if (result == SOCKET_ERROR)
		{
			//throw WsException("CleanUp Fun Failed");	//need to log instead
		}
	}


	void FCServerCore::Start()
	{
		_serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (_serverSocket == INVALID_SOCKET)
		{
			throw WsException("TCP Server Socket Creation Failed");
		}
	}


	void FCServerCore::Bind(const char* inIpAdress, short inPort)
	{
		sockaddr_in address;
		address.sin_family = AF_INET;
		InetPton(AF_INET, inIpAdress, &(address.sin_addr.s_addr));
		address.sin_port = htons(inPort);

		auto result = bind(_serverSocket, (SOCKADDR*)&address, sizeof(address));
		if (result == SOCKET_ERROR)
		{
			throw WsException("Binding Failed");
		}
	}


	void FCServerCore::Listen()
	{
		auto result = listen(_serverSocket, 1);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Listen Fun Failed");
		}
	}


	void FCServerCore::Accept()
	{
		sockaddr_in TCPClientAdd;
		int iTCPClientAdd = sizeof(TCPClientAdd);
		_clientSocket = accept(_serverSocket, (SOCKADDR*)&TCPClientAdd, &iTCPClientAdd);
		if (_clientSocket == INVALID_SOCKET)
		{
			throw WsException("Accept Failed");
		}
	}


	void FCServerCore::Disconnect()
	{
		auto result = closesocket(_clientSocket);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Disconnect Failed");
		}
	}


	void FCServerCore::SendString(const char* inData)
	{
		int dataSize = static_cast<int>(strlen(inData)) + 1;
		auto result = send(_clientSocket, inData, dataSize, 0);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Sending Failed");
		}
	}


	void FCServerCore::Send(const char* inData, size_t inDataSize)
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


	const char* FCServerCore::ReceiveString()
	{
		auto result = recv(_clientSocket, _receiveBuffer, static_cast<int>(_bufferSize), 0);
		if (result == SOCKET_ERROR)
		{
			throw WsException("Receive Data Failed");
		}
		return _receiveBuffer;
	}


	const char* FCServerCore::Receive(size_t inDataSize)
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
			auto bytesRecieved = recv(_clientSocket, _receiveBuffer + totalBytesRecieved, bytesleft, 0);
			if (bytesRecieved == SOCKET_ERROR)
			{
				throw WsException("Receive Data Failed");
			}
			totalBytesRecieved += bytesRecieved;
			bytesleft -= bytesRecieved;
		}
		return _receiveBuffer;
	}


	void FCServerCore::Shutdown()
	{
		auto iCloseSocket = closesocket(_serverSocket);
		if (iCloseSocket == SOCKET_ERROR)
		{
			throw WsException("Closing Socket Failed");
		}
	}
}
