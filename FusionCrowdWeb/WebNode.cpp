#include "WebNode.h"

#include <chrono>

#include "Ws2tcpip.h"


namespace FusionCrowdWeb
{
	bool WebNode::_isWsaStarted = false;

	
	WebNode::~WebNode()
	{
		if (_receiveBuffer != nullptr)
		{
			delete[] _receiveBuffer;
		}
	}


	void WebNode::GlobalStartup()
	{
		if (_isWsaStarted)
		{
			return;
		}
		
		WSADATA winsockData;
		auto result = WSAStartup(MAKEWORD(2, 2), &winsockData);
		if (result != 0)
		{
			throw FcWebException("WSAStartUp failed");
		}

		_isWsaStarted = true;
	}


	void WebNode::GlobalCleanup()
	{
		auto result = WSACleanup();
		CheckWsResult(result, "WSACleanup failed");
		
		_isWsaStarted = false;
	}


	void WebNode::StartServer(u_short inPort)
	{
		_ownServerSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		CheckSocket(_ownServerSocket, "TCP socket creation failed");

		sockaddr_in address = WebAddress("127.0.0.1", inPort);
		auto result = bind(_ownServerSocket, (SOCKADDR*)&address, sizeof(address));
		CheckWsResult(result, "Binding failed");

		result = listen(_ownServerSocket, SOMAXCONN);
		CheckWsResult(result, "Listening failed");
	}


	void WebNode::ShutdownServer()
	{
		auto result = closesocket(_ownServerSocket);
		CheckWsResult(result, "Closing socket failed");
	}


	SOCKET WebNode::AcceptInputConnection()
	{
		sockaddr_in address;
		int addressSize = sizeof(address);
		auto clientSocket = accept(_ownServerSocket, (SOCKADDR*)&address, &addressSize);
		CheckSocket(clientSocket, "Accept failed");

		return clientSocket;
	}


	SOCKET WebNode::TryConnectToServer(WebAddress inAddress)
	{
		auto connectedSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		CheckSocket(connectedSocket, "TCP socket creation failed");

		sockaddr_in address = inAddress;
		auto result = connect(connectedSocket, (SOCKADDR*)&address, sizeof(address));
		if (result == SOCKET_ERROR)
		{
			throw WsException("Connection failed");
		}

		return connectedSocket;
	}


	SOCKET WebNode::WaitForConnectionToServer(WebAddress inAddress, float inConnectionTimeout)
	{
		auto connectionStart = std::chrono::system_clock::now();

		bool tryToConnect = true;
		while (tryToConnect)
		{
			try
			{
				return TryConnectToServer(inAddress);
			}
			catch (WsException e)
			{
				auto now = std::chrono::system_clock::now();
				std::chrono::duration<float> timePassed = now - connectionStart;
				tryToConnect = timePassed.count() < inConnectionTimeout;
			}
		}

		throw FcWebException("Connection timeout");
	}


	void WebNode::Disconnect(SOCKET inSocket)
	{
		auto result = closesocket(inSocket);
		CheckWsResult(result, "Disconnect failed");
	}


	void WebNode::Send(SOCKET inSocket, WebCode inWebCode, const char* inData, size_t inDataSize)
	{
		WebMessageHead messageHead = { inWebCode, inDataSize };
		auto bytesSended = send(inSocket, reinterpret_cast<const char*>(&messageHead), sizeof(WebMessageHead), 0);
		CheckTransferredBytes(bytesSended, "Sending failed");

		int totalBytesSended = 0;
		int bytesleft = static_cast<int>(inDataSize);
		while (totalBytesSended < inDataSize)
		{
			bytesSended = send(inSocket, inData + totalBytesSended, bytesleft, 0);
			CheckTransferredBytes(bytesSended, "Sending failed");
			totalBytesSended += bytesSended;
			bytesleft -= bytesSended;
		}
	}

	void WebNode::Send(SOCKET inSocket, WebCode inWebCode)
	{
		Send(inSocket, inWebCode, nullptr, 0);
	}


	WebMessage WebNode::Receive(SOCKET inSocket)
	{
		WebMessageHead messageHead;
		auto bytesRecieved = recv(inSocket, reinterpret_cast<char*>(&messageHead), sizeof(WebMessageHead), 0);
		CheckTransferredBytes(bytesRecieved, "Receive data failed");

		auto messageLength = messageHead.MessageLength;

		if (messageLength > 0)
		{
			if (_bufferSize < messageLength)
			{
				_bufferSize = 3 * _bufferSize / 2;
				_bufferSize = (_bufferSize > messageLength) ? _bufferSize : messageLength;
				delete[] _receiveBuffer;
				_receiveBuffer = new char[_bufferSize];
			}

			int totalBytesRecieved = 0;
			int bytesleft = static_cast<int>(messageLength);
			while (totalBytesRecieved < messageLength)
			{
				auto bytesRecieved = recv(inSocket, _receiveBuffer + totalBytesRecieved, bytesleft, 0);
				CheckTransferredBytes(bytesRecieved, "Receive data failed");
				totalBytesRecieved += bytesRecieved;
				bytesleft -= bytesRecieved;
			}

			return WebMessage { messageHead.WebCode, _receiveBuffer };
		}
		else
		{
			return WebMessage { messageHead.WebCode, nullptr };
		}
	}


	void WebNode::CheckSocket(SOCKET inSocket, const char* inErrorMessage)
	{
		if (inSocket == INVALID_SOCKET)
		{
			throw WsException(inErrorMessage);
		}
	}


	void WebNode::CheckWsResult(int inResult, const char* inErrorMessage)
	{
		if (inResult == SOCKET_ERROR)
		{
			throw WsException(inErrorMessage);
		}
	}


	void WebNode::CheckTransferredBytes(int inBytesNum, const char* inErrorMessage)
	{
		if ((inBytesNum == SOCKET_ERROR) || (inBytesNum == 0))
		{
			throw WsException(inErrorMessage);
		}
	}


	WebAddress::WebAddress(const char* inIpAddress, u_short inPort) : IpAddress(inIpAddress), Port(inPort)
	{
	}


	WebAddress::operator sockaddr_in()
	{
		sockaddr_in address;
		address.sin_family = AF_INET;
		InetPton(AF_INET, IpAddress, &(address.sin_addr.s_addr));
		address.sin_port = htons(Port);

		return address;
	}
}
