#include "WebNode.h"

#include "Ws2tcpip.h"


namespace FusionCrowdWeb
{
	WebNode::~WebNode()
	{
		if (_receiveBuffer != nullptr)
		{
			delete[] _receiveBuffer;
		}
	}


	void WebNode::GlobalStartup()
	{
		WSADATA winsockData;
		auto result = WSAStartup(MAKEWORD(2, 2), &winsockData);
		if (result != 0)
		{
			throw FcWebException("WSAStartUp failed");
		}
	}


	void WebNode::GlobalCleanup()
	{
		auto result = WSACleanup();
		CheckWsResult(result, "WSACleanup failed");
	}


	void WebNode::StartServer(WebAddress inAddress)
	{
		_ownServerSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		CheckSocket(_ownServerSocket, "TCP socket creation failed");

		sockaddr_in address = inAddress;
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


	int WebNode::AcceptInputConnection()
	{
		sockaddr_in address;
		int addressSize = sizeof(address);
		auto clientSocket = accept(_ownServerSocket, (SOCKADDR*)&address, &addressSize);
		CheckSocket(clientSocket, "Accept failed");

		return SaveConnectedSocket(clientSocket);
	}


	int WebNode::TryConnectToServer(WebAddress inAddress)
	{
		auto connectedSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		CheckSocket(connectedSocket, "TCP socket creation failed");

		sockaddr_in address = inAddress;
		auto result = connect(connectedSocket, (SOCKADDR*)&address, sizeof(address));
		if (result == SOCKET_ERROR)
		{
			throw WsException("Connection failed");
		}

		return SaveConnectedSocket(connectedSocket);
	}


	int WebNode::WaitForConnectionToServer(WebAddress inAddress)
	{
		while (true)
		{
			try
			{
				return TryConnectToServer(inAddress);
			}
			catch (WsException e)
			{
				//connection error - try again
			}
		}
	}


	void WebNode::Disconnect(int inSocketId)
	{
		auto socket = GetConnectedSocket(inSocketId);

		auto result = closesocket(socket);
		CheckWsResult(result, "Disconnect failed");

		_connectedSockets.erase(inSocketId);
	}


	void WebNode::Send(int inSocketId, WebCode inWebCode, const char* inData, size_t inDataSize)
	{
		auto destSocket = GetConnectedSocket(inSocketId);

		WebMessageHead messageHead = { inWebCode, inDataSize };
		auto bytesSended = send(destSocket, reinterpret_cast<const char*>(&messageHead), sizeof(WebMessageHead), 0);
		CheckTransferredBytes(bytesSended, "Sending failed");

		int totalBytesSended = 0;
		int bytesleft = static_cast<int>(inDataSize);
		while (totalBytesSended < inDataSize)
		{
			bytesSended = send(destSocket, inData + totalBytesSended, bytesleft, 0);
			CheckTransferredBytes(bytesSended, "Sending failed");
			totalBytesSended += bytesSended;
			bytesleft -= bytesSended;
		}
	}

	void WebNode::Send(int inSocketId, WebCode inWebCode)
	{
		Send(inSocketId, inWebCode, nullptr, 0);
	}


	WebMessage WebNode::Receive(int inSocketId)
	{
		auto srcSocket = GetConnectedSocket(inSocketId);

		WebMessageHead messageHead;
		auto bytesRecieved = recv(srcSocket, reinterpret_cast<char*>(&messageHead), sizeof(WebMessageHead), 0);
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
				auto bytesRecieved = recv(srcSocket, _receiveBuffer + totalBytesRecieved, bytesleft, 0);
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


	int WebNode::SaveConnectedSocket(SOCKET inSocket)
	{
		_connectedSockets.insert({ _freeSocketId, inSocket });
		return _freeSocketId++;
	}


	SOCKET WebNode::GetConnectedSocket(int inSocketId)
	{
		auto clientSocketData = _connectedSockets.find(inSocketId);
		if (clientSocketData == _connectedSockets.end())
		{
			throw FcWebException("Wrong socket id");
		}

		return clientSocketData->second;
	}


	std::vector<int> WebNode::GetAllConnectedSocketsIds()
	{
		std::vector<int> ids;
		for (auto it = _connectedSockets.begin(); it != _connectedSockets.end(); ++it)
		{
			ids.push_back(it->first);
		}

		return ids;
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
