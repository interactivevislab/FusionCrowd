#include "FCServerCore.h"
#include <exception>
#include "WsException.h"
#include "Ws2tcpip.h"

#pragma comment(lib, "Ws2_32.lib")


namespace FusionCrowdWeb {

	FCServerCore::FCServerCore() {
		WSADATA winsockData;
		int result = WSAStartup(MAKEWORD(2, 2), &winsockData);
		if (result != 0) throw std::exception("WSAStartUp Failed");
	}


	FCServerCore::~FCServerCore() {
		delete[] _receiveBuffer;
		int result = WSACleanup();
		if (result == SOCKET_ERROR) throw WsException("CleanUp Fun Failed");
	}


	void FCServerCore::Start() {
		_serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (_serverSocket == INVALID_SOCKET) throw WsException("TCP Server Socket Creation Failed");
	}


	void FCServerCore::Bind(const char* ipAdress, short port) {
		sockaddr_in address;
		address.sin_family = AF_INET;
		InetPton(AF_INET, ipAdress, &(address.sin_addr.s_addr));
		address.sin_port = htons(port);

		int result = bind(_serverSocket, (SOCKADDR*)&address, sizeof(address));
		if (result == SOCKET_ERROR) throw WsException("Binding Failed");
	}


	void FCServerCore::Listen() {
		int result = listen(_serverSocket, 1);
		if (result == SOCKET_ERROR) throw WsException("Listen Fun Failed");
	}


	void FCServerCore::Accept() {
		sockaddr_in TCPClientAdd;
		int iTCPClientAdd = sizeof(TCPClientAdd);
		_clientSocket = accept(_serverSocket, (SOCKADDR*)&TCPClientAdd, &iTCPClientAdd);
		if (_clientSocket == INVALID_SOCKET) throw WsException("Accept Failed");
	}


	void FCServerCore::Disconnect() {
		int result = closesocket(_clientSocket);
		if (result == SOCKET_ERROR) throw WsException("Disconnect Failed");
	}


	void FCServerCore::SendString(const char* data) {
		int dataSize = strlen(data) + 1;
		int result = send(_clientSocket, data, dataSize, 0);
		if (result == SOCKET_ERROR) throw WsException("Sending Failed");
	}


	void FCServerCore::Send(const char* data, size_t dataSize) {
		size_t totalBytesSended = 0;
		size_t bytesleft = dataSize;
		size_t bytesSended;

		while (totalBytesSended < dataSize) {
			bytesSended = send(_clientSocket, data + totalBytesSended, bytesleft, 0);
			if (bytesSended == SOCKET_ERROR) throw WsException("Sending Failed");
			totalBytesSended += bytesSended;
			bytesleft -= bytesSended;
		}
	}


	const char* FCServerCore::ReceiveString() {
		int result = recv(_clientSocket, _receiveBuffer, _bufferSize, 0);
		if (result == SOCKET_ERROR) throw WsException("Receive Data Failed");
		return _receiveBuffer;
	}


	const char* FCServerCore::Receive(size_t dataSize) {
		if (_bufferSize < dataSize) {
			_bufferSize = 3 * _bufferSize / 2;
			_bufferSize = (_bufferSize > dataSize) ? _bufferSize : dataSize;
			delete[] _receiveBuffer;
			_receiveBuffer = new char[_bufferSize];
		}

		size_t totalBytesRecieved = 0;
		size_t bytesleft = dataSize;
		size_t bytesRecieved;

		while (totalBytesRecieved < dataSize) {
			bytesRecieved = recv(_clientSocket, _receiveBuffer + totalBytesRecieved, bytesleft, 0);
			if (bytesRecieved == SOCKET_ERROR) throw WsException("Receive Data Failed");
			totalBytesRecieved += bytesRecieved;
			bytesleft -= bytesRecieved;
		}
		return _receiveBuffer;
	}


	void FCServerCore::Shutdown() {
		int iCloseSocket = closesocket(_serverSocket);
		if (iCloseSocket == SOCKET_ERROR) throw WsException("Closing Socket Failed");
	}

}