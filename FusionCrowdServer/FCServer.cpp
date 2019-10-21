#include "FCServer.h"
#include <exception>
#include "WsException.h"
#include "Ws2tcpip.h"

#pragma comment(lib, "Ws2_32.lib")


FCServer::FCServer() {
	WSADATA winsockData;
	int result = WSAStartup(MAKEWORD(2, 2), &winsockData);
	if (result != 0) throw std::exception("WSAStartUp Failed");
}


FCServer::~FCServer() {
	int result = WSACleanup();
	if (result == SOCKET_ERROR) throw WsException("CleanUp Fun Failed");
}


void FCServer::Start() {
	_serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (_serverSocket == INVALID_SOCKET) throw WsException("TCP Server Socket Creation Failed");
}


void FCServer::Bind(const char* ipAdress, short port) {
	sockaddr_in address;
	address.sin_family = AF_INET;
	InetPton(AF_INET, ipAdress, &(address.sin_addr.s_addr));
	address.sin_port = htons(port);

	int result = bind(_serverSocket, (SOCKADDR*)&address, sizeof(address));
	if (result == SOCKET_ERROR) throw WsException("Binding Failed");
}


void FCServer::Listen() {
	int result = listen(_serverSocket, 2);
	if (result == SOCKET_ERROR) throw WsException("Listen Fun Failed");
}


void FCServer::Accept() {
	sockaddr_in TCPClientAdd;
	int iTCPClientAdd = sizeof(TCPClientAdd);
	_acceptSocket = accept(_serverSocket, (SOCKADDR*)&TCPClientAdd, &iTCPClientAdd);
	if (_acceptSocket == INVALID_SOCKET) throw WsException("Accept Failed");
}


void FCServer::Send(const char* data) {
	int dataSize = strlen(data) + 1;
	int result = send(_acceptSocket, data, dataSize, 0);
	if (result == SOCKET_ERROR) throw WsException("Sending Failed");
}


char* FCServer::Receive() {
	int result = recv(_acceptSocket, _receiveBuffer, _bufferSize, 0);
	if (result == SOCKET_ERROR) throw WsException("Receive Data Failed");
	return _receiveBuffer;
}


void FCServer::Shutdown() {
	int iCloseSocket = closesocket(_serverSocket);
	if (iCloseSocket == SOCKET_ERROR) throw WsException("Closing Socket Failed");
}