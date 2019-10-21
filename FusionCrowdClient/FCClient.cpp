#include "FCClient.h"
#include <exception>
#include "WsException.h"
#include "Ws2tcpip.h"

#pragma comment(lib, "Ws2_32.lib")


FCClient::FCClient() {
	WSADATA winsockData;
	int result = WSAStartup(MAKEWORD(2, 2), &winsockData);
	if (result != 0) throw std::exception("WSAStartUp Failed");
}


FCClient::~FCClient() {
	int result = WSACleanup();
	if (result == SOCKET_ERROR) throw WsException("CleanUp Fun Failed");
}


void FCClient::Start() {
	_clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (_clientSocket == INVALID_SOCKET) throw WsException("TCP Client Socket Creation Failed");
}


void FCClient::Connect(const char* ipAdress, short port) {
	sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	InetPton(AF_INET, ipAdress, &(serverAddress.sin_addr.s_addr));
	serverAddress.sin_port = htons(port);

	int result = connect(_clientSocket, (SOCKADDR*)&serverAddress, sizeof(serverAddress));
	if (result == SOCKET_ERROR) throw WsException("Connection Failed");
}


void FCClient::Send(const char* data) {
	int dataSize = strlen(data) + 1;
	int result = send(_clientSocket, data, dataSize, 0);
	if (result == SOCKET_ERROR) throw WsException("Sending Failed");
}


char* FCClient::Receive() {
	int result = recv(_clientSocket, _receiveBuffer, _bufferSize, 0);
	if (result == SOCKET_ERROR) throw WsException("Receive Data Failed");
	return _receiveBuffer;
}


void FCClient::Shutdown() {
	int result = closesocket(_clientSocket);
	if (result == SOCKET_ERROR) throw WsException("Closing Socket Failed");
}