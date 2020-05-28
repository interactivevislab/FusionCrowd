#pragma once

#include "FusionCrowdWebApi.h"

#include "WebMessage.h"

#include <utility>
#include <WinSock2.h>
#include <map>
#include <vector>


namespace FusionCrowdWeb
{
	class FC_WEB_API WebNode
	{
	public:
		~WebNode();

		void StartServer(const char* inIpAdress, short inPort);
		void ShutdownServer();

		int AcceptInputConnection();
		int ConnectToServer(const char* inIpAdress, short inPort);
		void Disconnect(int inSocketId);

		void Send(int inSocketId, WebCode inWebCode, const char* inData, size_t inDataSize);
		std::pair<WebCode, const char*> Receive(int inSocketId);

		static void GlobalStartup();
		static void GlobalCleanup();

	private:
		SOCKET _ownServerSocket;

		size_t _bufferSize = 512;
		char *_receiveBuffer = new char[_bufferSize + 1];

		std::map<int, SOCKET> _connectedSockets;
		int _freeSocketId = 0;

		static sockaddr_in GetSocketAddress(const char* inIpAdress, short inPort);

		int SaveConnectedSocket(SOCKET inSocket);
		SOCKET GetConnectedSocket(int inSocketId);
		std::vector<int> GetAllConnectedSocketsIds();

		static void CheckSocket(SOCKET inSocket, const char* inErrorMessage);
		static void CheckWsResult(int inResult, const char* inErrorMessage);
	};
}
