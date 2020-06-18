#pragma once

#include "FcWebApi.h"

#include "WsException.h"
#include "WebDataSerializer.h"
#include "WebMessage.h"

#include <WinSock2.h>
#include <map>
#include <vector>


namespace FusionCrowdWeb
{
	struct FC_WEB_API WebAddress
	{
		const char* IpAddress;
		u_short Port;

		WebAddress(const char* inIpAddress, short inPort);
		operator sockaddr_in();
	};


	class FC_WEB_API WebNode
	{
	public:
		~WebNode();

		void StartServer(WebAddress inAddress);
		void ShutdownServer();

		int AcceptInputConnection();
		int TryConnectToServer(WebAddress inAddress);
		int WaitForConnectionToServer(WebAddress inAddress);
		void Disconnect(int inSocketId);

		void Send(int inSocketId, WebCode inWebCode, const char* inData, size_t inDataSize);
		void Send(int inSocketId, WebCode inWebCode);

		template<typename DataType>
		void Send(int inSocketId, WebCode inWebCode, const DataType& inData)
		{
			char* rawData;
			auto dataSize = WebDataSerializer<DataType>::Serialize(inData, rawData);
			Send(inSocketId, inWebCode, rawData, dataSize);
			delete[] rawData;
		}

		WebMessage Receive(int inSocketId);

		template<typename DataType>
		DataType Receive(int inSocketId, WebCode inExpectedWebCode, char const* inErrorMessage)
		{
			auto message = Receive(inSocketId);
			if (message.WebCode.AsResponseCode != inExpectedWebCode.AsResponseCode)
			{
				throw FcWebException(inErrorMessage);
			}
			return WebDataSerializer<DataType>::Deserialize(message.Data);
		}

		static void GlobalStartup();
		static void GlobalCleanup();

	private:
		SOCKET _ownServerSocket;

		size_t _bufferSize = 512;
		char *_receiveBuffer = new char[_bufferSize + 1];

		std::map<int, SOCKET> _connectedSockets;
		int _freeSocketId = 0;

		int SaveConnectedSocket(SOCKET inSocket);
		SOCKET GetConnectedSocket(int inSocketId);
		std::vector<int> GetAllConnectedSocketsIds();

		static void CheckSocket(SOCKET inSocket, const char* inErrorMessage);
		static void CheckWsResult(int inResult, const char* inErrorMessage);
		static void CheckTransferredBytes(int inBytesNum, const char* inErrorMessage);
	};
}
