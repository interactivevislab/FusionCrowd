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
	/**
	* \struct WebAddress
	* \brief Address for web socket.
	*/
	struct FC_WEB_API WebAddress
	{
		/** IPv4 address part as string. */
		const char* IpAddress;

		/** Port address part. */
		u_short Port;

		WebAddress(const char* inIpAddress, u_short inPort);
		operator sockaddr_in();
	};


	/**
	* \struct WebNode
	* \brief Class for organizing data transfer via web sockets.
	*/
	class FC_WEB_API WebNode
	{
	public:
		~WebNode();

		/**
		* \fn StartServer
		* \brief Starts server on this node.
		*
		* @param inAddress	Socket address to start.
		*/
		void StartServer(WebAddress inAddress);

		/**
		* \fn ShutdownServer
		* \brief Shutdown server on this node.
		*/
		void ShutdownServer();

		/**
		* \fn AcceptInputConnection
		* \brief Waits and accepts incoming connection from other node.
		*
		* @return	Connected socket id.
		*/
		int AcceptInputConnection();

		/**
		* \fn TryConnectToServer
		* \brief Once trying to connect to another node's server.
		*
		* @param inAddress	Socket address of other node's server.
		*
		* @return	Connected socket id.
		*/
		int TryConnectToServer(WebAddress inAddress);

		/**
		* \fn TryConnectToServer
		* \brief Repeatedly trying to connect to another node's server until success.
		*
		* @param inAddress	Socket address of other node's server.
		*
		* @return	Connected socket id.
		*/
		int WaitForConnectionToServer(WebAddress inAddress);

		/**
		* \fn TryConnectToServer
		* \brief Disconnects from other node.
		*
		* @param inSocketId	Id of socket to disconnect.
		*/
		void Disconnect(int inSocketId);

		/**
		* \fn Send
		* \brief Sends data to other node.
		*
		* @param inSocketId	Id of socket to send data.
		* @param inWebCode	WebCode for short data description.
		* @param inData		Data to send.
		* @param inDataSize	Size of data to send.
		*/
		void Send(int inSocketId, WebCode inWebCode, const char* inData, size_t inDataSize);

		/**
		* \fn Send
		* \brief Sends data to other node.
		*
		* @param inSocketId	Id of socket to send data.
		* @param inWebCode	WebCode for short data description.
		*/
		void Send(int inSocketId, WebCode inWebCode);

		/**
		* \fn Send
		* \brief Sends data to other node.
		*
		* @param inSocketId	Id of socket to send data.
		* @param inWebCode	WebCode for short data description.
		* @param inData		Data to send.
		*/
		template<typename DataType>
		void Send(int inSocketId, WebCode inWebCode, const DataType& inData)
		{
			char* rawData;
			auto dataSize = WebDataSerializer<DataType>::Serialize(inData, rawData);
			Send(inSocketId, inWebCode, rawData, dataSize);
			delete[] rawData;
		}

		/**
		* \fn Receive
		* \brief Receives data from other node.
		*
		* @param inSocketId	Id of socket to receive data from.
		*
		* @return	Received data.
		*/
		WebMessage Receive(int inSocketId);

		/**
		* \fn Receive
		* \brief Receives data from other node.
		*
		* @param inSocketId			Id of socket to receive data from.
		* @param inExpectedWebCode	WebCode which is expected to be received.
		* @param inErrorMessage		Error message in case of receipt unexpected WebCode.
		*
		* @return	Received data.
		*/
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

		/**
		* \fn GlobalStartup
		* \brief Startup for web sockets processes.
		*/
		static void GlobalStartup();

		/**
		* \fn GlobalCleanup
		* \brief Cleanup for web sockets processes.
		*/
		static void GlobalCleanup();

	private:
		/** Socket for own server. */
		SOCKET _ownServerSocket {~(unsigned __int64)0};

		/** Size of buffer for sending and receiving data. */
		size_t _bufferSize = 512;

		/** Buffer for sending and receiving data. */
		char *_receiveBuffer = new char[_bufferSize + 1];

		/** Map of connected sockets with keys-ids. */
		std::map<int, SOCKET> _connectedSockets;

		/** Free id for web socket. */
		int _freeSocketId = 0;

		/**
		* \fn SaveConnectedSocket
		* \brief Saves information about a connected socket.
		*
		* @param inSocket	Socket data.
		*
		* @return	Id of saved socket.
		*/
		int SaveConnectedSocket(SOCKET inSocket);

		/**
		* \fn GetConnectedSocket
		* \brief Gets information about a connected socket by its id.
		*
		* @param inSocketId	Socket id.
		*
		* @return	Data of saved socket.
		*/
		SOCKET GetConnectedSocket(int inSocketId);

		/**
		* \fn GetAllConnectedSocketsIds
		* \brief Gets ids of all connected sockets.
		*
		* @return	Ids of saved sockets.
		*/
		std::vector<int> GetAllConnectedSocketsIds();

		/**
		* \fn CheckSocket
		* \brief Checks socket data for validity.
		*
		* @param inSocket		Socket data.
		* @param inErrorMessage	Error message in case of invalid socket data.
		*/
		static void CheckSocket(SOCKET inSocket, const char* inErrorMessage);

		/**
		* \fn CheckWsResult
		* \brief Checks socket operation result for validity.
		*
		* @param inResult		Socket operation result.
		* @param inErrorMessage	Error message in case of invalid socket operation result.
		*/
		static void CheckWsResult(int inResult, const char* inErrorMessage);

		/**
		* \fn CheckTransferredBytes
		* \brief Checks number of bytes transferred for validity.
		*
		* @param inBytesNum		Number of bytes transferred.
		* @param inErrorMessage	Error message in case of invalid number of bytes transferred.
		*/
		static void CheckTransferredBytes(int inBytesNum, const char* inErrorMessage);
	};
}
