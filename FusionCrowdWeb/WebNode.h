#pragma once

#include "FcWebApi.h"

#include "WsException.h"
#include "WebDataSerializer.h"
#include "WebMessage.h"

#include <WinSock2.h>


namespace FusionCrowdWeb
{
	/**
	* \struct WebAddress
	* \brief Address for web socket.
	*/
	struct FC_WEB_API WebAddress
	{
		/** IPv4 address part as string. */
		char* IpAddress = nullptr;

		/** Port address part. */
		u_short Port;

		WebAddress(const char* inIpAddress, u_short inPort);
		~WebAddress();
		WebAddress(WebAddress const& inOther) noexcept;
		WebAddress& operator=(WebAddress const& inOther) noexcept;
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
		* @param inPort	Socket port to start.
		*/
		void StartServer(u_short inPort);

		/**
		* \fn ShutdownServer
		* \brief Shutdown server on this node.
		*/
		void ShutdownServer();

		/**
		* \fn AcceptInputConnection
		* \brief Waits and accepts incoming connection from other node.
		*
		* @return	Connected socket.
		*/
		SOCKET AcceptInputConnection();

		/**
		* \fn TryConnectToServer
		* \brief Once trying to connect to another node's server.
		*
		* @param inAddress	Socket address of other node's server.
		*
		* @return	Connected socket id.
		*/
		SOCKET TryConnectToServer(WebAddress inAddress);

		/**
		* \fn TryConnectToServer
		* \brief Repeatedly trying to connect to another node's server until success.
		*
		* @param inAddress				Socket address of other node's server.
		* @param inConnectionTimeout	Connection timeout in seconds.
		*
		* @return	Connected socket.
		*/
		SOCKET WaitForConnectionToServer(WebAddress inAddress, float inConnectionTimeout);

		/**
		* \fn TryConnectToServer
		* \brief Disconnects from other node.
		*
		* @param inSocket	Socket to disconnect.
		*/
		void Disconnect(SOCKET inSocket);

		/**
		* \fn Send
		* \brief Sends data to other node.
		*
		* @param inSocket	Socket to send data.
		* @param inWebCode	WebCode for short data description.
		* @param inData		Data to send.
		* @param inDataSize	Size of data to send.
		*/
		void Send(SOCKET inSocket, WebCode inWebCode, const char* inData, size_t inDataSize);

		/**
		* \fn Send
		* \brief Sends data to other node.
		*
		* @param inSocket	Socket to send data.
		* @param inWebCode	WebCode for short data description.
		*/
		void Send(SOCKET inSocket, WebCode inWebCode);

		/**
		* \fn Send
		* \brief Sends data to other node.
		*
		* @param inSocket	Socket to send data.
		* @param inWebCode	WebCode for short data description.
		* @param inData		Data to send.
		*/
		template<typename DataType>
		void Send(SOCKET inSocket, WebCode inWebCode, const DataType& inData)
		{
			char* rawData;
			auto dataSize = WebDataSerializer<DataType>::Serialize(inData, rawData);
			Send(inSocket, inWebCode, rawData, dataSize);
			delete[] rawData;
		}

		/**
		* \fn Receive
		* \brief Receives data from other node.
		*
		* @param inSocket	Socket to receive data from.
		*
		* @return	Received data.
		*/
		WebMessage Receive(SOCKET inSocket);

		/**
		* \fn Receive
		* \brief Receives data from other node.
		*
		* @param inSocket			Socket to receive data from.
		* @param inExpectedWebCode	WebCode which is expected to be received.
		* @param inErrorMessage		Error message in case of receipt unexpected WebCode.
		*
		* @return	Received data.
		*/
		template<typename DataType>
		DataType Receive(SOCKET inSocket, WebCode inExpectedWebCode, char const* inErrorMessage)
		{
			auto message = Receive(inSocket);
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
		SOCKET _ownServerSocket { INVALID_SOCKET };

		/** Size of buffer for sending and receiving data. */
		size_t _bufferSize = 512;

		/** Buffer for sending and receiving data. */
		char *_receiveBuffer = new char[_bufferSize + 1];

		/** Free id for web socket. */
		int _freeSocketId = 0;

		/** Flag to determine if Windows Sockets API is running. */
		static bool _isWsaStarted;

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
