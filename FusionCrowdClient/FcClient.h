#pragma once

#include "FcClientApi.h"
#include "WebNode.h"
#include "FcWebData.h"


namespace FusionCrowdWeb
{
	/**
	* \class FusionCrowdClient
	* \brief Client side of remote computing multi-agent simulation.
	*
	* @see WebNode
	*/
	class FC_CLIENT_API FusionCrowdClient : public WebNode
	{
	public:
		/**
		* \fn ConnectToMainServer
		* \brief Establishes a connection with center of computing network.
		*
		* @param inAddress	Main server's address.
		*/
		void ConnectToMainServer(WebAddress inAddress);

		/**
		* \fn DisconnectFromMainServer
		* \brief Breaks the connection with computing network.
		*/
		void DisconnectFromMainServer();

		/**
		* \fn InitComputation
		* \brief Requests simulation initialization from computing network.
		*
		* @param inInitData	Data for simulation initialization.
		*/
		void InitComputation(const InitComputingData& inInitData);

		/**
		* \fn InitComputation
		* \brief Requests performing simulation step from computing network.
		*
		* @param inComputingData	Data for simulation step.
		*
		* @return Simulation results.
		*/
		OutputComputingData RequestComputation(const InputComputingData& inComputingData);

	private:
		/** Main server socket. */
		SOCKET _mainServerSocket { INVALID_SOCKET };
	};
}
