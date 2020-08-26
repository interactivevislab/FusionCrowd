#pragma once

#include "FcComputationalServerApi.h"
#include "Export/Export.h"
#include "WebNode.h"

#include <memory>


namespace FusionCrowdWeb
{
	/**
	* \class FcComputationalServer
	* \brief Server side of remote computing multi-agent simulation.
	*
	* @see WebNode
	*/
	class FC_COMPUTATIONAL_SERVER_API FcComputationalServer : public WebNode
	{
	public:
		/**
		* \fn AcceptMainServerConnection
		* \brief Waits and accepts incoming connection from main server.
		*/
		void AcceptMainServerConnection();

		/**
		* \fn InitComputation
		* \brief Receives initialization request from main server and performs it.
		*/
		void InitComputation();

		/**
		* \fn InitComputation
		* \brief Receives computation request from main server and performs it.
		*/
		void ProcessComputationRequest();

		/**
		* \fn StartOrdinaryRun
		* \brief Runs work according to the usual scenario.
		*
		* @param inPort	Socket port to start.
		*/
		void StartOrdinaryRun(u_short inPort);

	private:
		/** Id of main server socket. */
		int _mainServerId = -1;

		/** ISimulatorFacade builder. */
		std::shared_ptr<FusionCrowd::ISimulatorBuilder> _builder;

		/** Crowd simulator. */
		std::shared_ptr<FusionCrowd::ISimulatorFacade> _simulator;

		/** Area of NavMesh related to this server. */
		NavMeshRegion _navMeshRegion;
	};
}
