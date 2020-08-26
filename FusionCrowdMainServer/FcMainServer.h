#pragma once

#include "FcMainServerApi.h"
#include "WebNode.h"
#include "Export/Export.h"

#include <vector>
#include <map>
#include <memory>


namespace FusionCrowdWeb
{
	/**
	* \class FcMainServer
	* \brief Central node of computing network that distributes data between computing servers.
	*
	* @see WebNode
	*/
	class FC_MAIN_SERVER_API FcMainServer : public WebNode
	{
	public:
		/**
		* \fn ConnectToComputationalServers
		* \brief Establishes a connections with computing servers.
		*
		* @param inAddresses	Computing servers' addresses.
		*/
		void ConnectToComputationalServers(const std::vector<WebAddress>& inAddresses);

		/**
		* \fn DisconnectFromComputationalServers
		* \brief Breaks the connections with computing servers.
		*/
		void DisconnectFromComputationalServers();

		/**
		* \fn AcceptClientConnection
		* \brief Waits and accepts incoming connection from client.
		*/
		void AcceptClientConnection();

		/**
		* \fn InitComputation
		* \brief Processes simulation initialization request from client and passes it to computing servers.
		*/
		void InitComputation();

		/**
		* \fn InitComputation
		* \brief Processes performing simulation step request from client and passes it to computing servers.
		*/
		void ProcessComputationRequest();

		/**
		* \fn SaveRecording
		* \brief Serializes recording of current simulation into file.
		*
		* @param inRecordingFileName	Result file name.
		*/
		void SaveRecording(std::string inRecordingFileName);

		/**
		* \fn StartOrdinaryRun
		* \brief Runs work according to the usual scenario.
		*
		* @param inPort	Socket port to start.
		* @param computationalServersAddresses	Computing servers' addresses.
		*/
		void StartOrdinaryRun(u_short inPort, const std::vector<WebAddress>& computationalServersAddresses);

	private:
		/** Id of client socket. */
		int _clientId = -1;

		/** Ids of computational servers' sockets. */
		std::vector<int> _computationalServersIds;

		/** Copy of all agents fron previous simulation step. */
		FusionCrowd::FCArray<FusionCrowd::AgentInfo> _allAgents = FusionCrowd::FCArray<FusionCrowd::AgentInfo>(0);

		/** Copy of agents which need to be moved between computing servers. */
		std::vector<FusionCrowd::AgentInfo> _displacedAgents;

		/**
		* \fn _agentsIds
		* \brief Dictionary for translating local Id (on computing servers) to global (on main server).
		*
		* Key of outer map - id of computing server.
		* Key of inner map - id of agent on computing server.
		* Value of inner map - id of agent on main server.
		*/
		std::map<int, std::map<size_t, size_t>> _agentsIds;

		/** Free global id for agent. */
		size_t _freeId = 0;

		/** Areas of NavMesh related to computing servers. */
		std::map<int, NavMeshRegion> _navMeshRegions;

		/** NavMesh regions boundary zone size. */
		float _boundaryZoneDepth = 5;

		/** Recording of current simulation. */
		std::shared_ptr<FusionCrowd::IRecording> _recording;
	};
}
