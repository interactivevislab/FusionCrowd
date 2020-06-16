#pragma once

#include "FcMainServerApi.h"
#include "WebNode.h"
#include "Export/Export.h"

#include <vector>
#include <map>


namespace FusionCrowdWeb
{
	using namespace FusionCrowd;

	class FC_MAIN_SERVER_API FcMainServer : public WebNode
	{
	public:
		void StartServer(WebAddress inAddress) override;

		void ConnectToComputationalServers(const std::vector<WebAddress>& inAddresses);
		void DisconnectFromComputationalServers();

		void AcceptClientConnection();
		void InitComputation();
		void ProcessComputationRequest();

	private:
		int _clientId;
		std::vector<int> _computationalServersIds;
		//const int _noComputationalServerId = -1;

		FCArray<AgentInfo> _allAgents = FCArray<AgentInfo>(0);
		std::vector<AgentInfo> _displacedAgents;
		std::map<int, std::map<size_t, size_t>> _agentsIds;	// <serverId, <agentIdOnServer, trueAgentId>>
		size_t _freeId = 0;

		std::map<int, NavMeshRegion> _navMeshRegions;
		const float _boundaryZoneDepth = 5;
	};
}
