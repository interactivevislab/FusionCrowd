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
		std::vector<int> _computationalServersIds;
		std::map<int, NavMeshRegion> _navMeshRegions;

		int _clientId;
	};
}
