#pragma once

#include "FcMainServerApi.h"
#include "WebNode.h"
#include "Export/Export.h"


namespace FusionCrowdWeb
{
	using namespace FusionCrowd;

	class FC_MAIN_SERVER_API FcMainServer : public WebNode
	{
	public:
		void StartServer(WebAddress inAddress) override;

		void ConnectToComputationalServer(WebAddress inAddress);
		void DisconnectFromComputationalServer();

		void AcceptClientConnection();
		void InitComputation();
		void ProcessComputationRequest();

	private:
		int _computationalServerId;
		int _clientId;
	};
}
