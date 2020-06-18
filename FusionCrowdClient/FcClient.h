#pragma once

#include "FcClientApi.h"
#include "WebNode.h"
#include "FcWebData.h"


namespace FusionCrowdWeb
{
	class FC_CLIENT_API FusionCrowdClient : public WebNode
	{
	public:
		void ConnectToMainServer(WebAddress inAddress);
		void DisconnectFromMainServer();

		void InitComputation(const InitComputingData& inInitData);
		OutputComputingData RequestComputation(const InputComputingData& inComputingData);

	private:
		int _mainServerId;
	};
}
