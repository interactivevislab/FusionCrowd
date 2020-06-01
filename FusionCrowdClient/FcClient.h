#pragma once

#include "FcClientApi.h"
#include "WebNode.h"
#include "FcWebData.h"


namespace FusionCrowdWeb
{
	//using namespace FusionCrowd;

	class FC_CLIENT_API FusionCrowdClient : public WebNode
	{
	public:
		void ConnectToMainServer(const char* inIpAdress, short inPort);
		void DisconnectFromMainServer();

		void InitComputation(const InitComputingData& inInitData);
		OutputComputingData RequestComputation(const InputComputingData& inComputingData);

	private:
		int _mainServerId;
	};
}