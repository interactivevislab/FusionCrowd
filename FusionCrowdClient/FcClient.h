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

		void InitComputation(InitComputingData inInitData);
		void RequestComputation(int inSimulationStepsNum, float inSimulationStep);
		OutputComputingData GetComputationResult();

	private:
		int _mainServerId;
	};
}