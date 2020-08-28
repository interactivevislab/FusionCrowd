#include "FcClient.h"

#include "WsException.h"
#include "WebDataSerializer.h"
#include "WebMessage.h"


namespace FusionCrowdWeb
{
	void FusionCrowdClient::ConnectToMainServer(WebAddress inAddress, float inConnectionTimeout)
	{
		_mainServerSocket = WaitForConnectionToServer(inAddress, inConnectionTimeout);
	}


	void FusionCrowdClient::DisconnectFromMainServer()
	{
		Disconnect(_mainServerSocket);
	}


	void FusionCrowdClient::InitComputation(const InitComputingData& inInitData)
	{
		Send(_mainServerSocket, RequestCode::InitSimulation, inInitData);
	}


	OutputComputingData FusionCrowdClient::RequestComputation(const InputComputingData& inComputingData)
	{
		Send(_mainServerSocket, RequestCode::DoStep, inComputingData);

		auto result = Receive<OutputComputingData>(_mainServerSocket, ResponseCode::Success, "ResponseError");

		return result;
	}
}
