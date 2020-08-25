#include "FcClient.h"

#include "WsException.h"
#include "WebDataSerializer.h"
#include "WebMessage.h"


namespace FusionCrowdWeb
{
	void FusionCrowdClient::ConnectToMainServer(WebAddress inAddress)
	{
		_mainServerId = WaitForConnectionToServer(inAddress);
	}


	void FusionCrowdClient::DisconnectFromMainServer()
	{
		Disconnect(_mainServerId);
	}


	void FusionCrowdClient::InitComputation(const InitComputingData& inInitData)
	{
		Send(_mainServerId, RequestCode::InitSimulation, inInitData);
	}


	OutputComputingData FusionCrowdClient::RequestComputation(const InputComputingData& inComputingData)
	{
		Send(_mainServerId, RequestCode::DoStep, inComputingData);

		auto result = Receive<OutputComputingData>(_mainServerId, ResponseCode::Success, "ResponseError");

		return result;
	}
}
