#include "FcClient.h"

#include "WsException.h"
#include "WebDataSerializer.h"
#include "WebMessage.h"


namespace FusionCrowdWeb
{
	void FusionCrowdClient::ConnectToMainServer(WebAddress inAddress)
	{
		bool connected = false;
		while (!connected)
		{
			try
			{
				_mainServerId = ConnectToServer(inAddress);
				connected = true;
			}
			catch (...)
			{
				//connect error - try again
			}
		}
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

		auto request = Receive(_mainServerId);
		if (request.first.AsResponseCode != ResponseCode::Success)
		{
			throw FcWebException("ResponseError");
		}
		auto result = WebDataSerializer<OutputComputingData>::Deserialize(request.second);

		return result;
	}
}