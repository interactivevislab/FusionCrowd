#include "FcClient.h"

#include "WsException.h"
#include "WebDataSerializer.h"
#include "WebMessage.h"

#include <string.h> 
#include <iostream>


namespace FusionCrowdWeb
{
	void FusionCrowdClient::ConnectToMainServer(WebAddress inAddress)
	{
		std::cout << "Connecting to " << inAddress.IpAddress << ':' << inAddress.Port << "... ";

		bool connected = false;
		while (!connected)
		{
			try
			{
				_mainServerId = ConnectToServer(inAddress);
				connected = true;
				std::cout << " success" << std::endl << std::endl;
			}
			catch (...)
			{
				//
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
		std::cout << "Init data sent" << std::endl;
	}


	OutputComputingData FusionCrowdClient::RequestComputation(const InputComputingData& inComputingData)
	{
		Send(_mainServerId, RequestCode::DoStep, inComputingData);
		std::cout << "Computing data sent" << std::endl;

		auto request = Receive(_mainServerId);
		if (request.first.AsResponseCode != ResponseCode::Success)
		{
			throw FcWebException("ResponseError");
		}
		auto result = WebDataSerializer<OutputComputingData>::Deserialize(request.second);
		std::cout << "Computing result received" << std::endl;

		return result;
	}
}