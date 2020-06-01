#include "FcClient.h"

#include <string.h> 
#include <iostream>

#include "WsException.h"
#include "WebMessage.h"


namespace FusionCrowdWeb
{
	void FusionCrowdClient::ConnectToMainServer(const char* inIpAdress, short inPort)
	{
		std::cout << "Connecting to " << inIpAdress << ':' << inPort << "... ";

		bool connected = false;
		while (!connected)
		{
			try
			{
				_mainServerId = ConnectToServer(inIpAdress, inPort);
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
		//stub

		char* rawData = (char*)std::malloc(sizeof(float));
		InitComputingData::Serialize(inInitData, rawData);

		Send(_mainServerId, RequestCode(0), rawData, sizeof(InitComputingData));
		std::cout << "Init data sent - " << inInitData.StubData << std::endl;

		delete rawData;
	}


	OutputComputingData FusionCrowdClient::RequestComputation(const InputComputingData& inComputingData)
	{
		//stub

		char* rawData = (char*)std::malloc(sizeof(float));
		InputComputingData::Serialize(inComputingData, rawData);

		Send(_mainServerId, RequestCode(1), rawData, sizeof(InputComputingData));
		std::cout << "Computing data sent - " << inComputingData.StubData << std::endl;

		delete rawData;

		auto request = Receive(_mainServerId);
		OutputComputingData result = OutputComputingData::Deserialize(request.second);
		std::cout << "Computing result received - " << result.StubData << std::endl;

		return result;
	}
}