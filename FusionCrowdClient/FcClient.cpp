#include "FcClient.h"

#include <string.h> 
#include <iostream>

#include "WsException.h"
#include "WebMessage.h"


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
		char* rawData;
		auto dataSize = WebDataSerializer<InitComputingData>::Serialize(inInitData, rawData);
		Send(_mainServerId, RequestCode(0), rawData, dataSize);
		std::cout << "Init data sent" << std::endl;
		delete[] rawData;
	}


	OutputComputingData FusionCrowdClient::RequestComputation(const InputComputingData& inComputingData)
	{
		char* rawData;
		auto dataSize = WebDataSerializer<InputComputingData>::Serialize(inComputingData, rawData);
		Send(_mainServerId, RequestCode(1), rawData, dataSize);
		std::cout << "Computing data sent" << std::endl;
		delete[] rawData;

		auto request = Receive(_mainServerId);
		auto result = WebDataSerializer<OutputComputingData>::Deserialize(request.second);
		std::cout << "Computing result received" << std::endl;

		return result;
	}
}