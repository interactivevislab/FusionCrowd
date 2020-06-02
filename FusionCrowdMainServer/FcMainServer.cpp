#include "FcMainServer.h"

#include "FcWebData.h"

#include <iostream>


namespace FusionCrowdWeb
{
	void FcMainServer::StartServer(WebAddress inAddress)
	{
		WebNode::StartServer(inAddress);
		std::cout << "Successfully started on " << inAddress.IpAddress << ':' << inAddress.Port << std::endl << std::endl;
	}


	void FcMainServer::ConnectToComputationalServer(WebAddress inAddress)
	{
		std::cout << "Connecting to " << inAddress.IpAddress << ':' << inAddress.Port << "... ";

		bool connected = false;
		while (!connected)
		{
			try
			{
				_computationalServerId = ConnectToServer(inAddress);
				connected = true;
				std::cout << " success" << std::endl << std::endl;
			}
			catch (...)
			{
				//
			}
		}
	}


	void FcMainServer::DisconnectFromComputationalServer()
	{
		Disconnect(_computationalServerId);
	}


	void FcMainServer::AcceptClientConnection()
	{
		_clientId = AcceptInputConnection();
		std::cout << "Client connected" << std::endl << std::endl;
	}


	void FcMainServer::InitComputation()
	{
		//stub

		auto request = Receive(_clientId);
		InitComputingData data = InitComputingData::Deserialize(request.second);
		std::cout << "Init data received - " << data.StubData << std::endl;

		char* rawData = (char*)std::malloc(sizeof(float));
		InitComputingData::Serialize(data, rawData);
		Send(_computationalServerId, RequestCode(0), rawData, sizeof(InitComputingData));
		std::cout << "Init data sent - " << data.StubData << std::endl;

		delete rawData;
	}


	void FcMainServer::ProcessComputationRequest()
	{
		//stub

		auto request = Receive(_clientId);
		InputComputingData inData = InputComputingData::Deserialize(request.second);
		std::cout << "Computing data received - " << inData.StubData << std::endl;

		char* rawData = (char*)std::malloc(sizeof(float));
		InputComputingData::Serialize(inData, rawData);
		Send(_computationalServerId, RequestCode(1), rawData, sizeof(InputComputingData));
		std::cout << "Computing data sent - " << inData.StubData << std::endl;

		request = Receive(_computationalServerId);
		OutputComputingData outData = OutputComputingData::Deserialize(request.second);
		std::cout << "Computing result received - " << outData.StubData << std::endl;

		OutputComputingData::Serialize(outData, rawData);
		Send(_clientId, RequestCode(2), rawData, sizeof(OutputComputingData));
		std::cout << "Computing result sent - " << outData.StubData << std::endl;

		delete rawData;
	}
}
