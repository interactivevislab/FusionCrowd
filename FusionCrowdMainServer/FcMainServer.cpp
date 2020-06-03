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
		auto request = Receive(_clientId);
		auto data = WebDataSerializer<InitComputingData>::Deserialize(request.second);
		std::cout << "Init data received" << std::endl;

		//TODO: data processing

		char* rawData;
		auto dataSize = WebDataSerializer<InitComputingData>::Serialize(data, rawData);
		Send(_computationalServerId, RequestCode(0), rawData, dataSize);
		std::cout << "Init data sent" << std::endl;
		delete[] rawData;
	}


	void FcMainServer::ProcessComputationRequest()
	{
		auto request = Receive(_clientId);
		auto inData = WebDataSerializer<InputComputingData>::Deserialize(request.second);
		std::cout << "Computing data received" << std::endl;

		//TODO: inData processing

		char* rawData;
		auto dataSize = WebDataSerializer<InputComputingData>::Serialize(inData, rawData);
		Send(_computationalServerId, RequestCode(1), rawData, dataSize);
		std::cout << "Computing data sent" << std::endl;
		delete[] rawData;

		request = Receive(_computationalServerId);
		auto outData = WebDataSerializer<OutputComputingData>::Deserialize(request.second);
		std::cout << "Computing result received" << std::endl;

		//TODO: outData processing

		dataSize = WebDataSerializer<OutputComputingData>::Serialize(outData, rawData);
		Send(_clientId, RequestCode(2), rawData, dataSize);
		std::cout << "Computing result sent" << std::endl;
		delete[] rawData;
	}
}
