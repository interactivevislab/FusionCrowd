#include "FcMainServer.h"

#include "FcWebData.h"
#include "WebDataSerializer.h"
#include "WsException.h"

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
		if (request.first.AsRequestCode != RequestCode::InitSimulation)
		{
			throw FcWebException("RequestError");
		}
		auto data = WebDataSerializer<InitComputingData>::Deserialize(request.second);
		std::cout << "Init data received" << std::endl;

		//TODO: data processing

		Send(_computationalServerId, RequestCode::InitSimulation, data);
		std::cout << "Init data sent" << std::endl;
	}


	void FcMainServer::ProcessComputationRequest()
	{
		auto request = Receive(_clientId);
		if (request.first.AsRequestCode != RequestCode::DoStep)
		{
			throw FcWebException("RequestError");
		}
		auto inData = WebDataSerializer<InputComputingData>::Deserialize(request.second);
		std::cout << "Computing data received" << std::endl;

		//TODO: inData processing

		Send(_computationalServerId, RequestCode::DoStep, inData);
		std::cout << "Computing data sent" << std::endl;

		//waiting...

		request = Receive(_computationalServerId);
		if (request.first.AsResponseCode != ResponseCode::Success)
		{
			throw FcWebException("ResponseError");
		}
		auto outData = WebDataSerializer<OutputComputingData>::Deserialize(request.second);
		std::cout << "Computing result received" << std::endl;

		//TODO: outData processing

		Send(_clientId, ResponseCode::Success, outData);
		std::cout << "Computing result sent" << std::endl;
	}
}
