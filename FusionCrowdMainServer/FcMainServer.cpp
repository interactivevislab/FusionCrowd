#include "FcMainServer.h"

#include <iostream>


namespace FusionCrowdWeb
{
	void FcMainServer::StartServer(const char* inIpAdress, short inPort)
	{
		WebNode::StartServer(inIpAdress, inPort);
		std::cout << "Successfully started on " << inIpAdress << ':' << inPort << std::endl << std::endl;
	}


	void FcMainServer::ConnectToComputationalServer(const char* inIpAdress, short inPort)
	{
		std::cout << "Connecting to " << inIpAdress << ':' << inPort << "... ";

		bool connected = false;
		while (!connected)
		{
			try
			{
				_computationalServerId = ConnectToServer(inIpAdress, inPort);
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
	}


	void FcMainServer::ProcessComputationRequest()
	{
		//stub
	}
}
