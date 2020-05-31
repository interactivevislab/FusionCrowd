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


	void FusionCrowdClient::InitComputation(InitComputingData inInitData)
	{
		//stub
	}


	void FusionCrowdClient::RequestComputation(int inSimulationStepsNum, float inSimulationStep)
	{
		//stub
	}


	OutputComputingData FusionCrowdClient::GetComputationResult()
	{
		//stub
		return OutputComputingData();
	}
}