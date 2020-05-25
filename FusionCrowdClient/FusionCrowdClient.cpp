#include "FusionCrowdClient.h"

#include <string.h> 
#include <iostream>

#include "WsException.h"
#include "MessageCodes.h"


namespace FusionCrowdWeb
{
	void FusionCrowdClient::StartOn(const char* inIpAdress, short inPort)
	{
		_clientCore.Start();
		_clientCore.Connect(inIpAdress, inPort);

		std::cout << "Successfully connected to " << inIpAdress << ':' << inPort << std::endl << std::endl;
	}

	void FusionCrowdClient::Shutdown()
	{
		_clientCore.Shutdown();
	}
}