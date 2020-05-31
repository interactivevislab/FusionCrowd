#include "FcMainServer.h"

#include <iostream>


namespace FusionCrowdWeb
{
	void FcMainServer::StartOn(const char* inIpAdress, short inPort)
	{
		_webNode.StartServer(inIpAdress, inPort);

		std::cout << "Successfully started on " << inIpAdress << ':' << inPort << std::endl << std::endl;

		//while (true)
		//{
		//	auto clientId = _webNode.AcceptInputConnection();
		//	std::cout << "Client connected" << std::endl << std::endl;

		//	while (true)
		//	{
		//		try
		//		{
		//			std::cout << "Waiting for request..." << std::endl;
		//			ProcessRequest(clientId);
		//		}
		//		catch (WsException e)
		//		{
		//			std::cout << "Client disconnected" << std::endl << std::endl;
		//			_webNode.Disconnect(clientId);
		//			break;
		//		}			
		//	}

		//	// some way to Shutdown()

		//}
	}


	void FcMainServer::Shutdown()
	{
		_webNode.ShutdownServer();
	}
}
