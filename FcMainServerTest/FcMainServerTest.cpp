#include "FcMainServer.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FcMainServer::GlobalStartup();
	cout << "---Fusion Crowd Main Server---" << endl << endl;

	FcMainServer server;
	server.StartServer(WebAddress("127.0.0.1", 49000));
	std::cout << "Successfully started" << std::endl << std::endl;

	std::vector<WebAddress> computationalServersAddresses {
		WebAddress("127.0.0.1", 49001),
		WebAddress("127.0.0.1", 49002)
	};
	std::cout << "Connecting to computational servers... ";
	server.ConnectToComputationalServers(computationalServersAddresses);
	std::cout << "success" << std::endl << std::endl;

	server.AcceptClientConnection();
	std::cout << "Client connected" << std::endl << std::endl;

	std::cout << "Processing init data... ";
	server.InitComputation();
	std::cout << "success" << std::endl << endl;

	try
	{
		while (true)
		{
			std::cout << "Processing computation data... ";
			server.ProcessComputationRequest();
			std::cout << "success" << std::endl;
		}
	}
	catch (FusionCrowdWeb::FcWebException e)
	{
		cout << "error: " << e.What() << endl << endl;
		cout << "Client disconnected" << endl << endl;
	}

	server.DisconnectFromComputationalServers();
	cout << "Disconnection from computational servers" << endl << endl;

	server.ShutdownServer();
	cout << "Server shutdown" << endl << endl;

	FcMainServer::GlobalCleanup();

	system("pause");
	
	return 0;
}
