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
	cout << "Successfully started" << endl;

	vector<WebAddress> computationalServersAddresses {
		WebAddress("127.0.0.1", 49001),
		WebAddress("127.0.0.1", 49002)
	};
	cout << "Connecting to computational servers... ";
	server.ConnectToComputationalServers(computationalServersAddresses);
	cout << "success" << endl;

	server.AcceptClientConnection();
	cout << "Client connected" << endl;

	cout << "Processing init data... ";
	server.InitComputation();
	cout << "success" << endl;

	cout << "Processing computation data... ";
	try
	{
		while (true)
		{
			server.ProcessComputationRequest();
		}
	}
	catch (FusionCrowdWeb::FcWebException e)
	{
		cout << "success" << endl;
		//cout << "error: " << e.What() << endl;
		cout << "Client disconnected" << endl;
	}

	server.DisconnectFromComputationalServers();
	cout << "Disconnection from computational servers" << endl;

	server.ShutdownServer();
	cout << "Server shutdown" << endl << endl;

	FcMainServer::GlobalCleanup();

	system("pause");
	
	return 0;
}
