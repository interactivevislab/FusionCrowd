#include "FcMainServer.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FcMainServer::GlobalStartup();
	cout << "---Fusion Crowd Main Server---" << endl;

	FcMainServer server;
	server.StartServer(WebAddress("127.0.0.1", 8000));
	server.ConnectToComputationalServer(WebAddress("127.0.0.1", 8080));
	server.AcceptClientConnection();
	server.InitComputation();
	server.ProcessComputationRequest();

	system("pause");

	server.ShutdownServer();

	FcMainServer::GlobalCleanup();
	
	return 0;
}
