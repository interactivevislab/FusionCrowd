#include "FcMainServer.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FcMainServer::GlobalStartup();
	cout << "---Fusion Crowd Main Server---" << endl;

	FcMainServer server;

	server.ConnectToComputationalServer("127.0.0.1", 8000);;
	server.AcceptClientConnection();
	server.InitComputation();
	server.ProcessComputationRequest();

	server.ShutdownServer();

	FcMainServer::GlobalCleanup();

	system("pause");
	return 0;
}
