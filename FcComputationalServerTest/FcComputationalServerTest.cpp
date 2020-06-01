#include "FcComputationalServer.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FcComputationalServer::GlobalStartup();
	cout << "---Fusion Crowd Computational Server---" << endl;

	FcComputationalServer server;

	server.StartServer("127.0.0.1", 8080);
	server.AcceptMainServerConnection();
	server.InitComputation();
	server.ProcessComputationRequest();

	system("pause");

	server.ShutdownServer();

	FcComputationalServer::GlobalCleanup();

	return 0;
}
