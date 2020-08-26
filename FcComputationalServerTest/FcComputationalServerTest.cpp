#include "FcComputationalServer.h"

#include <iostream>
#include <vector>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FcComputationalServer::GlobalStartup();
	cout << "---Fusion Crowd Computational Servers(2)---" << endl << endl;

	FcComputationalServer server1, server2;

	u_short port = 49001;
	server1.StartServer(port);
	cout << "1) Successfully started on localhost:" << port << endl;
	port =  49002;
	server2.StartServer(port);
	cout << "2) Successfully started on localhost:" << port << endl;

	server1.AcceptMainServerConnection();
	cout << "1) MainServer connected" << endl;
	server2.AcceptMainServerConnection();
	cout << "2) MainServer connected" << endl;

	cout << "1) Computations initialization... ";
	server1.InitComputation();
	cout << "success" << endl;
	cout << "2) Computations initialization... ";
	server2.InitComputation();
	cout << "success" << endl;

	cout << "1/2) Processing computation data... ";
	try
	{
		while (true)
		{
			server1.ProcessComputationRequest();
			server2.ProcessComputationRequest();
		}
	}
	catch (FusionCrowdWeb::FcWebException e)
	{
		cout << "success" << endl;
		//cout << "error: " << e.What() << endl;
		cout << "Main server disconnected" << endl;
	}

	server1.ShutdownServer();
	cout << "1) Server shutdown" << endl;
	server2.ShutdownServer();
	cout << "2) Server shutdown" << endl << endl;

	FcComputationalServer::GlobalCleanup();

	system("pause");

	return 0;
}
