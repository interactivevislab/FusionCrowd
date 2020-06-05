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

	auto address = WebAddress("127.0.0.1", 49001);
	server1.StartServer(address);
	cout << "1) Successfully started on " << address.IpAddress << ':' << address.Port << endl;
	address = WebAddress("127.0.0.1", 49002);
	server2.StartServer(address);
	cout << "2) Successfully started on " << address.IpAddress << ':' << address.Port << endl << endl;

	server1.AcceptMainServerConnection();
	cout << "1) MainServer connected" << endl;
	server2.AcceptMainServerConnection();
	cout << "2) MainServer connected" << endl << endl;

	cout << "1) Computations initialization... ";
	server1.InitComputation();
	cout << "success" << endl;
	cout << "2) Computations initialization... ";
	server2.InitComputation();
	cout << "success" << endl << endl;

	try
	{
		while (true)
		{
			cout << "1) Processing computation data... ";
			server1.ProcessComputationRequest();
			cout << "success" << endl;
			cout << "2) Processing computation data... ";
			server2.ProcessComputationRequest();
			cout << "success" << endl;
		}
	}
	catch (FusionCrowdWeb::FcWebException e)
	{
		cout << "error: " << e.What() << endl << endl;
		cout << "Main server disconnected" << endl << endl;
	}

	server1.ShutdownServer();
	cout << "1) Server shutdown" << endl;
	server2.ShutdownServer();
	cout << "2) Server shutdown" << endl << endl;

	FcComputationalServer::GlobalCleanup();

	system("pause");

	return 0;
}
