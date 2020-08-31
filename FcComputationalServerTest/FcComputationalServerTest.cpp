#include "FcComputationalServer.h"

#include <iostream>
#include <vector>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FcComputationalServer::GlobalStartup();
	cout << "---Fusion Crowd Computational Server---" << endl << endl;

	u_short port = 49001;

	bool isVerboseRun = false;
	if (!isVerboseRun)
	{
		try
		{
			FcComputationalServer server;
			cout << "Ordinary run starts on localhost:" << port << endl;
			server.StartOrdinaryRun(port);
			cout << "Ordinary run ends" << endl << endl;
		}
		catch(FcWebException exception)
		{
			cout << "FcWebException: " << exception.What() << endl << endl;
		}
	}
	else
	{
		FcComputationalServer server;
		
		server.StartServer(port);
		cout << "Successfully started on localhost:" << port << endl;

		server.AcceptMainServerConnection();
		cout << "MainServer connected" << endl;

		cout << "Computations initialization... ";
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
			cout << "Main server disconnected" << endl;
		}

		server.ShutdownServer();
		cout << "Server shutdown" << endl << endl;
	}

	FcComputationalServer::GlobalCleanup();

	//system("pause");

	return 0;
}
